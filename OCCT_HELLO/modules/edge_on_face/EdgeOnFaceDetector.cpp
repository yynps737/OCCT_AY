#include "EdgeOnFaceDetector.h"
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Solid.hxx>
#include <BRep_Tool.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <Geom_Surface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <TopLoc_Location.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <TopTools_ListOfShape.hxx>
#include <iostream>
#include <iomanip>

namespace EdgeOnFace {

    // 构建拓扑关系
    void Detector::buildTopology(const TopoDS_Shape& shape) {
        edgeToFaces.Clear();
        faces.Clear();
        edges.Clear();
        solids.Clear();

        TopExp::MapShapesAndUniqueAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);
        TopExp::MapShapes(shape, TopAbs_FACE, faces);
        TopExp::MapShapes(shape, TopAbs_EDGE, edges);
        TopExp::MapShapes(shape, TopAbs_SOLID, solids);
    }

    // 构建实体归属映射
    void Detector::buildSolidOwnership(const TopoDS_Shape& shape) {
        edgeToSolid.clear();
        faceToSolid.clear();
        solidNames.clear();

        // 遍历每个实体
        for (int solidIdx = 1; solidIdx <= solids.Extent(); ++solidIdx) {
            const TopoDS_Solid& solid = TopoDS::Solid(solids(solidIdx));

            // 为实体生成名称
            solidNames[solidIdx] = "Solid_" + std::to_string(solidIdx);

            // 记录该实体的所有面
            TopExp_Explorer faceExp(solid, TopAbs_FACE);
            for (; faceExp.More(); faceExp.Next()) {
                const TopoDS_Face& face = TopoDS::Face(faceExp.Current());
                int faceIdx = faces.FindIndex(face);
                if (faceIdx > 0) {
                    faceToSolid[faceIdx] = solidIdx;
                }
            }

            // 记录该实体的所有边
            TopExp_Explorer edgeExp(solid, TopAbs_EDGE);
            for (; edgeExp.More(); edgeExp.Next()) {
                const TopoDS_Edge& edge = TopoDS::Edge(edgeExp.Current());
                int edgeIdx = edges.FindIndex(edge);
                if (edgeIdx > 0) {
                    edgeToSolid[edgeIdx] = solidIdx;
                }
            }
        }

        std::cout << "Built solid ownership for " << solids.Extent() << " solids" << std::endl;
    }

    // 获取边的相邻面
    std::set<int> Detector::getAdjacentFaces(int edgeIndex) {
        std::set<int> adjacentFaces;

        if (edgeIndex <= edgeToFaces.Extent()) {
            const TopTools_ListOfShape& faceList = edgeToFaces(edgeIndex);
            for (TopTools_ListOfShape::Iterator it(faceList); it.More(); it.Next()) {
                int faceIndex = faces.FindIndex(it.Value());
                if (faceIndex > 0) {
                    adjacentFaces.insert(faceIndex);
                }
            }
        }

        return adjacentFaces;
    }

    // 核心算法: 检查边是否在面上
    bool Detector::isEdgeOnFace(const TopoDS_Edge& edge, const TopoDS_Face& face,
                                double& matchPercentage) {

        // 创建边的曲线适配器
        BRepAdaptor_Curve curveAdaptor(edge);

        // 生成均匀采样点
        GCPnts_UniformAbscissa sampler(curveAdaptor, config.samplePoints);
        if (!sampler.IsDone() || sampler.NbPoints() < 2) {
            return false;
        }

        // 获取面的表面
        TopLoc_Location faceLoc;
        Handle(Geom_Surface) surface = BRep_Tool::Surface(face, faceLoc);
        if (surface.IsNull()) {
            return false;
        }

        // 检查每个采样点
        int pointsOnFace = 0;
        int totalPoints = sampler.NbPoints();

        for (int i = 1; i <= totalPoints; ++i) {
            gp_Pnt point = curveAdaptor.Value(sampler.Parameter(i));

            // 投影点到面
            GeomAPI_ProjectPointOnSurf projector(point, surface);
            if (!projector.IsDone() || projector.NbPoints() == 0) {
                continue;
            }

            // 检查投影距离
            if (projector.LowerDistance() <= config.tolerance) {
                // 获取UV坐标
                Standard_Real u, v;
                projector.LowerDistanceParameters(u, v);

                // 检查点是否在面的边界内
                BRepClass_FaceClassifier classifier(face, gp_Pnt2d(u, v), config.tolerance);
                TopAbs_State state = classifier.State();

                if (state == TopAbs_IN || state == TopAbs_ON) {
                    pointsOnFace++;
                }
            }
        }

        // 计算匹配百分比
        matchPercentage = 100.0 * pointsOnFace / totalPoints;

        // 判断是否足够多的点在面上
        return (static_cast<double>(pointsOnFace) / totalPoints >= config.matchThreshold);
    }

    // 主分析函数
    std::vector<EdgeFaceRelation> Detector::analyze(const TopoDS_Shape& shape) {
        std::vector<EdgeFaceRelation> results;

        // 构建拓扑
        buildTopology(shape);

        // 构建实体归属映射
        buildSolidOwnership(shape);

        std::cout << "\nPerforming edge-on-face detection..." << std::endl;
        std::cout << "Analyzing " << solids.Extent() << " solids, "
                  << edges.Extent() << " edges, and "
                  << faces.Extent() << " faces" << std::endl;

        // 遍历每条边
        for (int edgeIdx = 1; edgeIdx <= edges.Extent(); ++edgeIdx) {
            const TopoDS_Edge& edge = TopoDS::Edge(edges(edgeIdx));

            // 计算边长用于调试
            BRepAdaptor_Curve curveAdaptor(edge);
            double edgeLength = GCPnts_AbscissaPoint::Length(curveAdaptor);

            // 获取该边所属的实体
            int edgeSolidId = (edgeToSolid.count(edgeIdx) > 0) ? edgeToSolid[edgeIdx] : 0;

            // 获取相邻面
            std::set<int> adjacentFaces = getAdjacentFaces(edgeIdx);

            // 只对跨实体的边进行详细检查
            bool checkThisEdge = false;

            // 检查与所有非相邻面
            for (int faceIdx = 1; faceIdx <= faces.Extent(); ++faceIdx) {
                // 跳过相邻面
                if (adjacentFaces.count(faceIdx) > 0) continue;

                // 获取该面所属的实体
                int faceSolidId = (faceToSolid.count(faceIdx) > 0) ? faceToSolid[faceIdx] : 0;

                // 跳过同一实体内的边面关系（只检测不同实体间的接触）
                if (edgeSolidId > 0 && faceSolidId > 0 && edgeSolidId == faceSolidId) {
                    continue;
                }

                checkThisEdge = true;

                const TopoDS_Face& face = TopoDS::Face(faces(faceIdx));

                double matchPercentage;
                if (isEdgeOnFace(edge, face, matchPercentage)) {
                    EdgeFaceRelation relation;
                    relation.edgeId = edgeIdx;
                    relation.edgeSolidId = edgeSolidId;
                    relation.edgeSolidName = (edgeSolidId > 0) ? solidNames[edgeSolidId] : "Unknown";
                    relation.faceId = faceIdx;
                    relation.faceSolidId = faceSolidId;
                    relation.faceSolidName = (faceSolidId > 0) ? solidNames[faceSolidId] : "Unknown";
                    relation.matchPercentage = matchPercentage;
                    relation.adjacentFaceIds = adjacentFaces;

                    results.push_back(relation);
                }
            }

            // 调试输出：显示可能遗漏的115mm边
            if (checkThisEdge && std::abs(edgeLength - 115.0) < 5.0) {
                std::cout << "Debug: Edge" << edgeIdx << " (Solid_" << edgeSolidId
                          << ") has length " << edgeLength << "mm but no edge-on-face match found" << std::endl;
            }
        }

        return results;
    }

    // 按实体分组接触关系
    std::vector<SolidContactInfo> Detector::groupBySolids(const std::vector<EdgeFaceRelation>& results) {
        std::map<std::pair<int, int>, SolidContactInfo> contactMap;

        for (const auto& rel : results) {
            // 创建实体对的键（始终让较小的ID在前）
            int solid1 = std::min(rel.edgeSolidId, rel.faceSolidId);
            int solid2 = std::max(rel.edgeSolidId, rel.faceSolidId);
            auto key = std::make_pair(solid1, solid2);

            // 如果这对实体还没有记录，创建新的
            if (contactMap.find(key) == contactMap.end()) {
                SolidContactInfo info;
                info.solid1Id = solid1;
                info.solid1Name = (solid1 > 0 && solidNames.count(solid1)) ? solidNames[solid1] : "Unknown";
                info.solid2Id = solid2;
                info.solid2Name = (solid2 > 0 && solidNames.count(solid2)) ? solidNames[solid2] : "Unknown";
                contactMap[key] = info;
            }

            // 添加这个接触关系
            contactMap[key].contacts.push_back(rel);
        }

        // 转换为向量
        std::vector<SolidContactInfo> solidContacts;
        for (const auto& pair : contactMap) {
            solidContacts.push_back(pair.second);
        }

        return solidContacts;
    }

    // 显示分析结果
    void displayResults(const std::vector<EdgeFaceRelation>& results) {
        if (results.empty()) {
            std::cout << "\nNo edges found lying on non-adjacent faces." << std::endl;
            std::cout << "This means no inter-part contact detected through edge-face relationships." << std::endl;
            return;
        }

        std::cout << "\nFound " << results.size() << " edge-face relationships:\n" << std::endl;

        std::cout << std::left
                  << std::setw(15) << "Edge(Solid)"
                  << std::setw(15) << "Face(Solid)"
                  << std::setw(10) << "Match%" << std::endl;
        std::cout << std::string(40, '-') << std::endl;

        // 显示前20个结果
        size_t displayCount = std::min(results.size(), size_t(20));
        for (size_t i = 0; i < displayCount; ++i) {
            const auto& rel = results[i];

            // 格式化边信息
            std::string edgeInfo = "E" + std::to_string(rel.edgeId) +
                                  "(" + rel.edgeSolidName + ")";

            // 格式化面信息
            std::string faceInfo = "F" + std::to_string(rel.faceId) +
                                  "(" + rel.faceSolidName + ")";

            std::cout << std::setw(15) << edgeInfo
                      << std::setw(15) << faceInfo
                      << std::setw(10) << std::fixed << std::setprecision(1)
                      << rel.matchPercentage << std::endl;
        }

        if (results.size() > displayCount) {
            std::cout << "... (" << (results.size() - displayCount)
                      << " more)" << std::endl;
        }
    }

    // 显示详细的实体间接触信息
    void displayDetailedResults(const std::vector<EdgeFaceRelation>& results,
                                const std::vector<SolidContactInfo>& solidContacts) {

        std::cout << "\n========================================" << std::endl;
        std::cout << "SOLID-TO-SOLID CONTACT ANALYSIS" << std::endl;
        std::cout << "========================================" << std::endl;

        if (solidContacts.empty()) {
            std::cout << "No inter-solid contacts detected." << std::endl;
            return;
        }

        // 显示每对实体间的接触
        for (const auto& contact : solidContacts) {
            std::cout << "\n" << contact.solid1Name << " <--> " << contact.solid2Name << std::endl;
            std::cout << "Number of contacts: " << contact.contacts.size() << std::endl;
            std::cout << std::string(40, '-') << std::endl;

            // 显示具体的接触细节
            for (const auto& rel : contact.contacts) {
                std::cout << "  ";

                // 根据边和面的归属确定方向
                if (rel.edgeSolidId == contact.solid1Id) {
                    std::cout << contact.solid1Name << ".Edge" << rel.edgeId
                             << " --> "
                             << contact.solid2Name << ".Face" << rel.faceId;
                } else {
                    std::cout << contact.solid2Name << ".Edge" << rel.edgeId
                             << " --> "
                             << contact.solid1Name << ".Face" << rel.faceId;
                }

                std::cout << " (Match: " << std::fixed << std::setprecision(1)
                         << rel.matchPercentage << "%)" << std::endl;

                // 显示该边的相邻面
                if (!rel.adjacentFaceIds.empty()) {
                    std::cout << "    Adjacent faces of edge: ";
                    bool first = true;
                    for (int adjFace : rel.adjacentFaceIds) {
                        if (!first) std::cout << ", ";
                        std::cout << "F" << adjFace;
                        first = false;
                    }
                    std::cout << std::endl;
                }
            }
        }

        // 显示总结
        std::cout << "\n========================================" << std::endl;
        std::cout << "SUMMARY" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Total solid pairs in contact: " << solidContacts.size() << std::endl;
        std::cout << "Total edge-face contacts: " << results.size() << std::endl;
    }
}