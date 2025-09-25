#include "JointDetector.h"
#include "joint_type/ButtJointDetector.h"
#include "joint_type/CornerJointDetector.h"
#include "joint_type/TJointDetector.h"
#include "joint_type/LapJointDetector.h"
#include "joint_type/EdgeJointDetector.h"

#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Solid.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Geom_Surface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
#include <gp_Pnt.hxx>
#include <iostream>
#include <iomanip>
#include <cmath>

JointDetector::JointDetector() {
    // 创建各类型检测器
    buttDetector = new ButtJointDetector();
    cornerDetector = new CornerJointDetector();
    tDetector = new TJointDetector();
    lapDetector = new LapJointDetector();
    edgeDetector = new EdgeJointDetector();

    // 默认配置
    config.detectButtJoint = true;
    config.detectCornerJoint = true;
    config.detectTJoint = true;
    config.detectLapJoint = true;
    config.detectEdgeJoint = false;
    config.angleToleranceDeg = 15.0;
}

JointDetector::~JointDetector() {
    delete buttDetector;
    delete cornerDetector;
    delete tDetector;
    delete lapDetector;
    delete edgeDetector;
}

std::vector<JointDetector::JointData> JointDetector::detectJoints(const TopoDS_Shape& shape) {
    // 初始化拓扑数�?
    initialize(shape);

    std::cout << "\nDetecting welding joints based on standard types..." << std::endl;
    std::cout << "Configuration:" << std::endl;
    std::cout << "  Butt Joint: " << (config.detectButtJoint ? "ON" : "OFF") << std::endl;
    std::cout << "  Corner Joint (L): " << (config.detectCornerJoint ? "ON" : "OFF") << std::endl;
    std::cout << "  T-Joint: " << (config.detectTJoint ? "ON" : "OFF") << std::endl;
    std::cout << "  Lap Joint: " << (config.detectLapJoint ? "ON" : "OFF") << std::endl;
    std::cout << "  Edge Joint: " << (config.detectEdgeJoint ? "ON" : "OFF") << std::endl;

    // 根据实体数量选择检测模�?
    if (solids.Extent() == 1) {
        std::cout << "\nSingle-part model detected. Analyzing internal joints..." << std::endl;
        return detectJointsInSinglePart(shape);
    } else {
        std::cout << "\nMulti-part model detected (" << solids.Extent()
                  << " parts). Analyzing inter-part joints..." << std::endl;
        return detectJointsInMultiPart(shape);
    }
}

std::vector<JointDetector::JointData> JointDetector::detectJointsInSinglePart(const TopoDS_Shape& shape) {
    std::vector<JointData> joints;
    int analyzedCount = 0;

    // 为单实体检测配置CornerJointDetector
    // 单实体需要过滤掉边界边，只保留内部边作为焊缝
    CornerJointDetector::Config cornerConfig;
    cornerConfig.minBoundaryDistance = 50.0;  // 只有距离边界>50mm的边才被认为是内部焊缝
    cornerConfig.isMultiSolid = false;  // 单实体模式：OO状态不被认为是凹边
    cornerConfig.enableDebugOutput = true;
    cornerDetector->setConfig(cornerConfig);

    // 获取单个实体
    const TopoDS_Solid& solid = TopoDS::Solid(solids(1));

    // 遍历所有边
    for (int i = 1; i <= edges.Extent(); ++i) {
        const TopoDS_Edge& edge = TopoDS::Edge(edges(i));

        // 获取相邻面
        auto adjacentFaces = getAdjacentFaces(edge);
        if (adjacentFaces.size() != 2) {
            continue;
        }

        analyzedCount++;

        // 计算两面夹角
        double angle = calculateAngle(adjacentFaces[0], adjacentFaces[1]);

        // 识别接头类型 - 单实体使用自己的方法
        JointType type = UNKNOWN_JOINT;

        // 单实体的角接头检测 - 直接使用带solid参数的版本
        if (config.detectCornerJoint && cornerDetector->isCornerJoint(edge, adjacentFaces[0], adjacentFaces[1], solid)) {
            type = CORNER_JOINT;
        } else if (config.detectButtJoint && buttDetector->isButtJoint(edge, adjacentFaces[0], adjacentFaces[1])) {
            type = BUTT_JOINT;
        } else if (config.detectTJoint && tDetector->isTJoint(edge, adjacentFaces[0], adjacentFaces[1])) {
            type = T_JOINT;
        } else if (config.detectLapJoint && lapDetector->isLapJoint(edge, adjacentFaces[0], adjacentFaces[1])) {
            type = LAP_JOINT;
        } else if (config.detectEdgeJoint && edgeDetector->isEdgeJoint(edge, adjacentFaces[0], adjacentFaces[1])) {
            type = EDGE_JOINT;
        }

        // 如果找到了接头类型，添加到结果中
        if (type != UNKNOWN_JOINT) {
            JointData joint;
            joint.type = type;
            joint.edge = edge;
            joint.face1 = adjacentFaces[0];
            joint.face2 = adjacentFaces[1];
            joint.angle = angle;
            joint.confidence = 0.0;  // TODO: 计算置信�?
            joint.typeName = getJointTypeName(type);
            joint.solid1Id = 1;
            joint.solid2Id = 1;
            joint.solid1Name = "SinglePart";
            joint.solid2Name = "SinglePart";

            joints.push_back(joint);
            std::cout << "  Found: " << joint.typeName
                      << " at " << angle << " degrees" << std::endl;
        }
    }

    std::cout << "\nAnalysis summary:" << std::endl;
    std::cout << "  Total edges: " << edges.Extent() << std::endl;
    std::cout << "  Edges analyzed: " << analyzedCount << std::endl;
    std::cout << "  Joints detected: " << joints.size() << std::endl;

    return joints;
}

std::vector<JointDetector::JointData> JointDetector::detectJointsInMultiPart(const TopoDS_Shape& shape) {
    std::vector<JointData> joints;
    int analyzedCount = 0;

    // 为多实体检测配置CornerJointDetector
    CornerJointDetector::Config cornerConfig;
    cornerConfig.minBoundaryDistance = 0.0;  // 多实体不需要边界过滤
    cornerConfig.isMultiSolid = true;  // 多实体模式：OO状态可能是凹边
    cornerConfig.minEdgeLength = 30.0;  // 多实体模式下提高最小边长阈值，避免误判短边
    cornerConfig.enableDebugOutput = true;
    cornerDetector->setConfig(cornerConfig);

    std::cout << "\nAnalyzing joints in multi-solid assembly..." << std::endl;

    // 对于多实体，主要使用EdgeOnFace来检测接触
    // 因为不同实体的边通常不共享，而是通过边在面上的关系来判断接触
    std::cout << "\nStep 1: Using EdgeOnFace to detect inter-solid contacts..." << std::endl;


    EdgeOnFace::Detector edgeOnFaceDetector;
    EdgeOnFace::Config edgeOnFaceConfig;
    edgeOnFaceConfig.tolerance = 1e-4;
    edgeOnFaceConfig.samplePoints = 20;
    edgeOnFaceConfig.matchThreshold = 0.8;
    edgeOnFaceDetector.setConfig(edgeOnFaceConfig);

    auto edgeFaceRelations = edgeOnFaceDetector.analyze(shape);
    std::cout << "Found " << edgeFaceRelations.size() << " edge-on-face relations." << std::endl;

    // 用于跟踪已处理的边，避免重复
    std::set<int> processedEdges;

    // 处理所有边在面上的关系
    for (const auto& relation : edgeFaceRelations) {
        // 跳过已处理的边
        if (processedEdges.count(relation.edgeId) > 0) {
            std::cout << "Skip: Edge" << relation.edgeId << " already processed" << std::endl;
            continue;
        }
        // Debug: 输出关系信息
        std::cout << "Processing: Edge" << relation.edgeId
                  << " from " << relation.edgeSolidName
                  << " on Face" << relation.faceId
                  << " from " << relation.faceSolidName
                  << " (Match: " << relation.matchPercentage << "%)" << std::endl;

        // 获取边（来自实体A）
        const TopoDS_Edge& edge = TopoDS::Edge(edges(relation.edgeId));
        // 获取面（来自实体B）
        const TopoDS_Face& faceB = TopoDS::Face(faces(relation.faceId));

        // 获取边的两个相邻面（来自实体A）
        auto adjacentFaces = getAdjacentFaces(edge);
        if (adjacentFaces.size() != 2) {
            std::cout << "  Skip: Edge has " << adjacentFaces.size() << " adjacent faces (expected 2)" << std::endl;
            continue;
        }

        // 计算faceB的法向量
        gp_Vec normalB = calculateFaceNormal(faceB, edge);

        // 分别计算两个相邻面的法向量
        gp_Vec normal1 = calculateFaceNormal(adjacentFaces[0], edge);
        gp_Vec normal2 = calculateFaceNormal(adjacentFaces[1], edge);

        // 计算点积来判断关系
        double dot1 = normal1.Normalized().Dot(normalB.Normalized());
        double dot2 = normal2.Normalized().Dot(normalB.Normalized());

        std::cout << "  Adjacent faces analysis: dot1=" << dot1 << ", dot2=" << dot2 << std::endl;

        // 选择正确的面：
        // - 如果点积接近-1，说明法向相反，面对面贴合（隐藏面）
        // - 如果点积接近0，说明法向垂直，可能是角焊缝
        // - 如果点积接近1，说明法向相同，也不是焊缝

        TopoDS_Face selectedFaceA;
        bool isFace1Hidden = (dot1 < -0.7);  // 法向相反，是隐藏面
        bool isFace2Hidden = (dot2 < -0.7);  // 法向相反，是隐藏面

        // 选择非隐藏面
        if (isFace1Hidden && !isFace2Hidden) {
            selectedFaceA = adjacentFaces[1];  // 面1是隐藏面，选择面2
            std::cout << "  Selected Face2 (Face1 is hidden)" << std::endl;
        } else if (!isFace1Hidden && isFace2Hidden) {
            selectedFaceA = adjacentFaces[0];  // 面2是隐藏面，选择面1
            std::cout << "  Selected Face1 (Face2 is hidden)" << std::endl;
        } else if (!isFace1Hidden && !isFace2Hidden) {
            // 两个面都不是隐藏面，选择更垂直的那个
            selectedFaceA = (std::abs(dot1) < std::abs(dot2)) ? adjacentFaces[0] : adjacentFaces[1];
            std::cout << "  Selected " << ((std::abs(dot1) < std::abs(dot2)) ? "Face1" : "Face2") << " (both visible, chose more perpendicular)" << std::endl;
        } else {
            // 两个面都是隐藏面，跳过
            std::cout << "  Skip: Both faces are hidden" << std::endl;
            continue;
        }

        // 重要修正：对于多实体角接头检测
        // 边在面上只是确认了边在另一个实体上，但角接头应该检测边的真正相邻面！
        // 正确的做法：使用边的两个相邻面（都来自同一实体）进行角接头检测

        // 获取对应的实体
        TopoDS_Solid solidForDetection;
        if (relation.edgeSolidId > 0 && relation.edgeSolidId <= solids.Extent()) {
            solidForDetection = TopoDS::Solid(solids(relation.edgeSolidId));
        }

        // 计算边的两个相邻面之间的角度（这才是真正的角接头角度）
        double angle = calculateAngle(adjacentFaces[0], adjacentFaces[1]);

        // 只有当边的两个相邻面形成角接（75-105度）时，才可能是角接头
        if (angle < 75.0 || angle > 105.0) {
            std::cout << "  Skip: Adjacent faces angle " << angle << " is not in corner joint range [75-105]" << std::endl;
            continue;
        }

        // 识别接头类型 - 使用边的真正相邻面
        JointType type = UNKNOWN_JOINT;
        if (!solidForDetection.IsNull()) {
            // 传入边的两个真正相邻面，而不是混合不同实体的面
            type = identifyJointTypeWithSolid(edge, adjacentFaces[0], adjacentFaces[1], solidForDetection);
            std::cout << "  Joint type (with solid): " << getJointTypeName(type) << std::endl;
        } else {
            type = identifyJointType(edge, adjacentFaces[0], adjacentFaces[1]);
            std::cout << "  Joint type (without solid): " << getJointTypeName(type) << std::endl;
        }

        if (shouldAddJoint(type)) {
            JointData joint;
            joint.type = type;
            joint.edge = edge;
            joint.face1 = adjacentFaces[0];  // 边的第一个相邻面
            joint.face2 = adjacentFaces[1];  // 边的第二个相邻面
            joint.angle = angle;
            joint.confidence = relation.matchPercentage / 100.0;
            joint.typeName = getJointTypeName(type);
            joint.solid1Id = relation.edgeSolidId;
            joint.solid2Id = relation.faceSolidId;
            joint.solid1Name = relation.edgeSolidName;
            joint.solid2Name = relation.faceSolidName;

            joints.push_back(joint);
            std::cout << "  Found: " << joint.typeName
                      << " between " << joint.solid1Name << "-" << joint.solid2Name
                      << " at " << angle << " degrees"
                      << " (Face selection: dot1=" << dot1 << ", dot2=" << dot2
                      << ", selected=" << (selectedFaceA.IsSame(adjacentFaces[0]) ? "Face1" : "Face2") << ")" << std::endl;

            // 标记该边为已处理，避免重复检测
            processedEdges.insert(relation.edgeId);
            analyzedCount++;
        }
    }

    // 调试：输出所有未被处理的边-面关系
    std::cout << "\nUnprocessed edge-on-face relations:" << std::endl;
    for (const auto& relation : edgeFaceRelations) {
        if (processedEdges.count(relation.edgeId) == 0) {
            const TopoDS_Edge& edge = TopoDS::Edge(edges(relation.edgeId));
            BRepAdaptor_Curve curveAdaptor(edge);
            double edgeLength = GCPnts_AbscissaPoint::Length(curveAdaptor);

            std::cout << "  Unprocessed: Edge" << relation.edgeId
                      << " (L=" << edgeLength << "mm)"
                      << " from " << relation.edgeSolidName
                      << " on Face" << relation.faceId
                      << " from " << relation.faceSolidName << std::endl;
        }
    }

    std::cout << "\nAnalysis summary:" << std::endl;
    std::cout << "  Total solids: " << solids.Extent() << std::endl;
    std::cout << "  Edges analyzed: " << analyzedCount << std::endl;
    std::cout << "  Joints detected: " << joints.size() << std::endl;

    return joints;
}
void JointDetector::initialize(const TopoDS_Shape& shape) {
    // 清空旧数�?
    edges.Clear();
    faces.Clear();
    solids.Clear();
    edgeToFaces.Clear();
    faceToSolid.clear();
    edgeToSolid.clear();
    solidNames.clear();

    // 构建拓扑映射
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    TopExp::MapShapes(shape, TopAbs_FACE, faces);
    TopExp::MapShapes(shape, TopAbs_SOLID, solids);
    TopExp::MapShapesAndUniqueAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);

    // 构建归属映射
    for (int solidIdx = 1; solidIdx <= solids.Extent(); ++solidIdx) {
        TopoDS_Solid solid = TopoDS::Solid(solids(solidIdx));
        solidNames[solidIdx] = "Solid_" + std::to_string(solidIdx);

        // 记录面的归属
        TopExp_Explorer faceExp(solid, TopAbs_FACE);
        for (; faceExp.More(); faceExp.Next()) {
            const TopoDS_Face& face = TopoDS::Face(faceExp.Current());
            int faceIdx = faces.FindIndex(face);
            if (faceIdx > 0) {
                faceToSolid[faceIdx] = solidIdx;
            }
        }

        // 记录边的归属
        TopExp_Explorer edgeExp(solid, TopAbs_EDGE);
        for (; edgeExp.More(); edgeExp.Next()) {
            const TopoDS_Edge& edge = TopoDS::Edge(edgeExp.Current());
            int edgeIdx = edges.FindIndex(edge);
            if (edgeIdx > 0) {
                edgeToSolid[edgeIdx] = solidIdx;
            }
        }
    }

    std::cout << "\nTopology initialized:" << std::endl;
    std::cout << "  Solids: " << solids.Extent() << std::endl;
    std::cout << "  Faces: " << faces.Extent() << std::endl;
    std::cout << "  Edges: " << edges.Extent() << std::endl;
}

std::vector<TopoDS_Face> JointDetector::getAdjacentFaces(const TopoDS_Edge& edge) {
    std::vector<TopoDS_Face> adjacentFaces;
    int edgeIdx = edges.FindIndex(edge);

    if (edgeIdx > 0 && edgeIdx <= edgeToFaces.Extent()) {
        const TopTools_ListOfShape& faceList = edgeToFaces(edgeIdx);
        for (TopTools_ListOfShape::Iterator it(faceList); it.More(); it.Next()) {
            adjacentFaces.push_back(TopoDS::Face(it.Value()));
        }
    }
    return adjacentFaces;
}

double JointDetector::calculateAngle(const TopoDS_Face& face1, const TopoDS_Face& face2) {
    // TODO: 实现精确的角度计算
    // 暂时返回90度作为测试
    return 90.0;
}

bool JointDetector::shouldAddJoint(JointType type) {
    switch (type) {
        case BUTT_JOINT: return config.detectButtJoint;
        case CORNER_JOINT: return config.detectCornerJoint;
        case T_JOINT: return config.detectTJoint;
        case LAP_JOINT: return config.detectLapJoint;
        case EDGE_JOINT: return config.detectEdgeJoint;
        default: return false;
    }
}

gp_Vec JointDetector::calculateFaceNormal(const TopoDS_Face& face, const TopoDS_Edge& edge) {
    // 获取边的中点
    BRepAdaptor_Curve curveAdaptor(edge);
    double first = curveAdaptor.FirstParameter();
    double last = curveAdaptor.LastParameter();
    double mid = (first + last) / 2.0;
    gp_Pnt midPoint = curveAdaptor.Value(mid);

    // 获取面的表面
    TopLoc_Location loc;
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face, loc);

    // 将点投影到面上获取UV坐标
    GeomAPI_ProjectPointOnSurf projector(midPoint, surface);
    if (projector.NbPoints() > 0) {
        double u, v;
        projector.LowerDistanceParameters(u, v);

        // 计算法向量
        GeomLProp_SLProps props(surface, u, v, 1, 1e-6);
        if (props.IsNormalDefined()) {
            gp_Dir normal = props.Normal();

            // 考虑面的方向
            if (face.Orientation() == TopAbs_REVERSED) {
                normal.Reverse();
            }

            return gp_Vec(normal);
        }
    }

    // 如果失败，返回默认向量
    return gp_Vec(0, 0, 1);
}

JointDetector::JointType JointDetector::identifyJointType(const TopoDS_Edge& edge,
                                                          const TopoDS_Face& face1,
                                                          const TopoDS_Face& face2) {
    // 使用各个专门的检测器进行判断

    if (config.detectButtJoint && buttDetector->isButtJoint(edge, face1, face2)) {
        return BUTT_JOINT;
    }

    // 使用Corner Joint检测器 - 不带solid参数的基础版本
    // 注意：这个函数主要用于没有solid信息的情况
    // 如果有solid信息，应该使用identifyJointTypeWithSolid
    if (config.detectCornerJoint) {
        if (cornerDetector->isCornerJoint(edge, face1, face2)) {
            return CORNER_JOINT;
        }
    }

    if (config.detectTJoint && tDetector->isTJoint(edge, face1, face2)) {
        return T_JOINT;
    }

    if (config.detectLapJoint && lapDetector->isLapJoint(edge, face1, face2)) {
        return LAP_JOINT;
    }

    if (config.detectEdgeJoint && edgeDetector->isEdgeJoint(edge, face1, face2)) {
        return EDGE_JOINT;
    }

    return UNKNOWN_JOINT;
}

void JointDetector::displayResults(const std::vector<JointData>& joints) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "JOINT DETECTION RESULTS" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Total joints found: " << joints.size() << std::endl;

    if (joints.empty()) {
        std::cout << "No joints detected." << std::endl;
        return;
    }

    // 统计各类�?
    int buttCount = 0, cornerCount = 0, tCount = 0, lapCount = 0, edgeCount = 0;
    for (const auto& joint : joints) {
        switch (joint.type) {
            case BUTT_JOINT: buttCount++; break;
            case CORNER_JOINT: cornerCount++; break;
            case T_JOINT: tCount++; break;
            case LAP_JOINT: lapCount++; break;
            case EDGE_JOINT: edgeCount++; break;
        }
    }

    std::cout << "\nJoint Type Statistics:" << std::endl;
    if (buttCount > 0) std::cout << "  Butt Joint: " << buttCount << std::endl;
    if (cornerCount > 0) std::cout << "  Corner Joint (L): " << cornerCount << std::endl;
    if (tCount > 0) std::cout << "  T-Joint: " << tCount << std::endl;
    if (lapCount > 0) std::cout << "  Lap Joint: " << lapCount << std::endl;
    if (edgeCount > 0) std::cout << "  Edge Joint: " << edgeCount << std::endl;

    // 详细信息
    std::cout << "\nDetailed Joint Information:" << std::endl;
    for (size_t i = 0; i < joints.size(); ++i) {
        const auto& joint = joints[i];
        std::cout << "[" << (i + 1) << "] "
                  << joint.typeName
                  << " | " << joint.solid1Name << "-" << joint.solid2Name
                  << " | Angle: " << std::fixed << std::setprecision(1) << joint.angle << "°"
                  << " | Confidence: " << std::setprecision(2) << joint.confidence
                  << std::endl;
    }
    std::cout << "========================================" << std::endl;
}

JointDetector::JointType JointDetector::identifyJointTypeWithSolid(const TopoDS_Edge& edge,
                                                                   const TopoDS_Face& face1,
                                                                   const TopoDS_Face& face2,
                                                                   const TopoDS_Solid& solid) {
    // 使用各个专门的检测器进行判断

    if (config.detectButtJoint && buttDetector->isButtJoint(edge, face1, face2)) {
        return BUTT_JOINT;
    }

    // 使用改进的Corner Joint检测器，带solid参数
    if (config.detectCornerJoint) {
        double tempConfidence = 0.0;
        std::cout << "    Checking corner joint..." << std::endl;
        bool isCorner = cornerDetector->isCornerJointWithConfidence(edge, face1, face2, solid, tempConfidence);
        std::cout << "    Corner joint result: " << (isCorner ? "YES" : "NO")
                  << " (confidence: " << tempConfidence << ")" << std::endl;
        if (isCorner) {
            return CORNER_JOINT;
        }
    }

    if (config.detectTJoint && tDetector->isTJoint(edge, face1, face2)) {
        return T_JOINT;
    }

    if (config.detectLapJoint && lapDetector->isLapJoint(edge, face1, face2)) {
        return LAP_JOINT;
    }

    if (config.detectEdgeJoint && edgeDetector->isEdgeJoint(edge, face1, face2)) {
        return EDGE_JOINT;
    }

    return UNKNOWN_JOINT;
}

std::string JointDetector::getJointTypeName(JointType type) {
    switch (type) {
        case BUTT_JOINT: return "Butt Joint";
        case CORNER_JOINT: return "Corner Joint (L)";
        case T_JOINT: return "T-Joint";
        case LAP_JOINT: return "Lap Joint";
        case EDGE_JOINT: return "Edge Joint";
        default: return "Unknown";
    }
}
