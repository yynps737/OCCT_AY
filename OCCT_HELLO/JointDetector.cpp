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
    // 初始化拓扑数据
    initialize(shape);

    std::cout << "\nDetecting welding joints based on standard types..." << std::endl;
    std::cout << "Configuration:" << std::endl;
    std::cout << "  Butt Joint: " << (config.detectButtJoint ? "ON" : "OFF") << std::endl;
    std::cout << "  Corner Joint (L): " << (config.detectCornerJoint ? "ON" : "OFF") << std::endl;
    std::cout << "  T-Joint: " << (config.detectTJoint ? "ON" : "OFF") << std::endl;
    std::cout << "  Lap Joint: " << (config.detectLapJoint ? "ON" : "OFF") << std::endl;
    std::cout << "  Edge Joint: " << (config.detectEdgeJoint ? "ON" : "OFF") << std::endl;

    // 根据实体数量选择检测模式
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

        // 识别接头类型
        JointType type = identifyJointType(edge, adjacentFaces[0], adjacentFaces[1]);

        // 根据配置决定是否添加
        bool shouldAdd = false;
        switch (type) {
            case BUTT_JOINT:
                shouldAdd = config.detectButtJoint;
                break;
            case CORNER_JOINT:
                shouldAdd = config.detectCornerJoint;
                break;
            case T_JOINT:
                shouldAdd = config.detectTJoint;
                break;
            case LAP_JOINT:
                shouldAdd = config.detectLapJoint;
                break;
            case EDGE_JOINT:
                shouldAdd = config.detectEdgeJoint;
                break;
            default:
                shouldAdd = false;
        }

        if (shouldAdd && type != UNKNOWN_JOINT) {
            JointData joint;
            joint.type = type;
            joint.edge = edge;
            joint.face1 = adjacentFaces[0];
            joint.face2 = adjacentFaces[1];
            joint.angle = angle;
            joint.confidence = 0.0;  // TODO: 计算置信度
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

    // 使用EdgeOnFace模块检测实体间接触
    std::cout << "\nUsing EdgeOnFace module to detect inter-solid contacts..." << std::endl;

    EdgeOnFace::Detector edgeOnFaceDetector;
    EdgeOnFace::Config edgeOnFaceConfig;
    edgeOnFaceConfig.tolerance = 1e-4;
    edgeOnFaceConfig.samplePoints = 20;
    edgeOnFaceConfig.matchThreshold = 0.8;
    edgeOnFaceDetector.setConfig(edgeOnFaceConfig);

    // 获取边在面上的关系
    std::vector<EdgeOnFace::EdgeFaceRelation> edgeFaceRelations = edgeOnFaceDetector.analyze(shape);

    if (edgeFaceRelations.empty()) {
        std::cout << "No inter-solid contacts detected." << std::endl;
        return joints;
    }

    std::cout << "Found " << edgeFaceRelations.size() << " edge-face contacts." << std::endl;
    std::cout << "Converting contacts to joint candidates..." << std::endl;

    // 将边-面关系转换为接头检测
    for (const auto& relation : edgeFaceRelations) {
        // 获取边和相邻面
        const TopoDS_Edge& edge = TopoDS::Edge(edges(relation.edgeId));

        // 获取边的相邻面（用于计算角度）
        std::vector<TopoDS_Face> adjacentFaces = getAdjacentFaces(edge);
        if (adjacentFaces.size() < 2) {
            continue;
        }

        // 计算两面夹角
        double angle = calculateAngle(adjacentFaces[0], adjacentFaces[1]);

        // 识别接头类型
        JointType type = identifyJointType(edge, adjacentFaces[0], adjacentFaces[1]);

        // 根据配置决定是否添加
        bool shouldAdd = false;
        switch (type) {
            case BUTT_JOINT:
                shouldAdd = config.detectButtJoint;
                break;
            case CORNER_JOINT:
                shouldAdd = config.detectCornerJoint;
                break;
            case T_JOINT:
                shouldAdd = config.detectTJoint;
                break;
            case LAP_JOINT:
                shouldAdd = config.detectLapJoint;
                break;
            case EDGE_JOINT:
                shouldAdd = config.detectEdgeJoint;
                break;
            default:
                shouldAdd = false;
        }

        if (shouldAdd && type != UNKNOWN_JOINT) {
            JointData joint;
            joint.type = type;
            joint.edge = edge;
            joint.face1 = adjacentFaces[0];
            joint.face2 = adjacentFaces[1];
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
                      << " at " << angle << " degrees" << std::endl;
        }
    }

    std::cout << "\nAnalysis summary:" << std::endl;
    std::cout << "  Edge-face contacts: " << edgeFaceRelations.size() << std::endl;
    std::cout << "  Joints detected: " << joints.size() << std::endl;

    return joints;
}

void JointDetector::initialize(const TopoDS_Shape& shape) {
    // 清空旧数据
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
    // TODO: 实现角度计算
    // 暂时返回90度作为测试
    return 90.0;
}

JointDetector::JointType JointDetector::identifyJointType(const TopoDS_Edge& edge,
                                                          const TopoDS_Face& face1,
                                                          const TopoDS_Face& face2) {
    // 使用各个专门的检测器进行判断

    if (config.detectButtJoint && buttDetector->isButtJoint(edge, face1, face2)) {
        return BUTT_JOINT;
    }

    if (config.detectCornerJoint && cornerDetector->isCornerJoint(edge, face1, face2)) {
        return CORNER_JOINT;
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

    // 统计各类型
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