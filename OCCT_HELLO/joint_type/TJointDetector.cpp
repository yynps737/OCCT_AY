#include "TJointDetector.h"
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Plane.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GeomAbs_CurveType.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <cmath>

TJointDetector::TJointDetector() {
    // 默认配置适用于标准T型接头检测
    config.angleToleranceDeg = 15.0;
    config.normalSumThreshold = 0.8;  // T型内角阈值更低
    config.minConfidence = 0.7;
    config.requirePlanarFaces = true;
    config.requireStraightEdge = true;
    config.identifyStemBase = true;
}

TJointDetector::~TJointDetector() {}

std::vector<TJointDetector::TJoint> TJointDetector::detect(
    const TopoDS_Edge& edge,
    const TopoDS_Face& face1,
    const TopoDS_Face& face2) {

    std::vector<TJoint> results;

    if (isTJoint(edge, face1, face2)) {
        TJoint joint;
        joint.edge = edge;
        joint.face1 = face1;
        joint.face2 = face2;
        joint.angle = calculateDihedralAngle(face1, face2);
        joint.isInternalAngle = isInternalAngle(face1, face2);

        // 计算法向量和的模长
        gp_Vec n1 = getFaceNormal(face1);
        gp_Vec n2 = getFaceNormal(face2);
        gp_Vec normalSum = n1.Normalized() + n2.Normalized();
        joint.normalSumMagnitude = normalSum.Magnitude();

        // 识别T型的杆-底结构
        if (config.identifyStemBase) {
            identifyTConfiguration(edge, face1, face2, joint.stemFace, joint.baseFace);
        }

        joint.confidence = calculateConfidence(edge, face1, face2, joint.angle);

        results.push_back(joint);
    }

    return results;
}

bool TJointDetector::isTJoint(const TopoDS_Edge& edge,
                              const TopoDS_Face& face1,
                              const TopoDS_Face& face2) {

    // 1. 基础几何检查
    if (config.requirePlanarFaces) {
        if (!isPlanarFace(face1) || !isPlanarFace(face2)) {
            return false;
        }
    }

    if (config.requireStraightEdge) {
        if (!isStraightEdge(edge)) {
            return false;
        }
    }

    // 2. 角度检查 - 必须接近90度
    double angle = calculateDihedralAngle(face1, face2);
    if (std::abs(angle - 90.0) > config.angleToleranceDeg) {
        return false;
    }

    // 3. 内角检查 - T型接头的关键特征（与角接相反）
    if (!isInternalAngle(face1, face2)) {
        return false;
    }

    // 4. T型配置检查
    if (!isTJointConfiguration(edge, face1, face2)) {
        return false;
    }

    // 5. 垂直连接检查
    if (!isPerpendicularConnection(face1, face2)) {
        return false;
    }

    // 6. 置信度检查
    double confidence = calculateConfidence(edge, face1, face2, angle);
    if (confidence < config.minConfidence) {
        return false;
    }

    return true;
}

double TJointDetector::calculateDihedralAngle(const TopoDS_Face& face1, const TopoDS_Face& face2) {
    gp_Vec normal1 = getFaceNormal(face1);
    gp_Vec normal2 = getFaceNormal(face2);

    // 计算法向量夹角
    double dotProduct = normal1.Dot(normal2);

    // 防止数值误差
    if (dotProduct > 1.0) dotProduct = 1.0;
    if (dotProduct < -1.0) dotProduct = -1.0;

    double angleRad = std::acos(std::abs(dotProduct));
    double angleDeg = angleRad * 180.0 / M_PI;

    return angleDeg;
}

gp_Vec TJointDetector::getFaceNormal(const TopoDS_Face& face) {
    // 获取面的几何表面
    TopLoc_Location loc;
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face, loc);

    if (surface.IsNull()) {
        return gp_Vec(0, 0, 1); // 默认向上
    }

    // 获取面的UV边界
    Standard_Real umin, umax, vmin, vmax;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);

    // 在面的中心计算法向量
    Standard_Real umid = (umin + umax) / 2.0;
    Standard_Real vmid = (vmin + vmax) / 2.0;

    GeomLProp_SLProps props(surface, umid, vmid, 1, 1e-6);

    if (!props.IsNormalDefined()) {
        return gp_Vec(0, 0, 1); // 默认向上
    }

    gp_Dir normal = props.Normal();

    // 考虑面的方向
    if (face.Orientation() == TopAbs_REVERSED) {
        normal.Reverse();
    }

    return gp_Vec(normal);
}

bool TJointDetector::isInternalAngle(const TopoDS_Face& face1, const TopoDS_Face& face2) {
    // 计算归一化法向量和
    gp_Vec n1 = getFaceNormal(face1);
    gp_Vec n2 = getFaceNormal(face2);

    gp_Vec normalSum = n1.Normalized() + n2.Normalized();
    double magnitude = normalSum.Magnitude();

    // 内角：法向量和的模长接近0（法向量几乎相反）
    // 外角：法向量和的模长接近√2（法向量几乎同向）
    return magnitude < config.normalSumThreshold;
}

double TJointDetector::calculateConfidence(const TopoDS_Edge& edge,
                                           const TopoDS_Face& face1,
                                           const TopoDS_Face& face2,
                                           double angle) {
    double confidence = 1.0;

    // 1. 角度偏差的置信度惩罚
    double angleDeviation = std::abs(angle - 90.0);
    double angleScore = 1.0 - (angleDeviation / config.angleToleranceDeg);
    confidence *= std::max(0.0, angleScore);

    // 2. 法向量和模长的置信度（T型应该接近0）
    gp_Vec n1 = getFaceNormal(face1);
    gp_Vec n2 = getFaceNormal(face2);
    gp_Vec normalSum = n1.Normalized() + n2.Normalized();
    double magnitude = normalSum.Magnitude();

    // 理想的内角magnitude应该接近0
    double magnitudeScore = 1.0 - magnitude; // 模长越小越好
    confidence *= std::max(0.0, magnitudeScore);

    // 3. 几何形状的置信度
    double geometryScore = 1.0;

    if (config.requirePlanarFaces) {
        if (!isPlanarFace(face1)) geometryScore *= 0.7;
        if (!isPlanarFace(face2)) geometryScore *= 0.7;
    }

    if (config.requireStraightEdge) {
        if (!isStraightEdge(edge)) geometryScore *= 0.8;
    }

    confidence *= geometryScore;

    // 4. T型配置的置信度加成
    if (isPerpendicularConnection(face1, face2)) {
        confidence *= 1.1; // 10%的置信度提升
    }

    return std::max(0.0, std::min(1.0, confidence));
}

bool TJointDetector::identifyTConfiguration(const TopoDS_Edge& edge,
                                            const TopoDS_Face& face1,
                                            const TopoDS_Face& face2,
                                            TopoDS_Face& stemFace,
                                            TopoDS_Face& baseFace) {
    // T型配置识别：
    // - 杆面（stem）：通常是较小的面，代表"插入"的工件
    // - 底面（base）：通常是较大的面，代表"被插入"的工件

    // 暂时简单处理，可以根据面的大小或其他特征进一步优化
    gp_Vec n1 = getFaceNormal(face1);
    gp_Vec n2 = getFaceNormal(face2);

    // 通过法向量的方向性来判断哪个是"插入"的面
    // 这里使用简化的启发式方法
    if (n1.Z() > n2.Z()) {
        stemFace = face1;
        baseFace = face2;
    } else {
        stemFace = face2;
        baseFace = face1;
    }

    return true;
}

bool TJointDetector::isPerpendicularConnection(const TopoDS_Face& face1, const TopoDS_Face& face2) {
    gp_Vec n1 = getFaceNormal(face1);
    gp_Vec n2 = getFaceNormal(face2);

    // 垂直连接：两个法向量应该近似垂直
    double dotProduct = std::abs(n1.Normalized().Dot(n2.Normalized()));

    // 垂直时点积接近0，允许一定容差
    return dotProduct < 0.2; // 约等于cos(78.5°)
}

bool TJointDetector::isPlanarFace(const TopoDS_Face& face) {
    BRepAdaptor_Surface surfaceAdaptor(face);
    return surfaceAdaptor.GetType() == GeomAbs_Plane;
}

bool TJointDetector::isStraightEdge(const TopoDS_Edge& edge) {
    BRepAdaptor_Curve curveAdaptor(edge);
    return curveAdaptor.GetType() == GeomAbs_Line;
}

bool TJointDetector::isTJointConfiguration(const TopoDS_Edge& edge,
                                           const TopoDS_Face& face1,
                                           const TopoDS_Face& face2) {
    // T型配置的特征：
    // 1. 边在两个面的交界处
    // 2. 两个面形成内角
    // 3. 一个工件"撞击"另一个工件

    // 基本检查：确保这是一个有效的边-面配置
    if (BRep_Tool::Degenerated(edge)) {
        return false; // 退化边不是有效的T型接头
    }

    // 检查面的几何关系
    gp_Vec n1 = getFaceNormal(face1);
    gp_Vec n2 = getFaceNormal(face2);

    // 法向量不应该平行（这种情况下不是T型接头）
    double dotProduct = std::abs(n1.Normalized().Dot(n2.Normalized()));
    if (dotProduct > 0.95) { // 几乎平行
        return false;
    }

    // 法向量应该接近垂直（T型的特征）
    if (dotProduct > 0.2) { // 不够垂直
        return false;
    }

    return true;
}