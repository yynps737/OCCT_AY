#define _SILENCE_CXX17_ITERATOR_BASE_CLASS_DEPRECATION_WARNING

#include "CornerJointDetector.h"
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomLProp_SLProps.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <iostream>
#include <iomanip>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

CornerJointDetector::CornerJointDetector() {
    // 使用默认配置
}

CornerJointDetector::~CornerJointDetector() {
}

// ==================== 主检测接口 ====================

bool CornerJointDetector::isCornerJoint(const TopoDS_Edge& edge,
                                        const TopoDS_Face& face1,
                                        const TopoDS_Face& face2) {
    // 简化版本，缺少solid无法进行完整的凹凸性测试
    // 只能基于二面角和平面性判断
    double angle = calculateDihedralAngle(face1, face2, edge);
    double length = getEdgeLength(edge);
    bool planar = areBothFacesPlanar(face1, face2);

    return (angle >= m_config.minDihedralAngle &&
            angle <= m_config.maxDihedralAngle &&
            length > m_config.minEdgeLength &&
            planar);
}

bool CornerJointDetector::isCornerJoint(const TopoDS_Edge& edge,
                                        const TopoDS_Face& face1,
                                        const TopoDS_Face& face2,
                                        const TopoDS_Solid& solid) {
    WeldFeatures features = analyzeEdge(edge, face1, face2, solid);

    if (m_config.enableDebugOutput) {
        printFeatures(features);
    }

    return features.isCornerWeld;
}

bool CornerJointDetector::isCornerJointWithConfidence(const TopoDS_Edge& edge,
                                                      const TopoDS_Face& face1,
                                                      const TopoDS_Face& face2,
                                                      const TopoDS_Solid& solid,
                                                      double& confidence) {
    WeldFeatures features = analyzeEdge(edge, face1, face2, solid);

    if (m_config.enableDebugOutput) {
        printFeatures(features);
    }

    confidence = features.confidence;
    return features.isCornerWeld;
}

// ==================== 核心分析函数 ====================

CornerJointDetector::WeldFeatures CornerJointDetector::analyzeEdge(
    const TopoDS_Edge& edge,
    const TopoDS_Face& face1,
    const TopoDS_Face& face2,
    const TopoDS_Solid& solid) {

    WeldFeatures features;

    // 1. 基础信息
    features.edgeLength = getEdgeLength(edge);
    getEdgeMidpoint(edge, features.edgeMidpoint);

    // 2. 计算法向量
    calculateFaceNormal(face1, edge, features.normal1);
    calculateFaceNormal(face2, edge, features.normal2);

    // 3. 计算二面角和点积
    features.dihedralAngle = calculateDihedralAngle(face1, face2, edge);
    features.normalDotProduct = features.normal1[0] * features.normal2[0] +
                                features.normal1[1] * features.normal2[1] +
                                features.normal1[2] * features.normal2[2];

    // 4. 凹凸性测试（核心算法）
    features.isConcave = testConcavity(edge, face1, face2, solid, features);

    // 5. 边界距离计算
    features.distToBoundary = calculateBoundaryDistance(edge, solid);

    // 6. 平面性检查
    features.bothFacesPlanar = areBothFacesPlanar(face1, face2);

    // 7. 综合判定
    bool angleOK = (features.dihedralAngle >= m_config.minDihedralAngle &&
                    features.dihedralAngle <= m_config.maxDihedralAngle);
    bool lengthOK = (features.edgeLength > m_config.minEdgeLength);
    bool boundaryOK = (features.distToBoundary > m_config.minBoundaryDistance);

    features.isCornerWeld = features.isConcave && angleOK && lengthOK && boundaryOK && features.bothFacesPlanar;

    // 8. 计算置信度
    features.confidence = 0.0;
    if (features.isCornerWeld) {
        // 基于各项指标计算置信度
        double angleScore = 1.0 - std::abs(features.dihedralAngle - 90.0) / 15.0;
        double lengthScore = std::min(1.0, features.edgeLength / 100.0);
        double boundaryScore = std::min(1.0, features.distToBoundary / 50.0);
        features.confidence = (angleScore + lengthScore + boundaryScore) / 3.0;
    }

    // 设置ID（需要外部赋值）
    features.edgeId = -1;
    features.face1Id = -1;
    features.face2Id = -1;

    return features;
}

// ==================== 核心算法：凹凸性测试 ====================

bool CornerJointDetector::testConcavity(const TopoDS_Edge& edge,
                                        const TopoDS_Face& face1,
                                        const TopoDS_Face& face2,
                                        const TopoDS_Solid& solid,
                                        WeldFeatures& features) {
    // 角平分线分类法

    // 1. 计算角平分线方向
    gp_Vec n1(features.normal1[0], features.normal1[1], features.normal1[2]);
    gp_Vec n2(features.normal2[0], features.normal2[1], features.normal2[2]);

    // 归一化
    if (n1.Magnitude() > 0) n1.Normalize();
    if (n2.Magnitude() > 0) n2.Normalize();

    // 角平分线 = 两个单位法向量的和
    gp_Vec bisector = n1 + n2;
    if (bisector.Magnitude() > m_config.tolerance) {
        bisector.Normalize();
    } else {
        // 法向量近似反向（180度），尝试使用边的切向量
        BRepAdaptor_Curve curveAdaptor(edge);
        double u = (curveAdaptor.FirstParameter() + curveAdaptor.LastParameter()) / 2.0;
        gp_Vec tangent;
        gp_Pnt dummy;
        curveAdaptor.D1(u, dummy, tangent);

        // 使用边的切向量与法向量的叉积作为测试方向
        bisector = n1.Crossed(tangent);
        if (bisector.Magnitude() > m_config.tolerance) {
            bisector.Normalize();
        } else {
            // 仍然无效，可能是退化情况
            return false;
        }
    }

    features.bisectorDir[0] = bisector.X();
    features.bisectorDir[1] = bisector.Y();
    features.bisectorDir[2] = bisector.Z();

    // 2. 创建测试点
    gp_Pnt midPoint(features.edgeMidpoint[0],
                    features.edgeMidpoint[1],
                    features.edgeMidpoint[2]);

    gp_Pnt testPointOut = midPoint.Translated(bisector * m_config.testPointOffset);
    gp_Pnt testPointIn = midPoint.Translated(-bisector * m_config.testPointOffset);

    // 3. 使用SolidClassifier判断点的位置
    BRepClass3d_SolidClassifier classifierOut(solid, testPointOut, m_config.tolerance);
    BRepClass3d_SolidClassifier classifierIn(solid, testPointIn, m_config.tolerance);

    // 4. 转换状态
    auto toPointState = [](TopAbs_State state) -> WeldFeatures::PointState {
        switch(state) {
            case TopAbs_OUT: return WeldFeatures::OUT;
            case TopAbs_IN:  return WeldFeatures::IN;
            default:         return WeldFeatures::ON;
        }
    };

    features.testPointOutState = toPointState(classifierOut.State());
    features.testPointInState = toPointState(classifierIn.State());

    // 5. 判定凹凸性
    // 凹边：角平分线正方向指向外部，负方向指向内部
    // 或者：两个点都在内部（内腔角）
    bool isConcave = (features.testPointOutState == WeldFeatures::OUT &&
                     features.testPointInState == WeldFeatures::IN) ||
                     (features.testPointOutState == WeldFeatures::IN &&
                     features.testPointInState == WeldFeatures::IN);  // II状态也视为凹边

    return isConcave;
}

// ==================== 辅助计算函数 ====================

double CornerJointDetector::calculateDihedralAngle(const TopoDS_Face& face1,
                                                   const TopoDS_Face& face2,
                                                   const TopoDS_Edge& edge) {
    double normal1[3], normal2[3];
    calculateFaceNormal(face1, edge, normal1);
    calculateFaceNormal(face2, edge, normal2);

    // 计算点积
    double dot = normal1[0] * normal2[0] +
                 normal1[1] * normal2[1] +
                 normal1[2] * normal2[2];

    // 限制在[-1, 1]范围内
    if (dot > 1.0) dot = 1.0;
    if (dot < -1.0) dot = -1.0;

    // 二面角 = arccos(|dot|)
    double angleRad = std::acos(std::abs(dot));
    double angleDeg = angleRad * 180.0 / M_PI;

    return angleDeg;
}

void CornerJointDetector::calculateFaceNormal(const TopoDS_Face& face,
                                              const TopoDS_Edge& edge,
                                              double normal[3]) {
    // 获取边的中点参数
    BRepAdaptor_Curve curveAdaptor(edge);
    double u = (curveAdaptor.FirstParameter() + curveAdaptor.LastParameter()) / 2.0;
    gp_Pnt midPnt = curveAdaptor.Value(u);

    // 获取面的参数
    BRepAdaptor_Surface surface(face);

    // 简单方法：使用面的中心点计算法向量
    double uMin = surface.FirstUParameter();
    double uMax = surface.LastUParameter();
    double vMin = surface.FirstVParameter();
    double vMax = surface.LastVParameter();

    double uMid = (uMin + uMax) / 2.0;
    double vMid = (vMin + vMax) / 2.0;

    // 计算法向量
    GeomLProp_SLProps props(surface.Surface().Surface(), uMid, vMid, 1, m_config.tolerance);

    if (props.IsNormalDefined()) {
        gp_Dir norm = props.Normal();

        // 考虑面的朝向
        if (face.Orientation() == TopAbs_REVERSED) {
            norm.Reverse();
        }

        normal[0] = norm.X();
        normal[1] = norm.Y();
        normal[2] = norm.Z();
    } else {
        normal[0] = normal[1] = normal[2] = 0.0;
    }
}

double CornerJointDetector::calculateBoundaryDistance(const TopoDS_Edge& edge,
                                                      const TopoDS_Solid& solid) {
    // 改进的边界检测：检查边是否在外壳（Shell）上

    // 方法1：检查边的面数
    // 如果一条边只有一个相邻面，它一定在边界上
    TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;
    TopExp::MapShapesAndAncestors(solid, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);

    int edgeIndex = edgeToFaces.FindIndex(edge);
    if (edgeIndex > 0) {
        const TopTools_ListOfShape& faceList = edgeToFaces(edgeIndex);
        if (faceList.Extent() == 1) {
            // 只有一个面 = 边界边
            return 0.0;
        }
    }

    // 方法2：使用边界盒距离作为辅助判断
    // 但提高阈值，避免误判内部特征
    double midpoint[3];
    getEdgeMidpoint(edge, midpoint);
    gp_Pnt midPnt(midpoint[0], midpoint[1], midpoint[2]);

    Bnd_Box bbox;
    BRepBndLib::Add(solid, bbox);
    double xMin, yMin, zMin, xMax, yMax, zMax;
    bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    double dx = std::min(std::abs(midPnt.X() - xMin), std::abs(midPnt.X() - xMax));
    double dy = std::min(std::abs(midPnt.Y() - yMin), std::abs(midPnt.Y() - yMax));
    double dz = std::min(std::abs(midPnt.Z() - zMin), std::abs(midPnt.Z() - zMax));

    double bboxDist = std::min({dx, dy, dz});

    // 如果离边界盒很近（<1mm），则认为是边界
    if (bboxDist < 1.0) {
        return bboxDist;
    }

    // 否则返回一个较大的值，表示是内部边
    return 100.0;  // 内部边返回大值
}

bool CornerJointDetector::areBothFacesPlanar(const TopoDS_Face& face1,
                                             const TopoDS_Face& face2) {
    BRepAdaptor_Surface surf1(face1);
    BRepAdaptor_Surface surf2(face2);

    return (surf1.GetType() == GeomAbs_Plane &&
            surf2.GetType() == GeomAbs_Plane);
}

double CornerJointDetector::getEdgeLength(const TopoDS_Edge& edge) {
    BRepAdaptor_Curve curveAdaptor(edge);
    return GCPnts_AbscissaPoint::Length(curveAdaptor);
}

void CornerJointDetector::getEdgeMidpoint(const TopoDS_Edge& edge, double point[3]) {
    BRepAdaptor_Curve curveAdaptor(edge);
    double u = (curveAdaptor.FirstParameter() + curveAdaptor.LastParameter()) / 2.0;
    gp_Pnt pnt = curveAdaptor.Value(u);
    point[0] = pnt.X();
    point[1] = pnt.Y();
    point[2] = pnt.Z();
}

// ==================== 批量分析 ====================

std::vector<CornerJointDetector::WeldFeatures> CornerJointDetector::analyzeAllEdges(const TopoDS_Solid& solid) {
    std::vector<WeldFeatures> results;

    // 构建拓扑映射
    TopTools_IndexedMapOfShape edges;
    TopTools_IndexedMapOfShape faces;
    TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;

    TopExp::MapShapes(solid, TopAbs_EDGE, edges);
    TopExp::MapShapes(solid, TopAbs_FACE, faces);
    TopExp::MapShapesAndAncestors(solid, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);

    // 遍历所有边
    for (int i = 1; i <= edges.Extent(); ++i) {
        const TopoDS_Edge& edge = TopoDS::Edge(edges(i));

        // 获取相邻的两个面
        const TopTools_ListOfShape& faceList = edgeToFaces.FindFromIndex(i);
        if (faceList.Extent() == 2) {
            auto it = faceList.cbegin();
            const TopoDS_Face& face1 = TopoDS::Face(*it);
            ++it;
            const TopoDS_Face& face2 = TopoDS::Face(*it);

            // 分析边
            WeldFeatures features = analyzeEdge(edge, face1, face2, solid);

            // 设置ID
            features.edgeId = i;
            features.face1Id = faces.FindIndex(face1);
            features.face2Id = faces.FindIndex(face2);

            results.push_back(features);
        }
    }

    return results;
}

// ==================== 调试输出 ====================

void CornerJointDetector::printFeatures(const WeldFeatures& features) const {
    std::cout << "E" << std::setw(3) << features.edgeId
              << "|F" << std::setw(2) << features.face1Id
              << "-F" << std::setw(2) << features.face2Id
              << "|L=" << std::setw(6) << std::fixed << std::setprecision(1) << features.edgeLength
              << "|A=" << std::setw(3) << (int)features.dihedralAngle << "deg"
              << "|D=" << std::setw(5) << std::setprecision(1) << features.distToBoundary
              << "|";

    // State characters
    char outChar = (features.testPointOutState == WeldFeatures::OUT) ? 'O' :
                   (features.testPointOutState == WeldFeatures::IN) ? 'I' : 'N';
    char inChar = (features.testPointInState == WeldFeatures::OUT) ? 'O' :
                  (features.testPointInState == WeldFeatures::IN) ? 'I' : 'N';

    std::cout << outChar << inChar;

    // Result
    if (features.isCornerWeld) {
        std::cout << "|[WELD]|Conf=" << std::setprecision(2) << features.confidence;
    } else {
        std::cout << "|[SKIP]|Reason:";
        // Output exclusion reason
        if (!features.isConcave) std::cout << "NotConcave";
        else if (features.dihedralAngle < m_config.minDihedralAngle) std::cout << "AngleSmall";
        else if (features.dihedralAngle > m_config.maxDihedralAngle) std::cout << "AngleLarge";
        else if (features.edgeLength <= m_config.minEdgeLength) std::cout << "TooShort";
        else if (features.distToBoundary <= m_config.minBoundaryDistance) std::cout << "NearBoundary";
        else if (!features.bothFacesPlanar) std::cout << "NonPlanar";
    }

    std::cout << std::endl;
}