#pragma once
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Vec.hxx>
#include <vector>

// T型接头检测器 (T-Joint)
// 标准特征：一个工件垂直连接到另一个工件的表面，形成T形内角
class TJointDetector {
public:
    struct TJoint {
        TopoDS_Edge edge;
        TopoDS_Face face1;
        TopoDS_Face face2;
        double angle;
        double confidence;
        bool isInternalAngle;  // 是否为内角
        double normalSumMagnitude;  // 法向量和的模长
        TopoDS_Face stemFace;  // T型的"杆"面
        TopoDS_Face baseFace;  // T型的"底"面
    };

    struct Config {
        double angleToleranceDeg = 15.0;     // 角度容差
        double normalSumThreshold = 0.8;     // 法向量和阈值（区分内外角）
        double minConfidence = 0.7;          // 最小置信度
        bool requirePlanarFaces = true;      // 是否要求平面
        bool requireStraightEdge = true;     // 是否要求直边
        bool identifyStemBase = true;        // 是否识别杆-底结构
    };

public:
    TJointDetector();
    ~TJointDetector();

    // 设置配置
    void setConfig(const Config& cfg) { config = cfg; }

    // 检测T型接头
    std::vector<TJoint> detect(const TopoDS_Edge& edge,
                               const TopoDS_Face& face1,
                               const TopoDS_Face& face2);

    // 判断是否为T型接头
    bool isTJoint(const TopoDS_Edge& edge,
                  const TopoDS_Face& face1,
                  const TopoDS_Face& face2);

private:
    Config config;

    // 核心检测方法
    double calculateDihedralAngle(const TopoDS_Face& face1, const TopoDS_Face& face2);
    gp_Vec getFaceNormal(const TopoDS_Face& face);
    bool isInternalAngle(const TopoDS_Face& face1, const TopoDS_Face& face2);
    double calculateConfidence(const TopoDS_Edge& edge, const TopoDS_Face& face1, const TopoDS_Face& face2, double angle);

    // T型特征分析
    bool identifyTConfiguration(const TopoDS_Edge& edge, const TopoDS_Face& face1, const TopoDS_Face& face2,
                               TopoDS_Face& stemFace, TopoDS_Face& baseFace);
    bool isPerpendicularConnection(const TopoDS_Face& face1, const TopoDS_Face& face2);

    // 辅助检查方法
    bool isPlanarFace(const TopoDS_Face& face);
    bool isStraightEdge(const TopoDS_Edge& edge);
    bool isTJointConfiguration(const TopoDS_Edge& edge, const TopoDS_Face& face1, const TopoDS_Face& face2);
};