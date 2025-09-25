#pragma once
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Solid.hxx>
#include <vector>

// 角焊缝检测器 - 基于二面角凹边检测法（最顶级方案）
// 理论基础：ISO 9692标准，角焊缝定义为两个近似垂直平面形成的凹形内角
// 核心算法：角平分线分类法 + 多重几何过滤
class CornerJointDetector {
public:
    // 角焊缝几何特征数据
    struct WeldFeatures {
        // 基础信息
        int edgeId;
        int face1Id, face2Id;
        double edgeLength;

        // 核心几何特征
        double dihedralAngle;      // 二面角（度）
        double normalDotProduct;   // 法向量点积
        bool isConcave;           // 是否为凹边

        // 过滤特征
        double distToBoundary;    // 到边界的距离
        bool bothFacesPlanar;     // 两面都是平面

        // 判定结果
        bool isCornerWeld;        // 是否为角焊缝
        double confidence;        // 置信度 (0-1)

        // 调试信息
        double edgeMidpoint[3];   // 边中点
        double normal1[3];        // 面1法向量
        double normal2[3];        // 面2法向量
        double bisectorDir[3];    // 角平分线方向

        // 测试点状态
        enum PointState { IN, OUT, ON };
        PointState testPointOutState;  // 外测试点状态
        PointState testPointInState;   // 内测试点状态
    };

public:
    CornerJointDetector();
    ~CornerJointDetector();

    // 主检测接口
    bool isCornerJoint(const TopoDS_Edge& edge,
                      const TopoDS_Face& face1,
                      const TopoDS_Face& face2);

    bool isCornerJoint(const TopoDS_Edge& edge,
                      const TopoDS_Face& face1,
                      const TopoDS_Face& face2,
                      const TopoDS_Solid& solid);

    // Extended version that returns confidence
    bool isCornerJointWithConfidence(const TopoDS_Edge& edge,
                                    const TopoDS_Face& face1,
                                    const TopoDS_Face& face2,
                                    const TopoDS_Solid& solid,
                                    double& confidence);

    // 批量分析接口（用于调试）
    std::vector<WeldFeatures> analyzeAllEdges(const TopoDS_Solid& solid);

    // 单边分析（获取详细特征）
    WeldFeatures analyzeEdge(const TopoDS_Edge& edge,
                            const TopoDS_Face& face1,
                            const TopoDS_Face& face2,
                            const TopoDS_Solid& solid);

    // 配置参数
    struct Config {
        // 二面角范围（度）
        double minDihedralAngle = 75.0;
        double maxDihedralAngle = 105.0;

        // 边长过滤（mm）
        double minEdgeLength = 50.0;  // 提高到50mm，排除小连接特征

        // 边界距离过滤（mm）
        double minBoundaryDistance = 5.0;

        // 测试点偏移距离（mm）
        double testPointOffset = 1.0;

        // 数值容差
        double tolerance = 1e-6;

        // 是否输出调试信息
        bool enableDebugOutput = true;
    };

    void setConfig(const Config& config) { m_config = config; }
    const Config& getConfig() const { return m_config; }

private:
    Config m_config;

    // 核心算法实现
    double calculateDihedralAngle(const TopoDS_Face& face1,
                                 const TopoDS_Face& face2,
                                 const TopoDS_Edge& edge);

    bool testConcavity(const TopoDS_Edge& edge,
                      const TopoDS_Face& face1,
                      const TopoDS_Face& face2,
                      const TopoDS_Solid& solid,
                      WeldFeatures& features);

    double calculateBoundaryDistance(const TopoDS_Edge& edge,
                                    const TopoDS_Solid& solid);

    bool areBothFacesPlanar(const TopoDS_Face& face1,
                           const TopoDS_Face& face2);

    // 辅助函数
    void calculateFaceNormal(const TopoDS_Face& face,
                            const TopoDS_Edge& edge,
                            double normal[3]);

    double getEdgeLength(const TopoDS_Edge& edge);

    void getEdgeMidpoint(const TopoDS_Edge& edge, double point[3]);

    // 调试输出
    void printFeatures(const WeldFeatures& features) const;
};