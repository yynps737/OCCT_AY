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
        bool isCornerWeld;        // Is corner weld
        double confidence;        // Confidence (0-1)

        // Debug info
        double edgeMidpoint[3];   // Edge midpoint
        double edgeStart[3];      // Edge start point
        double edgeEnd[3];        // Edge end point
        double normal1[3];        // Face 1 normal vector
        double normal2[3];        // Face 2 normal vector
        double bisectorDir[3];    // Angle bisector direction

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

    // Batch analysis interface (for debugging)
    std::vector<WeldFeatures> analyzeAllEdges(const TopoDS_Solid& solid);

    // Export detected welds to HTML visualization
    bool exportVisualizationHTML(const std::vector<WeldFeatures>& welds,
                                const std::string& outputPath);

    // Single edge analysis (get detailed features)
    WeldFeatures analyzeEdge(const TopoDS_Edge& edge,
                            const TopoDS_Face& face1,
                            const TopoDS_Face& face2,
                            const TopoDS_Solid& solid);

    // 配置参数
    struct Config {
        // 二面角范围（度）
        double minDihedralAngle = 75.0;
        double maxDihedralAngle = 105.0;

        // Edge length filter (mm)
        double minEdgeLength = 50.0;  // Increased to 50mm to exclude small connection features

        // Boundary distance filter (mm)
        double minBoundaryDistance = 5.0;

        // 测试点偏移距离（mm）
        double testPointOffset = 1.0;

        // Numerical tolerance
        double tolerance = 1e-6;

        // Enable debug output
        bool enableDebugOutput = true;
    };

    void setConfig(const Config& config) { m_config = config; }
    const Config& getConfig() const { return m_config; }

private:
    Config m_config;

    // Core algorithm implementation
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

    // Helper functions
    void calculateFaceNormal(const TopoDS_Face& face,
                            const TopoDS_Edge& edge,
                            double normal[3]);

    double getEdgeLength(const TopoDS_Edge& edge);

    void getEdgeMidpoint(const TopoDS_Edge& edge, double point[3]);

    void getEdgeEndpoints(const TopoDS_Edge& edge, double start[3], double end[3]);

    // Debug output
    void printFeatures(const WeldFeatures& features) const;
};