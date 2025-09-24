#pragma once

#include <TopoDS_Shape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <string>

// Shape分析模块 - 负责分析模型的拓扑结构
namespace ShapeAnalyzer {

    // Shape统计信息 - 完整的8种拓扑类型
    struct ShapeStatistics {
        // 8种基本拓扑类型
        int compoundCount;    // 复合体 - 任意类型拓扑对象的组
        int compsolidCount;   // 复合实体 - 通过面连接的实体集合
        int solidCount;       // 实体 - 由壳体限定的三维空间
        int shellCount;       // 壳体 - 通过边连接的面集合
        int faceCount;        // 面 - 二维表面
        int wireCount;        // 线框 - 连接的边集合
        int edgeCount;        // 边 - 一维曲线
        int vertexCount;      // 顶点 - 零维点

        // 分析结果
        int estimatedParts;   // 基于Solid或Shell估算的零件数
        std::string shapeType; // 主要Shape类型描述
        bool isAssembly;      // 是否为装配体
        bool isSinglePart;    // 是否为单个零件
    };

    // 分析器类
    class Analyzer {
    private:
        ShapeStatistics stats;

        // 判断Shape的主要类型
        std::string determineShapeType(const ShapeStatistics& stats);

        // 估算零件数量
        int estimateParts(const ShapeStatistics& stats);

    public:
        // 分析Shape拓扑结构
        ShapeStatistics analyze(const TopoDS_Shape& shape);

        // 获取最近的分析结果
        const ShapeStatistics& getStatistics() const { return stats; }

        // 显示详细的分析报告
        void displayReport(const ShapeStatistics& stats) const;

        // 显示简要信息
        void displaySummary(const ShapeStatistics& stats) const;

        // 判断是否需要进行边面检测
        bool shouldPerformEdgeFaceDetection(const ShapeStatistics& stats) const;
    };
}