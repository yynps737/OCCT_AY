#pragma once

#include "EdgeOnFaceTypes.h"
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <map>
#include <set>

// 边在面上检测模块 - 负责检测边是否在非相邻面上
namespace EdgeOnFace {

    // 检测器类
    class Detector {
    private:
        Config config;
        TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;
        TopTools_IndexedMapOfShape faces;
        TopTools_IndexedMapOfShape edges;
        TopTools_IndexedMapOfShape solids;

        // 实体归属映射
        std::map<int, int> edgeToSolid;    // 边ID -> 实体ID
        std::map<int, int> faceToSolid;    // 面ID -> 实体ID
        std::map<int, std::string> solidNames; // 实体ID -> 名称

        // 构建拓扑关系
        void buildTopology(const TopoDS_Shape& shape);

        // 构建实体归属映射
        void buildSolidOwnership(const TopoDS_Shape& shape);

        // 获取边的相邻面
        std::set<int> getAdjacentFaces(int edgeIndex);

        // 核心算法: 检查边是否在面上
        bool isEdgeOnFace(const TopoDS_Edge& edge, const TopoDS_Face& face,
                         double& matchPercentage);

    public:
        // 构造函数
        explicit Detector(const Config& cfg = Config()) : config(cfg) {}

        // 设置配置
        void setConfig(const Config& cfg) { config = cfg; }

        // 主分析函数
        std::vector<EdgeFaceRelation> analyze(const TopoDS_Shape& shape);

        // 获取统计信息
        int getEdgeCount() const { return edges.Extent(); }
        int getFaceCount() const { return faces.Extent(); }
        int getSolidCount() const { return solids.Extent(); }

        // 按实体分组接触关系
        std::vector<SolidContactInfo> groupBySolids(const std::vector<EdgeFaceRelation>& results);
    };

    // 显示分析结果
    void displayResults(const std::vector<EdgeFaceRelation>& results);

    // 显示详细的实体间接触信息
    void displayDetailedResults(const std::vector<EdgeFaceRelation>& results,
                                const std::vector<SolidContactInfo>& solidContacts);
}