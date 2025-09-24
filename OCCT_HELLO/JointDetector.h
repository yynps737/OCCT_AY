#pragma once

#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include "modules/edge_on_face/EdgeOnFaceDetector.h"
#include <vector>
#include <map>
#include <string>

// 前向声明
class ButtJointDetector;
class CornerJointDetector;
class TJointDetector;
class LapJointDetector;
class EdgeJointDetector;

// 焊接接头综合检测器
class JointDetector {
public:
    // 接头类型枚举
    enum JointType {
        BUTT_JOINT,     // 对接接头
        CORNER_JOINT,   // 角接接头 (L形)
        T_JOINT,        // T型接头
        LAP_JOINT,      // 搭接接头
        EDGE_JOINT,     // 边缘接头
        UNKNOWN_JOINT   // 未知类型
    };

    // 接头数据
    struct JointData {
        JointType type;
        TopoDS_Edge edge;
        TopoDS_Face face1;
        TopoDS_Face face2;
        double angle;
        double confidence;
        std::string typeName;
        int solid1Id;
        int solid2Id;
        std::string solid1Name;
        std::string solid2Name;
    };

    // 配置
    struct Config {
        bool detectButtJoint = true;
        bool detectCornerJoint = true;
        bool detectTJoint = true;
        bool detectLapJoint = true;
        bool detectEdgeJoint = false;
        double angleToleranceDeg = 15.0;
    };

public:
    JointDetector();
    ~JointDetector();

    // 设置配置
    void setConfig(const Config& cfg) { config = cfg; }

    // 主检测函数
    std::vector<JointData> detectJoints(const TopoDS_Shape& shape);

    // 单实体检测
    std::vector<JointData> detectJointsInSinglePart(const TopoDS_Shape& shape);

    // 多实体检测
    std::vector<JointData> detectJointsInMultiPart(const TopoDS_Shape& shape);

    // 显示结果
    static void displayResults(const std::vector<JointData>& joints);

    // 获取接头类型名称
    static std::string getJointTypeName(JointType type);

private:
    Config config;

    // 各类型检测器
    ButtJointDetector* buttDetector;
    CornerJointDetector* cornerDetector;
    TJointDetector* tDetector;
    LapJointDetector* lapDetector;
    EdgeJointDetector* edgeDetector;

    // 拓扑映射
    TopTools_IndexedMapOfShape edges;
    TopTools_IndexedMapOfShape faces;
    TopTools_IndexedMapOfShape solids;
    TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;

    // 归属映射
    std::map<int, int> faceToSolid;
    std::map<int, int> edgeToSolid;
    std::map<int, std::string> solidNames;

    // 初始化拓扑数据
    void initialize(const TopoDS_Shape& shape);

    // 获取边的相邻面
    std::vector<TopoDS_Face> getAdjacentFaces(const TopoDS_Edge& edge);

    // 计算两面夹角
    double calculateAngle(const TopoDS_Face& face1, const TopoDS_Face& face2);

    // 识别接头类型
    JointType identifyJointType(const TopoDS_Edge& edge,
                                const TopoDS_Face& face1,
                                const TopoDS_Face& face2);
};