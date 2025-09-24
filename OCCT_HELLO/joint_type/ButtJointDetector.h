#pragma once
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <vector>

// 对接接头检测器 (Butt Joint)
// 两个工件端对端连接，在同一平面上
class ButtJointDetector {
public:
    struct ButtJoint {
        TopoDS_Edge edge;
        TopoDS_Face face1;
        TopoDS_Face face2;
        double angle;  // 两面夹角
        double confidence;  // 置信度 0-1
    };

public:
    ButtJointDetector();
    ~ButtJointDetector();

    // 检测对接接头
    std::vector<ButtJoint> detect(const TopoDS_Edge& edge,
                                  const TopoDS_Face& face1,
                                  const TopoDS_Face& face2);

    // 判断是否为对接接头
    bool isButtJoint(const TopoDS_Edge& edge,
                     const TopoDS_Face& face1,
                     const TopoDS_Face& face2);

private:
    double angleToleranceDeg = 15.0;  // 角度容差
};