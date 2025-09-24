#pragma once
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <vector>

// 角接接头检测器 (Corner Joint / L-Joint)
// 两个工件成90度角连接，形成L形
class CornerJointDetector {
public:
    struct CornerJoint {
        TopoDS_Edge edge;
        TopoDS_Face face1;
        TopoDS_Face face2;
        double angle;
        double confidence;
    };

public:
    CornerJointDetector();
    ~CornerJointDetector();

    // 检测角接接头
    std::vector<CornerJoint> detect(const TopoDS_Edge& edge,
                                    const TopoDS_Face& face1,
                                    const TopoDS_Face& face2);

    // 判断是否为角接接头
    bool isCornerJoint(const TopoDS_Edge& edge,
                       const TopoDS_Face& face1,
                       const TopoDS_Face& face2);

private:
    double angleToleranceDeg = 15.0;
};