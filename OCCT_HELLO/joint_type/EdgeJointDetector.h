#pragma once
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <vector>

// 边缘接头检测器 (Edge Joint)
// 两个工件边缘对边缘连接
class EdgeJointDetector {
public:
    struct EdgeJoint {
        TopoDS_Edge edge;
        TopoDS_Face face1;
        TopoDS_Face face2;
        double angle;
        double confidence;
    };

public:
    EdgeJointDetector();
    ~EdgeJointDetector();

    // 检测边缘接头
    std::vector<EdgeJoint> detect(const TopoDS_Edge& edge,
                                  const TopoDS_Face& face1,
                                  const TopoDS_Face& face2);

    // 判断是否为边缘接头
    bool isEdgeJoint(const TopoDS_Edge& edge,
                     const TopoDS_Face& face1,
                     const TopoDS_Face& face2);

private:
    double angleToleranceDeg = 15.0;
};