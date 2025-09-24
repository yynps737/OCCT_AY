#pragma once
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <vector>

// T型接头检测器 (T-Joint)
// 一个工件垂直连接到另一个工件的表面，形成T形
class TJointDetector {
public:
    struct TJoint {
        TopoDS_Edge edge;
        TopoDS_Face face1;
        TopoDS_Face face2;
        double angle;
        double confidence;
    };

public:
    TJointDetector();
    ~TJointDetector();

    // 检测T型接头
    std::vector<TJoint> detect(const TopoDS_Edge& edge,
                               const TopoDS_Face& face1,
                               const TopoDS_Face& face2);

    // 判断是否为T型接头
    bool isTJoint(const TopoDS_Edge& edge,
                  const TopoDS_Face& face1,
                  const TopoDS_Face& face2);

private:
    double angleToleranceDeg = 15.0;
};