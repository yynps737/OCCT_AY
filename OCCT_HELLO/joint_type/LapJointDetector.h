#pragma once
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <vector>

// 搭接接头检测器 (Lap Joint)
// 两个工件部分重叠连接
class LapJointDetector {
public:
    struct LapJoint {
        TopoDS_Edge edge;
        TopoDS_Face face1;
        TopoDS_Face face2;
        double angle;
        double overlapLength;  // 重叠长度
        double confidence;
    };

public:
    LapJointDetector();
    ~LapJointDetector();

    // 检测搭接接头
    std::vector<LapJoint> detect(const TopoDS_Edge& edge,
                                 const TopoDS_Face& face1,
                                 const TopoDS_Face& face2);

    // 判断是否为搭接接头
    bool isLapJoint(const TopoDS_Edge& edge,
                    const TopoDS_Face& face1,
                    const TopoDS_Face& face2);

private:
    double angleToleranceDeg = 15.0;
};