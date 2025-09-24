#include "LapJointDetector.h"

LapJointDetector::LapJointDetector() {}

LapJointDetector::~LapJointDetector() {}

std::vector<LapJointDetector::LapJoint> LapJointDetector::detect(
    const TopoDS_Edge& edge,
    const TopoDS_Face& face1,
    const TopoDS_Face& face2) {

    std::vector<LapJoint> results;

    // TODO: 实现搭接接头检测逻辑
    // 搭接接头特征：
    // 1. 两个工件平行或接近平行
    // 2. 有重叠区域
    // 3. 焊接在重叠边缘

    return results;
}

bool LapJointDetector::isLapJoint(const TopoDS_Edge& edge,
                                  const TopoDS_Face& face1,
                                  const TopoDS_Face& face2) {
    // TODO: 实现判断逻辑
    return false;
}