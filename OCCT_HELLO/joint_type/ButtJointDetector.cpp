#include "ButtJointDetector.h"

ButtJointDetector::ButtJointDetector() {}

ButtJointDetector::~ButtJointDetector() {}

std::vector<ButtJointDetector::ButtJoint> ButtJointDetector::detect(
    const TopoDS_Edge& edge,
    const TopoDS_Face& face1,
    const TopoDS_Face& face2) {

    std::vector<ButtJoint> results;

    // TODO: 实现对接接头检测逻辑
    // 对接接头特征：
    // 1. 两个面几乎共面（角度接近0°或180°）
    // 2. 边在两个工件的端部
    // 3. 工件厚度相近

    return results;
}

bool ButtJointDetector::isButtJoint(const TopoDS_Edge& edge,
                                    const TopoDS_Face& face1,
                                    const TopoDS_Face& face2) {
    // TODO: 实现判断逻辑
    return false;
}