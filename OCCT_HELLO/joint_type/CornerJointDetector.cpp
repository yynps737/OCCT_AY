#include "CornerJointDetector.h"

CornerJointDetector::CornerJointDetector() {}

CornerJointDetector::~CornerJointDetector() {}

std::vector<CornerJointDetector::CornerJoint> CornerJointDetector::detect(
    const TopoDS_Edge& edge,
    const TopoDS_Face& face1,
    const TopoDS_Face& face2) {

    std::vector<CornerJoint> results;

    // TODO: 实现角接接头检测逻辑
    // 角接接头特征：
    // 1. 两个面成90度角
    // 2. 边在两个工件的边缘相交处
    // 3. 形成外角（L形）

    return results;
}

bool CornerJointDetector::isCornerJoint(const TopoDS_Edge& edge,
                                        const TopoDS_Face& face1,
                                        const TopoDS_Face& face2) {
    // TODO: 实现判断逻辑
    return false;
}