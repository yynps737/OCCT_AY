#include "TJointDetector.h"

TJointDetector::TJointDetector() {}

TJointDetector::~TJointDetector() {}

std::vector<TJointDetector::TJoint> TJointDetector::detect(
    const TopoDS_Edge& edge,
    const TopoDS_Face& face1,
    const TopoDS_Face& face2) {

    std::vector<TJoint> results;

    // TODO: 实现T型接头检测逻辑
    // T型接头特征：
    // 1. 两个面成90度角
    // 2. 一个工件的端部连接到另一个工件的中间
    // 3. 形成内角（T形）

    return results;
}

bool TJointDetector::isTJoint(const TopoDS_Edge& edge,
                              const TopoDS_Face& face1,
                              const TopoDS_Face& face2) {
    // TODO: 实现判断逻辑
    return false;
}