#include "EdgeJointDetector.h"

EdgeJointDetector::EdgeJointDetector() {}

EdgeJointDetector::~EdgeJointDetector() {}

std::vector<EdgeJointDetector::EdgeJoint> EdgeJointDetector::detect(
    const TopoDS_Edge& edge,
    const TopoDS_Face& face1,
    const TopoDS_Face& face2) {

    std::vector<EdgeJoint> results;

    // TODO: 实现边缘接头检测逻辑
    // 边缘接头特征：
    // 1. 两个工件的边缘平行
    // 2. 通常用于薄板
    // 3. 焊接在边缘处

    return results;
}

bool EdgeJointDetector::isEdgeJoint(const TopoDS_Edge& edge,
                                    const TopoDS_Face& face1,
                                    const TopoDS_Face& face2) {
    // TODO: 实现判断逻辑
    return false;
}