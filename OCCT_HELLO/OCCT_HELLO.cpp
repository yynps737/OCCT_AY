// ============================================================================
// OCCT主程序 - 焊接接头检测系统
// ============================================================================
// 主循环：选择模型并进行接头类型检测
// ============================================================================

#include "FileManager.h"
#include "ShapeAnalyzer.h"
#include "modules/edge_on_face/EdgeOnFaceDetector.h"
#include "JointDetector.h"
#include "joint_type/CornerJointDetector.h"
#include <TopExp_Explorer.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Solid.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomAbs_CurveType.hxx>
#include <gp_Pnt.hxx>
#include <iostream>
#include <conio.h>
#include <string>
#include <vector>
#include <set>

// 显示程序标题
void showHeader() {
    std::cout << "========================================" << std::endl;
    std::cout << "     OCCT Joint Detector" << std::endl;
    std::cout << "   Welding Joint Type Analysis" << std::endl;
    std::cout << "========================================" << std::endl;
}

// 显示分析菜单
int showAnalysisMenu() {
    std::cout << "\nSelect Analysis Mode:" << std::endl;
    std::cout << "1. Edge-Face Detection (Classic)" << std::endl;
    std::cout << "2. Butt Joint" << std::endl;
    std::cout << "3. Corner Joint (L-type)" << std::endl;
    std::cout << "4. T-Joint" << std::endl;
    std::cout << "5. Lap Joint" << std::endl;
    std::cout << "6. All Joint Types" << std::endl;
    std::cout << "7. Custom Configuration" << std::endl;
    std::cout << "0. Skip Analysis" << std::endl;
    std::cout << "Choice: ";

    int choice;
    std::cin >> choice;
    return choice;
}

// 配置接头检测器
JointDetector::Config configureJointDetector(int mode) {
    JointDetector::Config config;

    switch (mode) {
        case 2:  // 对接接头
            config.detectButtJoint = true;
            config.detectCornerJoint = false;
            config.detectTJoint = false;
            config.detectLapJoint = false;
            config.detectEdgeJoint = false;
            break;
        case 3:  // 角接接头（L形）
            config.detectButtJoint = false;
            config.detectCornerJoint = true;
            config.detectTJoint = false;
            config.detectLapJoint = false;
            config.detectEdgeJoint = false;
            break;
        case 4:  // T型接头
            config.detectButtJoint = false;
            config.detectCornerJoint = false;
            config.detectTJoint = true;
            config.detectLapJoint = false;
            config.detectEdgeJoint = false;
            break;
        case 5:  // 搭接接头
            config.detectButtJoint = false;
            config.detectCornerJoint = false;
            config.detectTJoint = false;
            config.detectLapJoint = true;
            config.detectEdgeJoint = false;
            break;
        case 6:  // 所有接头类型
            config.detectButtJoint = true;
            config.detectCornerJoint = true;
            config.detectTJoint = true;
            config.detectLapJoint = true;
            config.detectEdgeJoint = true;
            break;
        case 7:  // 自定义配置
            char choice;
            std::cout << "\nCustom Configuration:" << std::endl;

            std::cout << "Detect Butt Joint? (y/n): ";
            std::cin >> choice;
            config.detectButtJoint = (choice == 'y' || choice == 'Y');

            std::cout << "Detect Corner Joint (L-type)? (y/n): ";
            std::cin >> choice;
            config.detectCornerJoint = (choice == 'y' || choice == 'Y');

            std::cout << "Detect T-Joint? (y/n): ";
            std::cin >> choice;
            config.detectTJoint = (choice == 'y' || choice == 'Y');

            std::cout << "Detect Lap Joint? (y/n): ";
            std::cin >> choice;
            config.detectLapJoint = (choice == 'y' || choice == 'Y');

            std::cout << "Detect Edge Joint? (y/n): ";
            std::cin >> choice;
            config.detectEdgeJoint = (choice == 'y' || choice == 'Y');

            std::cout << "Angle Tolerance (degrees, default 15): ";
            std::cin >> config.angleToleranceDeg;
            break;
        default:
            // 默认配置
            break;
    }

    return config;
}

// 分析单个模型
void analyzeModel(const FileManager::ModelFile& model) {
    system("cls");
    std::cout << "========================================" << std::endl;
    std::cout << "     ANALYZING MODEL" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "File: " << model.fileName << std::endl;
    std::cout << "Path: " << model.targetPath << std::endl;
    std::cout << std::endl;

    try {
        // 使用FileManager模块加载STEP文件
        TopoDS_Shape shape = FileManager::loadSTEPFile(model.targetPath);

        // 首先进行Shape拓扑分析
        ShapeAnalyzer::Analyzer shapeAnalyzer;
        auto shapeStats = shapeAnalyzer.analyze(shape);

        // 显示详细的Shape分析报告
        shapeAnalyzer.displayReport(shapeStats);

        // 显示分析菜单（不管单实体还是多实体都需要分析）
        int analysisChoice = showAnalysisMenu();

        // 执行边面检测（经典模式）
        if (analysisChoice == 1) {
            std::cout << "\n--- EDGE-ON-FACE DETECTION ---" << std::endl;

            // 创建边面检测器并设置参数
            EdgeOnFace::Config config;
            config.tolerance = 1e-4;
            config.samplePoints = 20;
            config.matchThreshold = 0.8;

            EdgeOnFace::Detector detector(config);

            // 执行边面检测
            std::vector<EdgeOnFace::EdgeFaceRelation> results = detector.analyze(shape);

            // 显示基本结果
            EdgeOnFace::displayResults(results);

            // 按实体分组并显示详细信息
            if (!results.empty()) {
                auto solidContacts = detector.groupBySolids(results);
                EdgeOnFace::displayDetailedResults(results, solidContacts);
            }
        }
        // 执行接头检测（新模式）
        else if (analysisChoice >= 2 && analysisChoice <= 7) {
            std::cout << "\n--- JOINT DETECTION ---" << std::endl;

            // 创建接头检测器
            JointDetector detector;

            // 根据选择配置检测器
            JointDetector::Config config = configureJointDetector(analysisChoice);
            detector.setConfig(config);

            // 执行接头检测
            std::vector<JointDetector::JointData> joints = detector.detectJoints(shape);

            // 显示结果
            JointDetector::displayResults(joints);

            // Export HTML visualization for corner joints
            if (config.detectCornerJoint) {
                std::cout << "\nGenerating HTML visualization..." << std::endl;

                // Create corner detector for visualization
                CornerJointDetector* cornerDetector = new CornerJointDetector();

                // Collect ALL edges from the shape for complete visualization
                std::vector<CornerJointDetector::WeldFeatures> allEdgeFeatures;

                // Now iterate through ALL edges in the shape
                // Use TopTools_IndexedMapOfShape to avoid duplicates
                TopTools_IndexedMapOfShape edgeMap;
                TopExp::MapShapes(shape, TopAbs_EDGE, edgeMap);

                int featureIndex = 0;
                int cornerJointCount = 0;
                int totalEdgeCount = edgeMap.Extent();

                // Track which joints have been matched to avoid double counting
                std::set<const JointDetector::JointData*> matchedJoints;

                for (int i = 1; i <= edgeMap.Extent(); i++) {
                    const TopoDS_Edge& edge = TopoDS::Edge(edgeMap(i));

                    CornerJointDetector::WeldFeatures feature;

                    // Get edge geometry
                    BRepAdaptor_Curve curveAdaptor(edge);
                    double edgeLength = GCPnts_AbscissaPoint::Length(curveAdaptor);

                    // Skip very small edges (< 1mm)
                    if (edgeLength < 1.0) continue;

                    // Get edge midpoint
                    double first = curveAdaptor.FirstParameter();
                    double last = curveAdaptor.LastParameter();
                    double mid = (first + last) / 2.0;
                    gp_Pnt midPnt = curveAdaptor.Value(mid);

                    // Get edge endpoints
                    gp_Pnt startPnt = curveAdaptor.Value(first);
                    gp_Pnt endPnt = curveAdaptor.Value(last);

                    // Fill feature data
                    feature.edgeId = ++featureIndex;
                    feature.edgeLength = edgeLength;

                    // Check if this edge is a detected corner joint
                    bool isCornerJoint = false;
                    for (const auto& joint : joints) {
                        if (joint.type == JointDetector::CORNER_JOINT &&
                            joint.edge.IsSame(edge)) {
                            isCornerJoint = true;
                            feature.face1Id = joint.solid1Id;
                            feature.face2Id = joint.solid2Id;
                            feature.dihedralAngle = joint.angle;
                            feature.confidence = joint.confidence;

                            // Only count if not already matched
                            if (matchedJoints.find(&joint) == matchedJoints.end()) {
                                cornerJointCount++;
                                matchedJoints.insert(&joint);
                            }
                            break;
                        }
                    }

                    feature.isCornerWeld = isCornerJoint;
                    if (!isCornerJoint) {
                        // Non-weld edges get default values
                        feature.face1Id = 0;
                        feature.face2Id = 0;
                        feature.dihedralAngle = 0;
                        feature.confidence = 0;
                    }

                    // Set coordinates
                    feature.edgeMidpoint[0] = midPnt.X();
                    feature.edgeMidpoint[1] = midPnt.Y();
                    feature.edgeMidpoint[2] = midPnt.Z();

                    feature.edgeStart[0] = startPnt.X();
                    feature.edgeStart[1] = startPnt.Y();
                    feature.edgeStart[2] = startPnt.Z();

                    feature.edgeEnd[0] = endPnt.X();
                    feature.edgeEnd[1] = endPnt.Y();
                    feature.edgeEnd[2] = endPnt.Z();

                    // Check edge type
                    GeomAbs_CurveType curveType = curveAdaptor.GetType();
                    feature.edgeType = (curveType == GeomAbs_Line) ? 0 :
                                      (curveType == GeomAbs_Circle) ? 1 :
                                      (curveType == GeomAbs_Ellipse) ? 2 :
                                      (curveType == GeomAbs_BSplineCurve || curveType == GeomAbs_BezierCurve) ? 3 : 4;

                    // For curved edges, sample points along the curve
                    if (feature.edgeType != 0) {
                        int numSamples = 20;
                        for (int i = 0; i <= numSamples; ++i) {
                            double t = first + (last - first) * i / numSamples;
                            gp_Pnt pt = curveAdaptor.Value(t);
                            std::vector<double> point = {pt.X(), pt.Y(), pt.Z()};
                            feature.curvePoints.push_back(point);
                        }
                    }

                    allEdgeFeatures.push_back(feature);
                }

                // Export to HTML with ALL edges
                std::string htmlPath = "C:\\hgtech\\OCCT\\test\\OCCT_HELLO\\weld_visualization.html";
                if (cornerDetector->exportVisualizationHTML(allEdgeFeatures, htmlPath)) {
                    std::cout << "Visualization saved to: " << htmlPath << std::endl;
                    std::cout << "Open the file in a web browser to view the 3D visualization." << std::endl;
                    std::cout << "Total edges in model: " << totalEdgeCount << std::endl;
                    std::cout << "Edges displayed: " << allEdgeFeatures.size() << " (excluding edges < 1mm)" << std::endl;
                    std::cout << "Corner joints detected: " << cornerJointCount << std::endl;
                }

                delete cornerDetector;
            }
        }

        std::cout << "\nAnalysis complete for: " << model.fileName << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "\nError analyzing file: " << e.what() << std::endl;
    }
}

// 主程序
int main() {
    try {
        showHeader();

        // 源目录路径
        const std::string sourceDir = "C:\\hgtech\\OCCT\\test\\OCCT_HELLO\\test_models";

        // 使用FileManager模块扫描模型文件
        std::cout << "\nScanning directory: " << sourceDir << std::endl;
        std::vector<FileManager::ModelFile> models = FileManager::scanModelDirectory(sourceDir);

        if (models.empty()) {
            std::cout << "No STEP/STP files found in " << sourceDir << std::endl;
            std::cout << "Press any key to exit..." << std::endl;
            _getch();
            return 1;
        }

        std::cout << "Found " << models.size() << " model file(s)" << std::endl;

        // 复制文件到输出目录
        std::cout << "\nCopying models to output directory..." << std::endl;
        FileManager::copyModelsToOutputDir(models);

        // 主循环
        bool continueAnalysis = true;
        while (continueAnalysis) {
            // 使用FileManager模块选择模型
            int selectedIndex = FileManager::selectModel(models);

            if (selectedIndex == -1) {
                std::cout << "\nExiting..." << std::endl;
                break;
            }

            // 分析选中的模型
            analyzeModel(models[selectedIndex]);

            // 询问是否继续
            std::cout << "\n========================================" << std::endl;
            std::cout << "Press ENTER to select another model, ESC to exit..." << std::endl;

            int key = _getch();
            if (key == 27) {  // ESC
                continueAnalysis = false;
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        std::cout << "Press any key to exit..." << std::endl;
        _getch();
        return 1;
    }

    return 0;
}