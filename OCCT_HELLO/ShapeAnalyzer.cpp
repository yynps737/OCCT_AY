#include "ShapeAnalyzer.h"
#include <TopExp.hxx>
#include <iostream>
#include <iomanip>

namespace ShapeAnalyzer {

    // 分析Shape拓扑结构
    ShapeStatistics Analyzer::analyze(const TopoDS_Shape& shape) {
        ShapeStatistics stats = {0, 0, 0, 0, 0, 0, 0, 0, 0, "", false, false};

        // 1. COMPOUND - 复合体
        TopTools_IndexedMapOfShape compounds;
        TopExp::MapShapes(shape, TopAbs_COMPOUND, compounds);
        stats.compoundCount = compounds.Extent();

        // 2. COMPSOLID - 复合实体
        TopTools_IndexedMapOfShape compsolids;
        TopExp::MapShapes(shape, TopAbs_COMPSOLID, compsolids);
        stats.compsolidCount = compsolids.Extent();

        // 3. SOLID - 实体
        TopTools_IndexedMapOfShape solids;
        TopExp::MapShapes(shape, TopAbs_SOLID, solids);
        stats.solidCount = solids.Extent();

        // 4. SHELL - 壳体
        TopTools_IndexedMapOfShape shells;
        TopExp::MapShapes(shape, TopAbs_SHELL, shells);
        stats.shellCount = shells.Extent();

        // 5. FACE - 面
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(shape, TopAbs_FACE, faces);
        stats.faceCount = faces.Extent();

        // 6. WIRE - 线框
        TopTools_IndexedMapOfShape wires;
        TopExp::MapShapes(shape, TopAbs_WIRE, wires);
        stats.wireCount = wires.Extent();

        // 7. EDGE - 边
        TopTools_IndexedMapOfShape edges;
        TopExp::MapShapes(shape, TopAbs_EDGE, edges);
        stats.edgeCount = edges.Extent();

        // 8. VERTEX - 顶点
        TopTools_IndexedMapOfShape vertices;
        TopExp::MapShapes(shape, TopAbs_VERTEX, vertices);
        stats.vertexCount = vertices.Extent();

        // 分析结果
        stats.estimatedParts = estimateParts(stats);
        stats.shapeType = determineShapeType(stats);
        stats.isAssembly = (stats.estimatedParts > 1) || (stats.compoundCount > 0 && stats.solidCount > 1);
        stats.isSinglePart = (stats.estimatedParts == 1);

        this->stats = stats;
        return stats;
    }

    // 判断Shape的主要类型
    std::string Analyzer::determineShapeType(const ShapeStatistics& stats) {
        if (stats.compoundCount > 0 && stats.solidCount > 1) {
            return "Assembly (Multiple solids in compound)";
        }
        else if (stats.compsolidCount > 0) {
            return "Connected Assembly (CompSolid)";
        }
        else if (stats.solidCount > 1) {
            return "Multi-part Model";
        }
        else if (stats.solidCount == 1) {
            return "Single Solid Part";
        }
        else if (stats.shellCount > 0) {
            return "Shell Model (No solid)";
        }
        else if (stats.faceCount > 0) {
            return "Surface Model (Faces only)";
        }
        else if (stats.wireCount > 0) {
            return "Wire Model";
        }
        else if (stats.edgeCount > 0) {
            return "Edge Model";
        }
        else {
            return "Unknown";
        }
    }

    // 估算零件数量
    int Analyzer::estimateParts(const ShapeStatistics& stats) {
        // 优先使用Solid数量
        if (stats.solidCount > 0) {
            return stats.solidCount;
        }

        // 如果没有Solid，检查独立的Shell（可能是薄壁零件）
        if (stats.shellCount > 0) {
            // 如果有多个Shell且没有Solid，每个Shell可能是独立零件
            if (stats.solidCount == 0) {
                return stats.shellCount;
            }
        }

        // 如果有CompSolid，内部的Solid是连接的
        if (stats.compsolidCount > 0) {
            return stats.compsolidCount; // 每个CompSolid算作一个整体
        }

        // 默认返回1（至少是一个对象）
        return stats.solidCount > 0 ? stats.solidCount : 1;
    }

    // 显示详细的分析报告
    void Analyzer::displayReport(const ShapeStatistics& stats) const {
        std::cout << "\n========================================" << std::endl;
        std::cout << "SHAPE TOPOLOGY ANALYSIS (8 Types)" << std::endl;
        std::cout << "========================================" << std::endl;

        // 显示8种拓扑类型
        std::cout << std::left << std::setw(15) << "1. COMPOUND"
                  << ": " << std::right << std::setw(4) << stats.compoundCount
                  << "  (Group of any shapes)" << std::endl;

        std::cout << std::left << std::setw(15) << "2. COMPSOLID"
                  << ": " << std::right << std::setw(4) << stats.compsolidCount
                  << "  (Connected solids)" << std::endl;

        std::cout << std::left << std::setw(15) << "3. SOLID"
                  << ": " << std::right << std::setw(4) << stats.solidCount
                  << "  (3D volumes)" << std::endl;

        std::cout << std::left << std::setw(15) << "4. SHELL"
                  << ": " << std::right << std::setw(4) << stats.shellCount
                  << "  (Connected faces)" << std::endl;

        std::cout << std::left << std::setw(15) << "5. FACE"
                  << ": " << std::right << std::setw(4) << stats.faceCount
                  << "  (2D surfaces)" << std::endl;

        std::cout << std::left << std::setw(15) << "6. WIRE"
                  << ": " << std::right << std::setw(4) << stats.wireCount
                  << "  (Connected edges)" << std::endl;

        std::cout << std::left << std::setw(15) << "7. EDGE"
                  << ": " << std::right << std::setw(4) << stats.edgeCount
                  << "  (1D curves)" << std::endl;

        std::cout << std::left << std::setw(15) << "8. VERTEX"
                  << ": " << std::right << std::setw(4) << stats.vertexCount
                  << "  (0D points)" << std::endl;

        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Shape Type     : " << stats.shapeType << std::endl;
        std::cout << "Estimated Parts: " << stats.estimatedParts << std::endl;
        std::cout << "Is Assembly    : " << (stats.isAssembly ? "Yes" : "No") << std::endl;
        std::cout << "========================================" << std::endl;
    }

    // 显示简要信息
    void Analyzer::displaySummary(const ShapeStatistics& stats) const {
        std::cout << "\nShape Summary: " << stats.shapeType << std::endl;
        std::cout << "Parts: " << stats.estimatedParts
                  << ", Solids: " << stats.solidCount
                  << ", Faces: " << stats.faceCount
                  << ", Edges: " << stats.edgeCount << std::endl;
    }

    // 判断是否需要进行边面检测
    bool Analyzer::shouldPerformEdgeFaceDetection(const ShapeStatistics& stats) const {
        // 只有多零件时才需要检测
        if (stats.estimatedParts <= 1) {
            std::cout << "\n[INFO] Single part detected. Edge-on-face detection not needed." << std::endl;
            std::cout << "        (A single part's edges cannot lie on its own non-adjacent faces)" << std::endl;
            return false;
        }

        std::cout << "\n[INFO] Multi-part model detected (" << stats.estimatedParts
                  << " parts). Edge-on-face detection recommended." << std::endl;
        return true;
    }
}