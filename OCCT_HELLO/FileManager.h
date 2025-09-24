#pragma once

#include <string>
#include <vector>
#include <filesystem>
#include <TopoDS_Shape.hxx>

// 文件管理模块 - 负责STEP文件的扫描、复制和加载
namespace FileManager {

    // 模型文件信息
    struct ModelFile {
        std::string fileName;
        std::string fullPath;
        std::string targetPath;
        size_t fileSize;
    };

    // 扫描目录中的STEP文件
    std::vector<ModelFile> scanModelDirectory(const std::string& sourceDir);

    // 复制模型文件到输出目录
    void copyModelsToOutputDir(std::vector<ModelFile>& models);

    // 读取STEP文件
    TopoDS_Shape loadSTEPFile(const std::string& filePath);

    // 显示模型选择菜单
    int selectModel(const std::vector<ModelFile>& models);
}