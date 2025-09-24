#include "FileManager.h"
#include <STEPControl_Reader.hxx>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <conio.h>
#include <Windows.h>

namespace fs = std::filesystem;

namespace FileManager {

    // 扫描目录中的STEP文件
    std::vector<ModelFile> scanModelDirectory(const std::string& sourceDir) {
        std::vector<ModelFile> models;

        if (!fs::exists(sourceDir)) {
            std::cout << "Warning: Source directory does not exist: " << sourceDir << std::endl;
            return models;
        }

        for (const auto& entry : fs::directory_iterator(sourceDir)) {
            if (entry.is_regular_file()) {
                std::string ext = entry.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

                if (ext == ".step" || ext == ".stp") {
                    ModelFile model;
                    model.fileName = entry.path().filename().string();
                    model.fullPath = entry.path().string();
                    model.fileSize = fs::file_size(entry);
                    models.push_back(model);
                }
            }
        }

        // 按文件名排序
        std::sort(models.begin(), models.end(),
                  [](const ModelFile& a, const ModelFile& b) {
                      return a.fileName < b.fileName;
                  });

        return models;
    }

    // 复制模型文件到输出目录
    void copyModelsToOutputDir(std::vector<ModelFile>& models) {
        // 获取可执行文件目录
        char exePath[MAX_PATH];
        GetModuleFileNameA(NULL, exePath, MAX_PATH);
        fs::path exeDir = fs::path(exePath).parent_path();

        // 创建models子目录
        fs::path modelsDir = exeDir / "models";
        if (!fs::exists(modelsDir)) {
            fs::create_directories(modelsDir);
            std::cout << "Created models directory: " << modelsDir << std::endl;
        }

        // 复制文件
        for (auto& model : models) {
            fs::path targetPath = modelsDir / model.fileName;
            model.targetPath = targetPath.string();

            try {
                if (!fs::exists(targetPath) ||
                    fs::last_write_time(model.fullPath) > fs::last_write_time(targetPath)) {
                    fs::copy_file(model.fullPath, targetPath,
                                 fs::copy_options::overwrite_existing);
                    std::cout << "Copied: " << model.fileName << std::endl;
                }
            } catch (const std::exception& e) {
                std::cout << "Failed to copy " << model.fileName << ": " << e.what() << std::endl;
            }
        }
    }

    // 读取STEP文件
    TopoDS_Shape loadSTEPFile(const std::string& filePath) {
        STEPControl_Reader reader;

        if (reader.ReadFile(filePath.c_str()) != IFSelect_RetDone) {
            throw std::runtime_error("Failed to read STEP file: " + filePath);
        }

        if (reader.TransferRoots() <= 0) {
            throw std::runtime_error("No entities transferred from STEP file");
        }

        TopoDS_Shape shape = reader.OneShape();
        if (shape.IsNull()) {
            throw std::runtime_error("Retrieved null shape from STEP file");
        }

        std::cout << "Successfully loaded: " << fs::path(filePath).filename().string() << std::endl;
        return shape;
    }

    // 显示模型选择菜单
    int selectModel(const std::vector<ModelFile>& models) {
        if (models.empty()) {
            std::cout << "No STEP/STP files found!" << std::endl;
            return -1;
        }

        int selected = 0;
        bool selecting = true;

        while (selecting) {
            // 清屏 (Windows)
            system("cls");

            std::cout << "========================================" << std::endl;
            std::cout << "        SELECT MODEL FILE" << std::endl;
            std::cout << "========================================" << std::endl;
            std::cout << "Use UP/DOWN arrows to select, ENTER to confirm, ESC to exit\n" << std::endl;

            // 显示文件列表
            for (size_t i = 0; i < models.size(); ++i) {
                if (i == selected) {
                    std::cout << " >> ";
                } else {
                    std::cout << "    ";
                }

                std::cout << std::left << std::setw(30) << models[i].fileName;

                // 显示文件大小
                double sizeKB = models[i].fileSize / 1024.0;
                std::cout << std::right << std::setw(10) << std::fixed << std::setprecision(1)
                         << sizeKB << " KB" << std::endl;
            }

            std::cout << "\nSelected: " << models[selected].fileName << std::endl;

            // 获取键盘输入
            int key = _getch();
            if (key == 0 || key == 224) {  // 特殊键
                key = _getch();
                switch (key) {
                    case 72:  // 上箭头
                        if (selected > 0) selected--;
                        break;
                    case 80:  // 下箭头
                        if (selected < static_cast<int>(models.size()) - 1) selected++;
                        break;
                }
            } else {
                switch (key) {
                    case 13:  // Enter
                        selecting = false;
                        break;
                    case 27:  // ESC
                        return -1;
                }
            }
        }

        return selected;
    }
}