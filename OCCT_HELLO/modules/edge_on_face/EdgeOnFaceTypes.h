#pragma once

#include <vector>
#include <set>
#include <string>

// 边在面上检测模块的类型定义
namespace EdgeOnFace {

    // 配置参数
    struct Config {
        double tolerance = 1e-4;           // 点到面的距离容差
        int samplePoints = 20;             // 边上采样点数
        double matchThreshold = 0.8;       // 匹配阈值（80%）
    };

    // 边-面关系（增强版，包含实体信息）
    struct EdgeFaceRelation {
        // 边信息
        int edgeId;                    // 边索引
        int edgeSolidId;               // 边所属的实体索引
        std::string edgeSolidName;     // 边所属的实体名称

        // 面信息
        int faceId;                    // 面索引
        int faceSolidId;               // 面所属的实体索引
        std::string faceSolidName;     // 面所属的实体名称

        // 匹配信息
        double matchPercentage;        // 匹配百分比
        std::set<int> adjacentFaceIds; // 该边的相邻面

        EdgeFaceRelation() : edgeId(0), edgeSolidId(0), faceId(0), faceSolidId(0), matchPercentage(0) {}
    };

    // 实体间的接触关系
    struct SolidContactInfo {
        int solid1Id;
        std::string solid1Name;
        int solid2Id;
        std::string solid2Name;
        std::vector<EdgeFaceRelation> contacts; // 这两个实体间的所有接触
    };

}