/*** 
 * @Author: lwr
 * @Date: 2025-03-04 15:39:40
 * @LastEditTime: 2025-04-10 16:17:53
 * @LastEditors: lwr
 * @Description: AGV配置文件实现
 * @FilePath: /qr_agv/src/usbcan/include/usbcan/agv_config.h
 */
#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <yaml-cpp/yaml.h>

struct TrayInfo {
    double tray_height; 
    double tray_rotation; 
};

struct AGVInfo {
    int enable;  // 读取的抱闸状态
    int steer_enable; // 读取的转向使能
    int drive_enable;  // 读取的驱动使能
    int16_t speed_command; // 速度指令值
    int16_t steer_command; // 角度指令值
    int operate_mode; // 车辆操作模式  手动0/自动1

};

struct BatteryInfo {
    int       charge_status;      // 充电状态. 0:未充电; 1:正在充电; 2:充电完成; 
    int       charge_allowed;     // 充电是否允许. 0 允许充电 ；1 不允许充电 ；注意不同型号的车协议可能会不同
    int    discharge_allowed;     // 放电是否允许. 0 不允许; 1 允许
    int    charge_connection;
    
    double battery_level;   // 电池组当前容量指数 0-100%
    double battery_life;    // 电池寿命 0-100%
    double total_voltage;   // 电池组当前总电压 0-980V
    double total_current;   // 电池组当前总电流 -1600-1600A
};

struct MapTag {
    int tag_label;
    float x;
    float y;
    float angle;
};

struct VehicleConfig {
    std::string name;
    std::string type;
    std::string id;
    std::string logid; 
    int pgvid;
    float wheel_separation;
};

class AGVConfigManager {
public:
    AGVConfigManager(const AGVConfigManager&) = delete;
    AGVConfigManager& operator=(const AGVConfigManager&) = delete;

    static AGVConfigManager& getInstance() {
        static AGVConfigManager instance;
        return instance;
    }

    bool initialize(const std::string& config_file) {
        std::lock_guard<std::mutex> lock(mutex_);
        try {
            YAML::Node config = YAML::LoadFile(config_file);
            parseMapTags(config["map_tags"]);
            parseVehicles(config["vehicles"]);
            initialized_ = true;
            return true;
        } catch (const std::exception& e) {
            last_error_ = e.what();
            return false;
        }
    }

    const std::vector<MapTag>& getMapTags() const { return map_tags_; }
    const std::vector<VehicleConfig>& getVehicles() const { return vehicles_; }

    const MapTag* findTag(int label) const {  
        auto it = tag_map_.find(label);
        return it != tag_map_.end() ? &it->second : nullptr;
    }

    bool isInitialized() const { return initialized_; }
    std::string getLastError() const { return last_error_; }

private:
    AGVConfigManager() = default;

    void parseMapTags(const YAML::Node& node) {
        for (const auto& tag_node : node) {
            MapTag tag{
                .tag_label = tag_node["tag_label"].as<int>(),  
                .x = tag_node["x"].as<float>(),
                .y = tag_node["y"].as<float>(),
                .angle = tag_node["angle"].as<float>()
            };
            map_tags_.push_back(tag);
            tag_map_[tag.tag_label] = tag; 
        }
    }

    void parseVehicles(const YAML::Node& node) {
        for (const auto& vehicle_node : node) {
            vehicles_.push_back({
                .name = vehicle_node["name"].as<std::string>(),
                .type = vehicle_node["type"].as<std::string>(),
                .id = vehicle_node["id"].as<std::string>(),
                .logid = vehicle_node["logid"].as<std::string>(),
                .pgvid = vehicle_node["pgvid"].as<int>(),
                .wheel_separation = vehicle_node["wheel_separation"].as<float>()
            });
        }
    }

    std::vector<MapTag> map_tags_;
    std::vector<VehicleConfig> vehicles_;
    std::unordered_map<int, MapTag> tag_map_; 
    mutable std::mutex mutex_;
    bool initialized_ = false;
    std::string last_error_;
};