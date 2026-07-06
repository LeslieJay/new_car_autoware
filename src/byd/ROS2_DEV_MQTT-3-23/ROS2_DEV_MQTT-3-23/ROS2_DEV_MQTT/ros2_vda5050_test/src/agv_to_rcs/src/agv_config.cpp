#include "agv_config.h"
#include <yaml-cpp/yaml.h>
#include <iostream>

AGVConfig agv_config;

bool load_agv_config(const std::string& path) {
    try {
        YAML::Node config = YAML::LoadFile(path);
        agv_config.vehicle_type = config["vehicle_type"].as<std::string>();
        agv_config.max_linear_velocity = config["max_linear_velocity"].as<double>();
        agv_config.max_angular_velocity = config["max_angular_velocity"].as<double>();
        agv_config.min_battery_level = config["min_battery_level"].as<int>();
        agv_config.max_offset = config["max_offset"].as<double>();
        agv_config.high_distance_precision = config["high_distance_precision"].as<double>();
        agv_config.high_angle_precision = config["high_angle_precision"].as<double>();
        agv_config.low_distance_precision = config["low_distance_precision"].as<double>();
        agv_config.low_angle_precision = config["low_angle_precision"].as<double>();
        agv_config.PI = config["PI"].as<double>();
        agv_config.can_channels = config["can_channels"].as<int>();
        agv_config.can0_frames = config["can0_frames"].as<int>();
        agv_config.can1_frames = config["can1_frames"].as<int>();
        agv_config.gap = config["gap"].as<std::vector<int>>();
        agv_config.static_file_path = config["static_file_path"].as<std::string>();
        agv_config.instant_file_path = config["instant_file_path"].as<std::string>();
        agv_config.candata_file_path = config["candata_file_path"].as<std::string>();
        agv_config.file_path_prefix = config["file_path_prefix"].as<std::string>();
        agv_config.record_batch_size = config["record_batch_size"].as<int>();
        agv_config.version = config["version"].as<std::string>();
        agv_config.manufacturer = config["manufacturer"].as<std::string>();
        agv_config.serial_number = config["serial_number"].as<std::string>();

        // 嵌套结构体
        auto ts = config["type_specification"];
        agv_config.type_specification.series_name = ts["series_name"].as<std::string>();
        agv_config.type_specification.agv_kinematic = ts["agv_kinematic"].as<std::string>();
        agv_config.type_specification.agv_class = ts["agv_class"].as<std::string>();
        agv_config.type_specification.max_load_mass = ts["max_load_mass"].as<int>();
        agv_config.type_specification.navigation_types = ts["navigation_types"].as<std::string>();

        auto pp = config["physical_parameters"];
        agv_config.physical_parameters.speed_min = pp["speed_min"].as<double>();
        agv_config.physical_parameters.speed_max = pp["speed_max"].as<double>();
        agv_config.physical_parameters.acceleration_max = pp["acceleration_max"].as<double>();
        agv_config.physical_parameters.deceleration_max = pp["deceleration_max"].as<double>();
        agv_config.physical_parameters.height_max = pp["height_max"].as<double>();
        agv_config.physical_parameters.width = pp["width"].as<double>();
        agv_config.physical_parameters.length = pp["length"].as<double>();

        auto ap = config["agv_position"];
        agv_config.agv_position.map_id = ap["map_id"].as<std::string>();
        agv_config.agv_position.position_initialized = ap["position_initialized"].as<bool>();

        agv_config.operating_mode = config["operating_mode"].as<std::string>();
        agv_config.fork_running_height = config["fork_running_height"].as<int>();
        agv_config.fork_action_height = config["fork_action_height"].as<int>();
        agv_config.record_duration = config["record_duration"].as<int>();

        // sites 是二维数组
        agv_config.sites.clear();
        for (const auto& site : config["sites"]) {
            std::vector<double> point;
            for (const auto& v : site) {
                point.push_back(v.as<double>());
            }
            agv_config.sites.push_back(point);
        }

        agv_config.sites_num = config["sites_num"].as<int>();
        agv_config.micro_distance = config["micro_distance"].as<double>();
        agv_config.max_pause_time = config["max_pause_time"].as<int>();

        // 通信配置（设置默认值以保持向后兼容）
        agv_config.communication_mode = config["communication_mode"] ? 
            config["communication_mode"].as<std::string>() : "ros2_only";
        agv_config.enable_mqtt_subscription = config["enable_mqtt_subscription"] ? 
            config["enable_mqtt_subscription"].as<bool>() : false;
        agv_config.mqtt_broker_host = config["mqtt_broker_host"] ? 
            config["mqtt_broker_host"].as<std::string>() : "localhost";
        agv_config.mqtt_broker_port = config["mqtt_broker_port"] ? 
            config["mqtt_broker_port"].as<int>() : 1883;
        agv_config.mqtt_username = config["mqtt_username"] ? 
            config["mqtt_username"].as<std::string>() : "";
        agv_config.mqtt_password = config["mqtt_password"] ? 
            config["mqtt_password"].as<std::string>() : "";
        agv_config.mqtt_order_topic = config["mqtt_order_topic"] ? 
            config["mqtt_order_topic"].as<std::string>() : "uagv/order";
        agv_config.mqtt_instant_action_topic = config["mqtt_instant_action_topic"] ? 
            config["mqtt_instant_action_topic"].as<std::string>() : "uagv/instantActions";

        return true;
    } catch (const std::exception& e) {
        std::cerr << "加载配置文件失败: " << e.what() << std::endl;
        return false;
    }
}
