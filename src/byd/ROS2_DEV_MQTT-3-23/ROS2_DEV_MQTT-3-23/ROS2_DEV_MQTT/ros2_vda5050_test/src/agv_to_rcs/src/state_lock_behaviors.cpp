/************************************** File Info ****************************************
* @file:       state_lock_behaviors.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-11                                        
* @version:    V0.0                                                                              
* @brief:      agv锁死状态，待完善
******************************************************************************************/
# include "state_lock_behaviors.h"
# include "agv_config.h"
# include "control_agv_driver.h"
# include <filesystem>
# include <chrono>
# include <iomanip>
# include <sstream>
# include <thread>
# include <cstdlib>
# include <vector>
# include <string>

LockStateBehaviors::LockStateBehaviors(const std::string& name, const NodeConfig& config, AGVBone agv_bone):
    SyncActionNode(name, config),
    current_pose_listener(agv_bone.agv_current_pose_listener),
    instant_action_listener(agv_bone.agv_instant_action_listener),
    data_publish(agv_bone.agv_data_publish),
    can_data_listener_(agv_bone.agv_can_data_listener){}

/*****************************************************************************************
* @brief:      删除7天以前的备份文件夹
* @param:      无
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-11
* @version:    V0.0
******************************************************************************************/
void LockStateBehaviors::deleteOldBackupFolders()
{
    try {
        // 获取当前时间并转换为本地时间
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        std::tm* local_now = std::localtime(&now_time_t);
        
        // 计算7天前的时间点（使用本地时间）
        std::tm seven_days_ago_tm = *local_now;
        seven_days_ago_tm.tm_mday -= 7;
        auto seven_days_ago_time_t = std::mktime(&seven_days_ago_tm);
        
        // 检查目录是否存在
        if (!std::filesystem::exists(agv_config.file_path_prefix)) {
            std::cout << "目录不存在: " << agv_config.file_path_prefix << std::endl;
            return;
        }
        
        // 遍历目录下的所有条目
        for (const auto& entry : std::filesystem::directory_iterator(agv_config.file_path_prefix)) {
            if (entry.is_directory()) {
                std::string folder_name = entry.path().filename().string();
                
                // 跳过 "latest_data" 文件夹
                if (folder_name == "latest_data") {
                    continue;
                }
                
                // 尝试解析文件夹名称（格式：YYYY-MM-DD_HH-MM）
                std::tm tm = {};
                std::istringstream ss(folder_name);
                if (ss >> std::get_time(&tm, "%Y-%m-%d_%H-%M")) {
                    // 将解析的时间转换为 time_t（本地时间）
                    auto folder_time_t = std::mktime(&tm);
                    
                    // 如果文件夹时间早于7天前，则删除
                    if (folder_time_t < seven_days_ago_time_t) {
                        try {
                            std::filesystem::remove_all(entry.path());
                            std::cout << "删除7天前的备份文件夹: " << entry.path() << std::endl;
                        } catch (const std::filesystem::filesystem_error& ex) {
                            std::cout << "删除文件夹失败: " << entry.path() << ", 错误: " << ex.what() << std::endl;
                        }
                    }
                }
                // 如果文件夹名称不符合时间戳格式，则跳过（可能是其他类型的文件夹）
            }
        }
    } catch (const std::filesystem::filesystem_error& ex) {
        std::cout << "遍历目录时发生错误: " << ex.what() << std::endl;
    } catch (const std::exception& ex) {
        std::cout << "删除旧备份文件夹时发生错误: " << ex.what() << std::endl;
    }
}



NodeStatus LockStateBehaviors::tick()
{
    // 预备工作*****************
    // 先保存黑盒数据 此处需要补全代码
    
    // 删除7天以前的备份文件夹
    deleteOldBackupFolders();
    
    // 获取当前时间并格式化为文件夹名
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d_%H-%M");
    std::string time_folder_name = ss.str();
    
    // 创建以当前时间命名的文件夹路径
    std::string backup_folder_path = agv_config.file_path_prefix + time_folder_name;
    std::string source_files_path = agv_config.file_path_prefix + "latest_data";
    
    try {
        // 创建时间命名的文件夹
        std::filesystem::create_directories(backup_folder_path);
        std::cout << "创建备份文件夹: " << backup_folder_path << std::endl;
        
        // 递归复制files文件夹的所有内容到新文件夹
        if (std::filesystem::exists(source_files_path)) {
            std::filesystem::copy(source_files_path, backup_folder_path, 
                                std::filesystem::copy_options::recursive | 
                                std::filesystem::copy_options::overwrite_existing);
            std::cout << "成功备份文件从 " << source_files_path << " 到 " << backup_folder_path << std::endl;
        } else {
            std::cout << "警告: 源文件夹不存在: " << source_files_path << std::endl;
        }

        // 将 ~/.ros/Log 下的指定日志保存至备份文件夹
        const char* home = std::getenv("HOME");
        if (home) {
            std::string ros_log_dir = std::string(home) + "/.ros/Log";
            const std::vector<std::string> log_files = {
                "description.log", "usbcan.log", "end_guide.log", "vda.log", "hins.log"
            };
            for (const auto& name : log_files) {
                std::filesystem::path src = std::filesystem::path(ros_log_dir) / name;
                if (std::filesystem::exists(src)) {
                    std::filesystem::path dst = std::filesystem::path(backup_folder_path) / name;
                    std::filesystem::copy(src, dst, std::filesystem::copy_options::overwrite_existing);
                    std::cout << "已备份日志: " << name << " -> " << backup_folder_path << std::endl;
                } else {
                    std::cout << "日志不存在，跳过: " << src.string() << std::endl;
                }
            }
        } else {
            std::cout << "警告: 无法获取 HOME，跳过 ros Log 备份" << std::endl;
        }
    } catch (const std::filesystem::filesystem_error& ex) {
        std::cout << "文件操作错误: " << ex.what() << std::endl;
    }
    

    // 主逻辑*****************
    std::string fault_code;
    bool success = this->config().blackboard->get("fault_code", fault_code);
    if (success)
    {
        std::cout << "成功获取错误码：" << fault_code << std::endl;
    }
    else
    {
        std::cout << "获取错误码失败！！！" << std::endl;
        return NodeStatus::FAILURE;
    }

    data_publish->state_timer_callback(fault_code);

    if(fault_code == "NAVIGATION_LOST"){
        std::cout << this->name() << "小车丢失导航，需要人工恢复" << std::endl;
        OnManualRecovery();
    }
    else if(fault_code == "VELOCITY_OVER_LIMIT"){
        std::cout << this->name() << "小车速度超限，需要人工恢复" << std::endl;
        OnManualRecovery();
    }
    else if(fault_code == "ANGULAR_VELOCITY_OVER_LIMIT"){
        std::cout << this->name() << "小车角速度超限，需要人工恢复" << std::endl;
        OnManualRecovery();
    }
    else if(fault_code == "BATTERY_LOW"){
        std::cout << this->name() << "小车电量过低，需要人工恢复" << std::endl;
        OnManualRecovery();
    }
    else if(fault_code == "PATH_OFFSET_TOO_LARGE"){
        std::cout << this->name() << "小车路径偏移过大，需要人工恢复" << std::endl;
        OnManualRecovery();
    }
    else if(fault_code == "LOADING_CONFLICT"){
        std::cout << this->name() << "小车装载冲突，需要人工恢复" << std::endl;
        OnManualRecovery();
    }
    else if(fault_code == "OBSTACLE_DANGER"){
        std::cout << this->name() << "小车障碍物危险，需要人工恢复" << std::endl;
        OnManualRecovery();
    }
    else if(fault_code == "AGV_OFFLINE"){
        std::cout << this->name() << "小车下线，需要人工恢复" << std::endl;
        OnManualRecovery();
    }
    else if(fault_code == "INIT_FAILED"){
        std::cout << this->name() << "小车初始化失败，需要人工恢复" << std::endl;
        OnManualRecovery();
    }
    else if(fault_code == "CAN_DATA_ERROR"){
        std::cout << this->name() << "小车CAN数据异常，需要人工恢复" << std::endl;
        OnManualRecovery();
    }
    else
    {
        std::cout << "fault_code: " << fault_code << std::endl;
        std::cout << this->name() << "特殊紧急情况，需要人工恢复" << std::endl;
        OnManualRecovery();
    }
    // 收尾工作*****************
    this->config().blackboard->set("fault_code", "NONE");
    return NodeStatus::SUCCESS;
}


void LockStateBehaviors::OnManualRecovery(){

    // 小车发生重大故障，需要下线 发送下线消息到RCS 以下是相关代码
    data_publish->publish_connection_request("OFFLINE");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000*5));
    data_publish->cancel_connection_timer(); // 及时关闭下线定时器
    instant_action_listener->action_type = "no_message_received"; // 设置action_type为no_message_received

    // io_data.auto_manual_switch=0:手动模式，1:自动模式
    while(can_data_listener_->get_operation_mode() != 0){
        std::cout << "小车未进入手动模式，等待人工干预" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    std::cout << "小车已进入手动模式，开始人工处理" << std::endl;

    recovery_flag = manual_recovery();
    if(recovery_flag){
        std::cout << "人工处理完毕！准备重新初始化" << std::endl;
        this->config().blackboard->set("last_state", "lock");
        this->config().blackboard->set("current_state", "init");
        this->config().blackboard->set("AGV_Event", "init_try");
        this->config().blackboard->set("fault_code", "NONE");
    }
    else{
        std::cout << "人工处理失败，即将进入错误状态！" << std::endl;
        this->config().blackboard->set("last_state", "lock");
        this->config().blackboard->set("current_state", "error");
        this->config().blackboard->set("AGV_Event", "manual_recovery_failed");
    }
}


/**
 * 这个函数用于人工处理锁死状态
 * @return 执行完毕返回true，否则false
 */
bool LockStateBehaviors::manual_recovery(){
    // 人工处理锁死状态，执行完毕返回true，否则false

    InterruptOrderMessage interrupt_order_message_;

    while(current_pose_listener->get_pose == false || can_data_listener_->get_operation_mode() != 1) // TODO：后续操作模式需要从can报文里面获取
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "系统尚未完全恢复，请求人工恢复！");
        // 休眠5s 
        std::this_thread::sleep_for(std::chrono::milliseconds(1000*5));

        // 有的时候rcs会在锁死态时取消任务，需要监测,但由于小车已经离线，所以无需上报完成取消任务。
        interrupt_order_message_ = instant_action_listener->get_interrupt_order_message();
        if(interrupt_order_message_.action_type == "cancelOrder" && interrupt_order_message_.action_status == "WAITING") // 表明现在要取消任务
        {
            instant_action_listener->update_interrupt_order_message_status("FINISHED");
            interrupt_order_message_.action_status = "FINISHED";
        }

    }

    return true;
}
