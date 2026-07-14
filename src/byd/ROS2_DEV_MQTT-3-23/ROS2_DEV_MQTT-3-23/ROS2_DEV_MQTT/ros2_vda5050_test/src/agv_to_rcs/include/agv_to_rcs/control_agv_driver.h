/************************************** File Info ****************************************
* @file:       control_agv_driver.h   
* @author:     刘鸿彬        
* @date:       2024-11-07   
* @version:    V0.0      
* @brief:      驱动控制接口实现头文件
******************************************************************************************/

# ifndef CONTROL_AGV_DRIVER_H
# define CONTROL_AGV_DRIVER_H

# include "agv_bone.h"
# include "client_send_multi_poses.h"

// 包含vector头文件
#include <vector>
#include <mutex>
#include <memory>

class LaserDriverControl{

public:
    // 获取单例实例（懒汉模式，线程安全）
    static std::shared_ptr<LaserDriverControl> getInstance();
    
    // 初始化单例（需要在第一次使用前调用）
    static void initialize(AGVBone agv_bone);
    
    // 删除拷贝构造和赋值操作
    LaserDriverControl(const LaserDriverControl&) = delete;
    LaserDriverControl& operator=(const LaserDriverControl&) = delete;

    /*****************************************************************************************
    * @brief:      AGV驱动的实现函数
    * @param:      无
    * @return:     驱动成功返回true否则返回false
    * @author:     刘鸿彬
    * @date:       2024-11-06
    ******************************************************************************************/
    bool control();
    /*****************************************************************************************
    * @brief:      获取外界的数据，将外界数据传输给类内变量，为驱动提供参数
    * @param:      goal_x：x轴坐标点容器
    * @param:      goal_y：y轴坐标点容器
    * @param:      goal_theta：旋转角度容器
    * @return:     无
    * @author:     刘鸿彬
    * @date:       2024-11-06
    ******************************************************************************************/
    void setPostion(std::vector<Point> points);
    void setForward(bool forward);
    // 取消任务
    bool cancel();

    bool get_flag_finish();

    bool get_flag_aborted();

    bool get_flag_driving();

    void set_flag_finish(bool flag);

    // 声明一个
    std::shared_ptr<LaserSendMultiPose> send_multi_pose_;

private:
    // 私有构造函数
    LaserDriverControl(AGVBone agv_bone);

    // 静态单例实例
    static std::shared_ptr<LaserDriverControl> instance_;
    // 静态互斥锁（用于线程安全）
    static std::mutex mutex_;

    std::vector<Point> goal_points;
    bool forward;
};


class QRDriverControl{

public:
    // 获取单例实例（懒汉模式，线程安全）
    static std::shared_ptr<QRDriverControl> getInstance();
    
    // 初始化单例（需要在第一次使用前调用）
    static void initialize(AGVBone agv_bone);
    
    // 删除拷贝构造和赋值操作
    QRDriverControl(const QRDriverControl&) = delete;
    QRDriverControl& operator=(const QRDriverControl&) = delete;

    /*****************************************************************************************
    * @brief:      AGV驱动的实现函数
    * @param:      无
    * @return:     驱动成功返回true否则返回false
    * @author:     刘鸿彬
    * @date:       2024-11-06
    ******************************************************************************************/
    bool control();
    /*****************************************************************************************
    * @brief:      获取外界的数据，将外界数据传输给类内变量，为驱动提供参数
    * @param:      goal_x：x轴坐标点容器
    * @param:      goal_y：y轴坐标点容器
    * @param:      goal_theta：旋转角度容器
    * @return:     无
    * @author:     刘鸿彬
    * @date:       2024-11-06
    ******************************************************************************************/
    void setPostion(std::vector<agv_interfaces::msg::Poses> goal_poses);

    // 取消任务
    bool cancel();

    bool get_flag_finish();

    bool get_flag_aborted();

    bool get_flag_driving();

    void set_flag_finish(bool flag);

private:
    // 私有构造函数
    QRDriverControl(AGVBone agv_bone);

    // 静态单例实例
    static std::shared_ptr<QRDriverControl> instance_;
    // 静态互斥锁（用于线程安全）
    static std::mutex mutex_;

    // 声明一个
    std::shared_ptr<QRSendMultiPose> send_multi_pose_;

    std::vector<agv_interfaces::msg::Poses> goal_poses_;
};


# endif