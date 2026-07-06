/************************************** File Info ****************************************
* @file:       control_agv_fork.h   
* @author:     刘鸿彬        
* @date:       2024-11-07   
* @version:    V0.0      
* @brief:      货叉控制接口实现头文件
******************************************************************************************/

# ifndef CONTROL_AGV_FORK_H
# define CONTROL_AGV_FORK_H

# include "client_fork_action.h"
# include "agv_bone.h"

class LaserForkControl
{

public:

    LaserForkControl(AGVBone agv_bone);

    /*****************************************************************************************
    * @brief:      AGV货叉控制的实现函数
    * @param:      无
    * @return:     驱动成功返回true否则返回false
    * @author:     刘鸿彬
    * @date:       2024-11-06
    ******************************************************************************************/
    bool control();
    /*****************************************************************************************
    * @brief:      获取外界的数据，将外界数据传输给类内变量，为驱动提供参数
    * @param:      forkEnable：货叉控制使能位（true：开启/false：关闭）
    * @param:      forkHeight：货叉目标高度
    * @return:     无
    * @author:     刘鸿彬
    * @date:       2024-11-06
    ******************************************************************************************/
    void setForkParameters(bool forkEnable,int forkHeight);

    bool get_flag_finish();

    void set_flag_finish(bool flag);

    /*****************************************************************************************
    * @brief:      获取任务是否被中断的标志
    * @param:      无
    * @return:     如果任务被中断返回true，否则返回false
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    bool get_flag_aborted();

    /*****************************************************************************************
    * @brief:      获取任务是否正在执行的标志
    * @param:      无
    * @return:     如果任务正在执行返回true，否则返回false
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    bool get_flag_driving();

    /*****************************************************************************************
    * @brief:      获取当前货叉高度
    * @param:      无
    * @return:     当前货叉高度
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    int get_fork_height() const;

    /*****************************************************************************************
    * @brief:      取消正在执行的货叉动作
    * @param:      无
    * @return:     取消成功返回true，否则返回false
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    bool cancel();

    std::shared_ptr<LsaerForkActionClient> forkActionClient;
    
private:

    int current_fork_height_;
    
    bool forkEnable;//货叉控制使能位（true：开启/false：关闭）
    int forkHeight;//货叉目标高度
};

// -----------------------------------------------------------------------------------------------

class QRForkControl
{

public:

    QRForkControl(AGVBone agv_bone);

    /*****************************************************************************************
    * @brief:      AGV货叉控制的实现函数
    * @param:      无
    * @return:     驱动成功返回true否则返回false
    * @author:     刘鸿彬
    * @date:       2024-11-06
    ******************************************************************************************/
    bool control();
    /*****************************************************************************************
    * @brief:      获取外界的数据，将外界数据传输给类内变量，为驱动提供参数
    * @param:      forkEnable：货叉控制使能位（true：开启/false：关闭）
    * @param:      forkHeight：货叉目标高度
    * @return:     无
    * @author:     刘鸿彬
    * @date:       2024-11-06
    ******************************************************************************************/
    void setForkParameters(bool forkEnable,int forkHeight, int cargoRotation);

    bool get_flag_finish();

    void set_flag_finish(bool flag);

    /*****************************************************************************************
    * @brief:      获取任务是否被中断的标志
    * @param:      无
    * @return:     如果任务被中断返回true，否则返回false
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    bool get_flag_aborted();

    /*****************************************************************************************
    * @brief:      获取任务是否正在执行的标志
    * @param:      无
    * @return:     如果任务正在执行返回true，否则返回false
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    bool get_flag_driving();

    /*****************************************************************************************
    * @brief:      获取当前货叉高度
    * @param:      无
    * @return:     当前货叉高度
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    int get_fork_height() const;

    /*****************************************************************************************
    * @brief:      取消正在执行的货叉动作
    * @param:      无
    * @return:     取消成功返回true，否则返回false
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    bool cancel();

    std::shared_ptr<QRForkActionClient> forkActionClient;
    
private:

    int current_fork_height_;
    
    bool forkEnable;//货叉控制使能位（true：开启/false：关闭）
    int forkHeight;//货叉目标高度
    int cargoRotation;
};

# endif