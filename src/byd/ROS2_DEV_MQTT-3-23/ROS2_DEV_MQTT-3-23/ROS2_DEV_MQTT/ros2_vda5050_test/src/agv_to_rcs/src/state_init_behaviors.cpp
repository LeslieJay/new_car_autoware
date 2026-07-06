/************************************** File Info ****************************************
* @file:       state_init_behaviors.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-11                                       
* @version:    V0.0                                                                              
* @brief:      agv初始化状态，待完善
******************************************************************************************/
// state_init_behaviors.cpp
# include "state_init_behaviors.h"

InitStateBehaviors::InitStateBehaviors(const std::string& name, const NodeConfig& config):
    SyncActionNode(name, config) {}


NodeStatus InitStateBehaviors::tick()
{
    std::cout << this->name() << " 正在执行:OnInitTry() " << std::endl;
    OnInitTry();
    return NodeStatus::SUCCESS;
}


/*****************************************************************************************
* @brief:      用于初始化操作，进行上次任务检查以及各模块功能检查
* @param:      无
* @return:     返回当前状态处理之后的事件
* @author:     刘鸿彬
* @date:       2024-11-09
* @note：      读取运行日志，查找上次任务完成情况，未完成上次任务，报告给RCS等待处理结果
* @note：      1、读取相机模块，相机模块反馈状态，2、读取雷达模块，雷达模块返回状态，3、读取cvc模块，cvc模块返回状态
* @version:    V0.0
******************************************************************************************/
void InitStateBehaviors::OnInitTry(){
    // 进行上次任务检查，现在全部为true
    check_last_task_ = check_last_task();
    // 进行模块上线检查
    check_module_ = check_module();

    // 初始化完成，进行状态跳转，现在全部为true
    if(check_last_task_ && check_module_){
        std::cout << "AGV状态初始化成功，即将跳转至空闲状态！" << std::endl;
        // 任务完成，跳转至空闲状态
        this->config().blackboard->set("last_state", "init");
        this->config().blackboard->set("current_state", "idle");
        this->config().blackboard->set("AGV_Event", "init_success");
    }
    else{
        std::cout << "AGV状态初始化失败，即将跳转至异常状态！" << std::endl;
        // 初始化失败
        this->config().blackboard->set("last_state", "init");
        this->config().blackboard->set("current_state", "lock");
        this->config().blackboard->set("AGV_Event", "init_failed");
        this->config().blackboard->set("fault_code", "INIT_FAILED");
    }
    
}

/*****************************************************************************************
* @brief:      检查相机、雷达、cvc是否能够正常连接
* @param:      
* @return:     连接正常返回true，否则false
* @author:     刘鸿彬
* @date:       2024-11-09
* @version:    V0.0
******************************************************************************************/
bool InitStateBehaviors::check_module(){

    // 相机、雷达、plc是否能够正常连接,连接正常返回true，否则false
    return true;
}

/*****************************************************************************************
* @brief:      检查上次任务是否完成
* @param:      无
* @return:     完成返回true，否则false
* @author:     刘鸿彬
* @date:       2024-11-09
* @version:    V0.0
* @note:       1、获取最新修改的日志文件名称，2、打开该文件，读取最后一条数据，3、通过该数据判断上次任务完成情况
******************************************************************************************/
bool InitStateBehaviors::check_last_task(){
    
    // 检查上次任务是否完成
    return true;
}
