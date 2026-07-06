#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/msg/u_int8.hpp>
#include <vector>
#include <string>
#include <algorithm>

// 该版本为梯形区域版本，但是行人检测框与区域的相交判定方式由单点（脚底中心）判断改为四角+中心五点综合判断，提升了对大尺寸行人检测框的覆盖率和安全性
//“检测框 5 关键点触线判定法”（左上、右上、左下、右下、中心点）
class RearPedestrianWarningNode : public rclcpp::Node {
public:
    RearPedestrianWarningNode() : Node("rear_pedestrian_warning_node") {
        // 1. 初始化 ROS2 参数并设置默认值
        // 这里的默认值适用于宽 810、高 1080 的图像，采用“远窄近宽”的透视梯形
        // 坐标顺序：[左下, 左上, 右上, 右下] 的顺时针/逆时针闭合多边形
        this->declare_parameter<std::vector<long>>("stop_zone_coords", {50, 1080, 200, 900, 610, 900, 760, 1080});
        this->declare_parameter<std::vector<long>>("caution_zone_coords", {50, 1080, 280, 700, 530, 700, 760, 1080});
        this->declare_parameter<std::vector<long>>("warning_zone_coords", {50, 1080, 330, 550, 480, 550, 760, 1080});

        // 解析并加载多边形区域
        loadPolygonZones();

        // 2. 初始化发布者
        pub_warning_level_ = this->create_publisher<std_msgs::msg::UInt8>("/control/rear_warning_level", 10);
        pub_debug_image_ = this->create_publisher<sensor_msgs::msg::Image>("/perception/rear_pedestrian_warning/debug_image", 10);

        // 3. 配置带有时间同步的订阅者（采用近似时间同步策略策略 ApproximateTime）
        sub_image_.subscribe(this, "/cam1/image_rect"); 
        sub_objects_.subscribe(this, "/perception/object_recognition/detection/rois0"); 

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(30), sub_image_, sub_objects_);
        sync_->registerCallback(std::bind(&RearPedestrianWarningNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "【安全警报】倒车全框判定行人预警节点已成功启动！");
    }

private:
    // 预警状态枚举定义
    enum WarningLevel { SAFE = 0, WARNING = 1, CAUTION = 2, STOP = 3 };

    // 将一维 Param 数组解析为 OpenCV 的 cv::Point 数组
    void loadPolygonZones() {
        auto parse_coords = [this](const std::string& param_name, std::vector<cv::Point>& zone) {
            zone.clear();
            auto coords = this->get_parameter(param_name).as_integer_array();
            if (coords.size() % 2 != 0 || coords.empty()) {
                RCLCPP_ERROR(this->get_logger(), "参数 %s 长度非法，无法解析为坐标对！", param_name.c_str());
                return;
            }
            for (size_t i = 0; i < coords.size(); i += 2) {
                zone.push_back(cv::Point(static_cast<int>(coords[i]), static_cast<int>(coords[i+1])));
            }
        };
        parse_coords("stop_zone_coords", stop_zone_);
        parse_coords("caution_zone_coords", caution_zone_);
        parse_coords("warning_zone_coords", warning_zone_);
    }

    // 【核心辅助函数】：利用 5 个关键点严苛判定检测框（矩形）与安全多边形是否相交
    bool checkBoxInPolygon(const cv::Rect& bbox, const std::vector<cv::Point>& polygon) {
        if (polygon.empty()) return false;

        std::vector<cv::Point> test_points;
        test_points.push_back(cv::Point(bbox.x, bbox.y));                                 // 1. 左上角点
        test_points.push_back(cv::Point(bbox.x + bbox.width, bbox.y));                    // 2. 右上角点
        test_points.push_back(cv::Point(bbox.x, bbox.y + bbox.height));                   // 3. 左下角点
        test_points.push_back(cv::Point(bbox.x + bbox.width, bbox.y + bbox.height));           // 4. 右下角点
        test_points.push_back(cv::Point(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2));   // 5. 几何中心点

        // 只要 5 个关键点中任意一个点落在多边形内部或边缘（pointPolygonTest 返回值 >= 0），即触发警报
        for (const auto& pt : test_points) {
            if (cv::pointPolygonTest(polygon, pt, false) >= 0) {
                return true; 
            }
        }
        return false;
    }

    // 时间同步回调函数
    void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
                      const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr& obj_msg) {
        
        // 将 ROS 图像消息转换为 OpenCV 矩阵，并显式指定图像格式为 BGR8
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
            return;
        }

        cv::Mat debug_img = cv_ptr->image;
        WarningLevel highest_level = SAFE;

        // --- 第一步：半透明叠加渲染三个透视梯形区域 ---
        cv::Mat overlay = debug_img.clone();
        std::vector<std::vector<cv::Point>> fill_pts;
        
        if (!warning_zone_.empty()) { fill_pts = {warning_zone_}; cv::fillPoly(overlay, fill_pts, cv::Scalar(0, 255, 255)); }   // 黄色预警区
        if (!caution_zone_.empty()) { fill_pts = {caution_zone_}; cv::fillPoly(overlay, fill_pts, cv::Scalar(0, 165, 255)); }   // 橙色减速区
        if (!stop_zone_.empty())    { fill_pts = {stop_zone_};    cv::fillPoly(overlay, fill_pts, cv::Scalar(0, 0, 255)); }     // 红色绝对停车区
        
        // 图层融合（不遮挡画面背景物）
        float alpha = 0.25;
        cv::addWeighted(overlay, alpha, debug_img, 1.0 - alpha, 0, debug_img);

        // 绘制高亮多边形边界轮廓线
        if (!warning_zone_.empty()) cv::polylines(debug_img, warning_zone_, true, cv::Scalar(0, 255, 255), 2);
        if (!caution_zone_.empty()) cv::polylines(debug_img, caution_zone_, true, cv::Scalar(0, 165, 255), 2);
        if (!stop_zone_.empty())    cv::polylines(debug_img, stop_zone_, true, cv::Scalar(0, 0, 255), 2);

        // --- 第二步：遍历并过滤 Autoware 检测到的 2D 目标 ---
        // for (const auto& feature_obj : obj_msg->feature_objects) {
        //     const auto& obj = feature_obj.object;
            
        //     // 过滤行人标签 (PEDESTRIAN)
        //     if (obj.classification.empty() || 
        //         obj.classification[0].label != autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN) {
        //         continue;
        //     }

        //     // 获取图像中的 2D Bounding Box 并转化为 OpenCV 矩形类
        //     const auto& bbox_msg = feature_obj.feature.roi;
        //     cv::Rect pedestrian_box(bbox_msg.x_offset, bbox_msg.y_offset, bbox_msg.width, bbox_msg.height);

        //     // 核心级改进：利用多点包络相交检测替代原有的“脚底中心单点”检测
        //     WarningLevel current_obj_level = SAFE;
        //     if (checkBoxInPolygon(pedestrian_box, stop_zone_)) {
        //         current_obj_level = STOP;
        //     } else if (checkBoxInPolygon(pedestrian_box, caution_zone_)) {
        //         current_obj_level = CAUTION;
        //     } else if (checkBoxInPolygon(pedestrian_box, warning_zone_)) {
        //         current_obj_level = WARNING;
        //     }

        //     // 维持全域的最高预警等级
        //     highest_level = std::max(highest_level, current_obj_level);

        //     // --- 第三步：针对不同等级为当前行人绘制彩色检测框与状态说明 ---
        //     cv::Scalar box_color = cv::Scalar(0, 255, 0); // 默认安全（绿色）
        //     std::string label_text = "Pedestrian";
            
        //     if (current_obj_level == WARNING)      { box_color = cv::Scalar(0, 255, 255); label_text = "WARN!"; }
        //     else if (current_obj_level == CAUTION)  { box_color = cv::Scalar(0, 165, 255); label_text = "CAUTION!"; }
        //     else if (current_obj_level == STOP)     { box_color = cv::Scalar(0, 0, 255);   label_text = "!!!STOP!!!"; }

        //     // 绘制行人矩形框以及标签文本
        //     cv::rectangle(debug_img, pedestrian_box, box_color, 2);
        //     cv::putText(debug_img, label_text, cv::Point(pedestrian_box.x, pedestrian_box.y - 10), 
        //                 cv::FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2);
        // }
        // --- 第二步：遍历 Autoware 检测到的 2D 目标（不再仅限行人，面向全目标检测） ---
        for (const auto& feature_obj : obj_msg->feature_objects) {
            const auto& obj = feature_obj.object;
            
            // 【修改点 1】：安全防空指针检查。若没有分类数据，默认为未知物体，不再直接 continue 丢弃
            uint8_t current_label = 0; // 0 在 Autoware 中代表 UNKNOWN 或 PEDESTRIAN，视具体版本而定
            if (!obj.classification.empty()) {
                current_label = obj.classification[0].label;
            }

            // 获取图像中的 2D Bounding Box 并转化为 OpenCV 矩形类
            const auto& bbox_msg = feature_obj.feature.roi;
            cv::Rect object_box(bbox_msg.x_offset, bbox_msg.y_offset, bbox_msg.width, bbox_msg.height);

            // 核心级改进：利用多点包络相交检测判定当前物体处于哪一个警报区域
            WarningLevel current_obj_level = SAFE;
            if (checkBoxInPolygon(object_box, stop_zone_)) {
                current_obj_level = STOP;
            } else if (checkBoxInPolygon(object_box, caution_zone_)) {
                current_obj_level = CAUTION;
            } else if (checkBoxInPolygon(object_box, warning_zone_)) {
                current_obj_level = WARNING;
            }

            // 维持全域的最高预警等级
            highest_level = std::max(highest_level, current_obj_level);

            // --- 第三步：动态转换标签名称，针对不同等级绘制彩色检测框 ---
            cv::Scalar box_color = cv::Scalar(0, 255, 0); // 默认安全（绿色）
            
            // 【修改点 2】：根据 YOLO26 转换出的标准 Autoware 类别数字，映射为可视化文字
            std::string class_name = "Unknown";
            switch (current_label) {
                case 0: class_name = "Pedestrian"; break; // PEDESTRIAN 底层常数通常为 0
                case 1: class_name = "Bicycle";    break; // BICYCLE
                case 2: class_name = "Car";        break; // CAR
                case 3: class_name = "Truck";      break; // TRUCK
                case 5: class_name = "Bus";        break; // BUS
                default: class_name = "Obstacle";  break;
            }

            // 根据危险程度追加状态文字，并改变边框颜色
            std::string label_text = class_name;
            if (current_obj_level == WARNING)      { box_color = cv::Scalar(0, 255, 255); label_text += " [WARN]"; }
            else if (current_obj_level == CAUTION)  { box_color = cv::Scalar(0, 165, 255); label_text += " [CAUTION]"; }
            else if (current_obj_level == STOP)     { box_color = cv::Scalar(0, 0, 255);   label_text += " [!!!STOP!!!]"; }

            // 绘制物体矩形框以及状态文本
            cv::rectangle(debug_img, object_box, box_color, 2);
            cv::putText(debug_img, label_text, cv::Point(object_box.x, object_box.y - 10), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, box_color, 2);
        }

        // --- 第四步：对外发布全局警告数据与 Debug 图像 ---
        std_msgs::msg::UInt8 warning_msg;
        warning_msg.data = highest_level;
        pub_warning_level_->publish(warning_msg);

        // 画面左上角渲染系统的全局总控状态
        std::string status_text = "System Status: SAFE";
        cv::Scalar status_color = cv::Scalar(0, 255, 0);
        if (highest_level == WARNING)      { status_text = "STATUS: WARNING"; status_color = cv::Scalar(0, 255, 255); }
        else if (highest_level == CAUTION)  { status_text = "STATUS: CAUTION"; status_color = cv::Scalar(0, 165, 255); }
        else if (highest_level == STOP)     { status_text = "STATUS: EMERGENCY STOP"; status_color = cv::Scalar(0, 0, 255); }
        
        cv::putText(debug_img, status_text, cv::Point(30, 60), cv::FONT_HERSHEY_SIMPLEX, 0.9, status_color, 3);

        // 将渲染好的图片通过自定义 Debug 话题发送出去，方便在 Rviz 中监测
        pub_debug_image_->publish(*cv_ptr->toImageMsg());
    }

    // 基础底层会话管理组件
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_image_;
    message_filters::Subscriber<tier4_perception_msgs::msg::DetectedObjectsWithFeature> sub_objects_;
    
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, 
        tier4_perception_msgs::msg::DetectedObjectsWithFeature
    > SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_warning_level_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_image_;

    // 多边形顶点数组
    std::vector<cv::Point> stop_zone_;
    std::vector<cv::Point> caution_zone_;
    std::vector<cv::Point> warning_zone_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RearPedestrianWarningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}