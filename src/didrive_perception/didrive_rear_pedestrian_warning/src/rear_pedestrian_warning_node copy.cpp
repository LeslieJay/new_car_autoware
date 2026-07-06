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

// 该版本为梯形区域版本，但是行人检测框与区域的相交判定方式为单点（脚底中心）判断
class RearPedestrianWarningNode : public rclcpp::Node {
public:
    RearPedestrianWarningNode() : Node("rear_pedestrian_warning_node") {
        // 1. 初始化 ROS2 参数 (可通过 launch 文件或 yaml 配置非对称梯形的顶点坐标)
        // 坐标顺序：顺时针或逆时针闭合多边形
        // 坐标顺序：[左下, 左上, 右上, 右下] 的顺时针/逆时针闭合多边形
        this->declare_parameter<std::vector<long>>("stop_zone_coords",{50, 1080, 200, 900, 610, 900, 760, 1080});
        this->declare_parameter<std::vector<long>>("caution_zone_coords", {50, 1080, 280, 700, 530, 700, 760, 1080});
        this->declare_parameter<std::vector<long>>("warning_zone_coords", {50, 1080, 330, 550, 480, 550, 760, 1080});

        loadPolygonZones();

        // 2. 状态发布者
        pub_warning_level_ = this->create_publisher<std_msgs::msg::UInt8>("/control/rear_warning_level", 10);
        pub_debug_image_ = this->create_publisher<sensor_msgs::msg::Image>("/perception/rear_pedestrian_warning/debug_image", 10);

        // 3. 设置带有时间同步的订阅者 (使用近似时间同步策略)
        sub_image_.subscribe(this, "/sensing/camera/camera0/image_rect_color"); // 请根据实际相机话题修改
        sub_objects_.subscribe(this, "/perception/object_recognition/detection/rois0"); // Autoware YOLOX 默认输出

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), sub_image_, sub_objects_);
        sync_->registerCallback(std::bind(&RearPedestrianWarningNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "倒车行人检测预警节点已成功启动！");
    }

private:
    enum WarningLevel { SAFE = 0, WARNING = 1, CAUTION = 2, STOP = 3 };

    // 将一维 Param 数组解析为 OpenCV 的多边形点集
    void loadPolygonZones() {
        auto parse_coords = [this](const std::string& param_name, std::vector<cv::Point>& zone) {
            auto coords = this->get_parameter(param_name).as_integer_array();
            for (size_t i = 0; i < coords.size(); i += 2) {
                zone.push_back(cv::Point(coords[i], coords[i+1]));
            }
        };
        parse_coords("stop_zone_coords", stop_zone_);
        parse_coords("caution_zone_coords", caution_zone_);
        parse_coords("warning_zone_coords", warning_zone_);
    }

    void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
                      const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr& obj_msg) {
        
        // 转换 ROS 图像为 OpenCV 格式用于绘制 Debug 图
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
            return;
        }

        cv::Mat debug_img = cv_ptr->image;
        WarningLevel highest_level = SAFE;

        // --- 第一步：绘制三个非对称梯形区域（半透明填充） ---
        cv::Mat overlay = debug_img.clone();
        std::vector<std::vector<cv::Point>> fill_pts;
        
        // 预警区：黄色 (0, 255, 255)
        fill_pts = {warning_zone_};
        cv::fillPoly(overlay, fill_pts, cv::Scalar(0, 255, 255));
        // 减速区：橙色 (0, 165, 255)
        fill_pts = {caution_zone_};
        cv::fillPoly(overlay, fill_pts, cv::Scalar(0, 165, 255));
        // 停车区：红色 (0, 0, 255)
        fill_pts = {stop_zone_};
        cv::fillPoly(overlay, fill_pts, cv::Scalar(0, 0, 255));
        
        // 叠加半透明效果 (不遮挡画面)
        float alpha = 0.3;
        cv::addWeighted(overlay, alpha, debug_img, 1.0 - alpha, 0, debug_img);

        // 绘制多边形边界轮廓线使其更清晰
        cv::polylines(debug_img, warning_zone_, true, cv::Scalar(0, 255, 255), 2);
        cv::polylines(debug_img, caution_zone_, true, cv::Scalar(0, 165, 255), 2);
        cv::polylines(debug_img, stop_zone_, true, cv::Scalar(0, 0, 255), 2);

        // --- 第二步：遍历 YOLOX 检测到的目标 ---
        for (const auto& feature_obj : obj_msg->feature_objects) {
            const auto& obj = feature_obj.object;
            
            // 过滤行人：Autoware.Universe 的分类定义在 Label 结构体中
            // 通常 1 是 PEDESTRIAN，如果你的分类映射不同，请查阅 autoware_perception_msgs
            if (obj.classification.empty() || 
                obj.classification[0].label != autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN) {
                continue;
            }

            // 获取图像中的 2D Bounding Box
            const auto& bbox = feature_obj.feature.roi; // 这里的 roi 包含 x_offset, y_offset, width, height
            
            int x_min = bbox.x_offset;
            int y_min = bbox.y_offset;
            int x_max = bbox.x_offset + bbox.width;
            int y_max = bbox.y_offset + bbox.height;

            // 计算行人脚部中心点 (Bottom-Center)
            cv::Point foot_point(bbox.x_offset + bbox.width / 2, y_max);

            // 核心算法：通过 pointPolygonTest 判定点在哪个多边形内
            // 返回值 >= 0 代表点在多边形内或边界上
            WarningLevel current_obj_level = SAFE;
            if (cv::pointPolygonTest(stop_zone_, foot_point, false) >= 0) {
                current_obj_level = STOP;
            } else if (cv::pointPolygonTest(caution_zone_, foot_point, false) >= 0) {
                current_obj_level = CAUTION;
            } else if (cv::pointPolygonTest(warning_zone_, foot_point, false) >= 0) {
                current_obj_level = WARNING;
            }

            highest_level = std::max(highest_level, current_obj_level);

            // --- 第三步：在 Debug 图像上绘制检测框与状态 ---
            cv::Scalar box_color = cv::Scalar(0, 255, 0); // 默认绿色（安全）
            std::string label_text = "Pedestrian: SAFE";
            
            if (current_obj_level == WARNING) { box_color = cv::Scalar(0, 255, 255); label_text = "WARN!"; }
            else if (current_obj_level == CAUTION) { box_color = cv::Scalar(0, 165, 255); label_text = "CAUTION!!"; }
            else if (current_obj_level == STOP) { box_color = cv::Scalar(0, 0, 255); label_text = "!!!STOP!!!"; }

            // 画检测框和脚部重心点
            cv::rectangle(debug_img, cv::Point(x_min, y_min), cv::Point(x_max, y_max), box_color, 2);
            cv::circle(debug_img, foot_point, 6, box_color, -1);
            cv::putText(debug_img, label_text, cv::Point(x_min, y_min - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2);
        }

        // --- 第四步：发布警告等级与 Debug 图像 ---
        std_msgs::msg::UInt8 warning_msg;
        warning_msg.data = highest_level;
        pub_warning_level_->publish(warning_msg);

        // 在左上角打印全局最高警告状态
        std::string status_text = "System Status: SAFE";
        cv::Scalar status_color = cv::Scalar(0, 255, 0);
        if (highest_level == WARNING) { status_text = "STATUS: WARNING"; status_color = cv::Scalar(0, 255, 255); }
        else if (highest_level == CAUTION) { status_text = "STATUS: CAUTION"; status_color = cv::Scalar(0, 165, 255); }
        else if (highest_level == STOP) { status_text = "STATUS: EMERGENCY STOP"; status_color = cv::Scalar(0, 0, 255); }
        cv::putText(debug_img, status_text, cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, status_color, 3);

        // 发布处理好的 Debug 图像话题，可以在 RViz2 中直接查看
        pub_debug_image_->publish(*cv_ptr->toImageMsg());
    }

    // 会话定义与成员变量
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