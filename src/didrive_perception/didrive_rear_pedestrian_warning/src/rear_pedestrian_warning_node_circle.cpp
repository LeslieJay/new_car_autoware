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
#include <cmath>

class RearPedestrianWarningNode : public rclcpp::Node {
public:
    RearPedestrianWarningNode() : Node("rear_pedestrian_warning_node") {
        // 1. 初始化参数：配置圆心、半径、以及扇形角度（角度制，水平向右为 0 度，顺时针增加）
        // 假设图像尺寸为 1280x720，车尾相机在图像底部的中点 (640, 720) 附近
        this->declare_parameter<int>("center_x", 405);
        this->declare_parameter<int>("center_y", 1080);
        
        // 角度范围：180度表示完全正后方，这里假设扫描车尾后方 45 度到 135 度的扇形面
        this->declare_parameter<double>("start_angle", 210.0); 
        this->declare_parameter<double>("end_angle", 330.0);

        // 设定三个预警区间的内外半径 (单位：像素级距离)
        this->declare_parameter<int>("stop_min_radius", 0);
        this->declare_parameter<int>("stop_max_radius", 150);

        this->declare_parameter<int>("caution_min_radius", 150);
        this->declare_parameter<int>("caution_max_radius", 350);

        this->declare_parameter<int>("warning_min_radius", 350);
        this->declare_parameter<int>("warning_max_radius", 600);

        // 生成扇形多边形
        generateSectorZones();

        // 2. 状态发布者与时间同步订阅（保持不变）
        pub_warning_level_ = this->create_publisher<std_msgs::msg::UInt8>("/control/rear_warning_level", 10);
        pub_debug_image_ = this->create_publisher<sensor_msgs::msg::Image>("/perception/rear_pedestrian_warning/debug_image", 10);

        sub_image_.subscribe(this, "/sensing/camera/camera0/image_rect_color"); 
        sub_objects_.subscribe(this, "/perception/object_recognition/detection/rois0"); 

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), sub_image_, sub_objects_);
        sync_->registerCallback(std::bind(&RearPedestrianWarningNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "倒车扇形预警区域检测节点已启动！");
    }

private:
    enum WarningLevel { SAFE = 0, WARNING = 1, CAUTION = 2, STOP = 3 };

    // 【核心改进】：利用三角函数动态生成扇形闭合多边形
    void generateSectorZones() {
        int cx = this->get_parameter("center_x").as_int();
        int cy = this->get_parameter("center_y").as_int();
        double start_ang = this->get_parameter("start_angle").as_double();
        double end_ang = this->get_parameter("end_angle").as_double();

        auto build_sector = [cx, cy, start_ang, end_ang](int min_r, int max_r, std::vector<cv::Point>& zone) {
            zone.clear();
            const double DEG2RAD = M_PI / 180.0;
            // 步长：每 4 度采样一个点，值越小弧线越平滑
            int step = 4; 

            // 1. 顺时针构建外圆弧上的点
            for (double a = start_ang; a <= end_ang; a += step) {
                int x = cx + static_cast<int>(max_r * std::cos(a * DEG2RAD));
                int y = cy + static_cast<int>(max_r * std::sin(a * DEG2RAD));
                zone.push_back(cv::Point(x, y));
            }
            // 确保终点精准闭合
            zone.push_back(cv::Point(cx + static_cast<int>(max_r * std::cos(end_ang * DEG2RAD)),
                                     cy + static_cast<int>(max_r * std::sin(end_ang * DEG2RAD))));

            // 2. 逆时针构建内圆弧上的点（如果内径为 0，则直接连接圆心）
            if (min_r <= 0) {
                zone.push_back(cv::Point(cx, cy));
            } else {
                for (double a = end_ang; a >= start_ang; a -= step) {
                    int x = cx + static_cast<int>(min_r * std::cos(a * DEG2RAD));
                    int y = cy + static_cast<int>(min_r * std::sin(a * DEG2RAD));
                    zone.push_back(cv::Point(x, y));
                }
                zone.push_back(cv::Point(cx + static_cast<int>(min_r * std::cos(start_ang * DEG2RAD)),
                                         cy + static_cast<int>(min_r * std::sin(start_ang * DEG2RAD))));
            }
        };

        build_sector(this->get_parameter("stop_min_radius").as_int(), this->get_parameter("stop_max_radius").as_int(), stop_zone_);
        build_sector(this->get_parameter("caution_min_radius").as_int(), this->get_parameter("caution_max_radius").as_int(), caution_zone_);
        build_sector(this->get_parameter("warning_min_radius").as_int(), this->get_parameter("warning_max_radius").as_int(), warning_zone_);
    }

    void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
                      const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr& obj_msg) {
        
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
            return;
        }

        cv::Mat debug_img = cv_ptr->image;
        WarningLevel highest_level = SAFE;

        // --- 绘制三个扇形区域 ---
        cv::Mat overlay = debug_img.clone();
        std::vector<std::vector<cv::Point>> fill_pts;
        
        fill_pts = {warning_zone_};
        cv::fillPoly(overlay, fill_pts, cv::Scalar(0, 255, 255)); // 黄
        fill_pts = {caution_zone_};
        cv::fillPoly(overlay, fill_pts, cv::Scalar(0, 165, 255)); // 橙
        fill_pts = {stop_zone_};
        cv::fillPoly(overlay, fill_pts, cv::Scalar(0, 0, 255));   // 红
        
        float alpha = 0.25;
        cv::addWeighted(overlay, alpha, debug_img, 1.0 - alpha, 0, debug_img);

        cv::polylines(debug_img, warning_zone_, true, cv::Scalar(0, 255, 255), 2);
        cv::polylines(debug_img, caution_zone_, true, cv::Scalar(0, 165, 255), 2);
        cv::polylines(debug_img, stop_zone_, true, cv::Scalar(0, 0, 255), 2);

        // --- 遍历并过滤行人 ---
        for (const auto& feature_obj : obj_msg->feature_objects) {
            const auto& obj = feature_obj.object;
            
            if (obj.classification.empty() || 
                obj.classification[0].label != autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN) {
                continue;
            }

            const auto& bbox = feature_obj.feature.roi;
            int x_min = bbox.x_offset;
            int y_min = bbox.y_offset;
            int x_max = bbox.x_offset + bbox.width;
            int y_max = bbox.y_offset + bbox.height;

            // 依然踩用行人接触地面的中点（脚部）进行扇形相交判定
            cv::Point foot_point(bbox.x_offset + bbox.width / 2, y_max);

            // 扇形本质上已被转化为多边形，这里无缝支持 pointPolygonTest
            WarningLevel current_obj_level = SAFE;
            if (cv::pointPolygonTest(stop_zone_, foot_point, false) >= 0) {
                current_obj_level = STOP;
            } else if (cv::pointPolygonTest(caution_zone_, foot_point, false) >= 0) {
                current_obj_level = CAUTION;
            } else if (cv::pointPolygonTest(warning_zone_, foot_point, false) >= 0) {
                current_obj_level = WARNING;
            }

            highest_level = std::max(highest_level, current_obj_level);

            // 绘制 Bounding Box 
            cv::Scalar box_color = cv::Scalar(0, 255, 0);
            std::string label_text = "Pedestrian";
            if (current_obj_level == WARNING) { box_color = cv::Scalar(0, 255, 255); label_text = "WARN!"; }
            else if (current_obj_level == CAUTION) { box_color = cv::Scalar(0, 165, 255); label_text = "CAUTION!"; }
            else if (current_obj_level == STOP) { box_color = cv::Scalar(0, 0, 255); label_text = "STOP!!!"; }

            cv::rectangle(debug_img, cv::Point(x_min, y_min), cv::Point(x_max, y_max), box_color, 2);
            cv::circle(debug_img, foot_point, 5, box_color, -1);
        }

        // --- 发布全局信号与调试图像 ---
        std_msgs::msg::UInt8 warning_msg;
        warning_msg.data = highest_level;
        pub_warning_level_->publish(warning_msg);

        // 顶层状态渲染
        std::string status_text = "System Status: SAFE";
        cv::Scalar status_color = cv::Scalar(0, 255, 0);
        if (highest_level == WARNING) { status_text = "STATUS: WARNING"; status_color = cv::Scalar(0, 255, 255); }
        else if (highest_level == CAUTION) { status_text = "STATUS: CAUTION"; status_color = cv::Scalar(0, 165, 255); }
        else if (highest_level == STOP) { status_text = "STATUS: EMERGENCY STOP"; status_color = cv::Scalar(0, 0, 255); }
        cv::putText(debug_img, status_text, cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, status_color, 3);

        pub_debug_image_->publish(*cv_ptr->toImageMsg());
    }

    message_filters::Subscriber<sensor_msgs::msg::Image> sub_image_;
    message_filters::Subscriber<tier4_perception_msgs::msg::DetectedObjectsWithFeature> sub_objects_;
    
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, 
        tier4_perception_msgs::msg::DetectedObjectsWithFeature
    > SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_warning_level_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_image_;

    // 存储采样出来的多边形边界
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