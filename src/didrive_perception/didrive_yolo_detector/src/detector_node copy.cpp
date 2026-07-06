#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>

// 【新增】：引入 Tier4 感知特有消息头文件
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
// 引入 ROS2 图像标准传输库
#include <image_transport/image_transport.hpp>

#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>

// TensorRT 10.x 标准日志接收器
class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cout << "[TensorRT] " << msg << std::endl;
        }
    }
} gLogger;

struct YOLO26Result {
    int class_id;
    float confidence;
    cv::Rect2f box; // [x, y, w, h] 对应原图分辨率
};

class YOLO26DetectorNode : public rclcpp::Node {
public:
    YOLO26DetectorNode() : Node("yolo26_detector_node") {
        // ---- 1. 参数声明与读取 ----
        this->declare_parameter<std::string>("model_path", "yolo26n.engine");
        this->declare_parameter<std::string>("input_topic", "/sensing/camera/camera0/image_rect_color");
        this->declare_parameter<std::string>("output_topic", "/perception/object_recognition/detection/rois");
        this->declare_parameter<double>("score_threshold", 0.25);

        model_path_ = this->get_parameter("model_path").as_string();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        score_threshold_ = this->get_parameter("score_threshold").as_double();

        // ---- 2. 初始化 TensorRT 引擎 ----
        if (!initTensorRT()) {
            RCLCPP_ERROR(this->get_logger(), "TensorRT Engine 初始化失败，请检查路径或 trtexec 编译是否成功！");
            return;
        }

        // ---- 3. ROS2 订阅与发布 ----
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, rclcpp::QoS{10}.best_effort(),
            std::bind(&YOLO26DetectorNode::imageCallback, this, std::placeholders::_1));

        objects_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
            output_topic, rclcpp::QoS{1});

        // 【新增功能】：初始化 Debug 图像发布者
        // 发布的 debug 话题为：/perception/object_recognition/detection/debug_image
        image_pub_ = image_transport::create_publisher(this, "/perception/object_recognition/detection/debug_image", rmw_qos_profile_default);

        // 【新增】：初始化 rois0 话题发布者
        rois0_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
            "/perception/object_recognition/detection/rois0", rclcpp::QoS{1});

        RCLCPP_INFO(this->get_logger(), "YOLO26 高精度检测节点成功启动（Debug 图像发布功能已同步激活）。");
    }

    ~YOLO26DetectorNode() {
        if (device_buffers_[0]) cudaFree(device_buffers_[0]);
        if (device_buffers_[1]) cudaFree(device_buffers_[1]);
    }

private:
    bool initTensorRT() {
        std::ifstream file(model_path_, std::ios::binary);
        if (!file.good()) {
            RCLCPP_ERROR(this->get_logger(), "模型文件不存在: %s", model_path_.c_str());
            return false;
        }

        file.seekg(0, std::ios::end);
        size_t size = file.tellg();
        file.seekg(0, std::ios::beg);
        std::vector<char> engine_data(size);
        file.read(engine_data.data(), size);
        file.close();

        // 创建推理运行时
        runtime_ = std::unique_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(gLogger));
        if (!runtime_) {
            RCLCPP_ERROR(this->get_logger(), "无法创建 TensorRT Runtime！");
            return false;
        }

        // TensorRT 10 标准安全释放智能指针写法
        engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
            runtime_->deserializeCudaEngine(engine_data.data(), size),
            std::default_delete<nvinfer1::ICudaEngine>()
        );
        
        if (!engine_) {
            RCLCPP_ERROR(this->get_logger(), "TensorRT Engine 反序列化失败！文件可能损坏或版本不匹配。");
            return false;
        }

        // 创建执行上下文
        context_ = std::unique_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
        if (!context_) {
            RCLCPP_ERROR(this->get_logger(), "无法创建 TensorRT Execution Context！");
            return false;
        }

        // 申请 Orin 显存 (输入: 1x3x640x640 FP32, 输出: 1x300x6 FP32)
        cudaMalloc(&device_buffers_[0], 1 * 3 * 640 * 640 * sizeof(float));
        cudaMalloc(&device_buffers_[1], 1 * 300 * 6 * sizeof(float));

        output_host_buffer_.resize(1 * 300 * 6);
        return true;
    }

    // 核心前处理：带 Letterbox 垫黑边的自适应图像缩放
    cv::Mat preprocessLetterbox(const cv::Mat& src, cv::Size target_size, float& scale, int& dw, int& dh) {
        int w = src.cols;
        int h = src.rows;
        
        scale = std::min((float)target_size.width / w, (float)target_size.height / h);
        int new_w = std::round(w * scale);
        int new_h = std::round(h * scale);
        
        dw = (target_size.width - new_w) / 2;
        dh = (target_size.height - new_h) / 2;

        cv::Mat resized_img;
        if (w != new_w || h != new_h) {
            cv::resize(src, resized_img, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
        } else {
            resized_img = src;
        }

        cv::Mat dst;
        cv::copyMakeBorder(resized_img, dst, dh, target_size.height - new_h - dh, 
                           dw, target_size.width - new_w - dw, 
                           cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
        return dst;
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常转换失败: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        
        // ---- 1. 前处理 ----
        float scale;
        int dw, dh;
        cv::Mat letterbox_img = preprocessLetterbox(frame, cv::Size(640, 640), scale, dw, dh);

        cv::Mat blob;
        cv::dnn::blobFromImage(letterbox_img, blob, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false);

        // ---- 2. 拷贝进 GPU ----
        cudaMemcpy(device_buffers_[0], blob.ptr<float>(), 1 * 3 * 640 * 640 * sizeof(float), cudaMemcpyHostToDevice);

        // ---- 3. TensorRT 推理 ----
        context_->setInputTensorAddress("images", device_buffers_[0]);
        context_->setOutputTensorAddress("output0", device_buffers_[1]);
        context_->enqueueV3(0);

        // ---- 4. 结果考回 CPU ----
        cudaMemcpy(output_host_buffer_.data(), device_buffers_[1], 1 * 300 * 6 * sizeof(float), cudaMemcpyDeviceToHost);

        // ---- 5. 后处理还原比例 ----
        std::vector<YOLO26Result> detections;
        for (int i = 0; i < 300; ++i) {
            int index = i * 6;
            float x_min = output_host_buffer_[index + 0];
            float y_min = output_host_buffer_[index + 1];
            float x_max = output_host_buffer_[index + 2];
            float y_max = output_host_buffer_[index + 3];
            float score = output_host_buffer_[index + 4];
            int class_id = static_cast<int>(output_host_buffer_[index + 5]);

            if (score < score_threshold_) continue;

            if (x_max <= 1.01f && y_max <= 1.01f) {
                x_min *= 640.0f;
                y_min *= 640.0f;
                x_max *= 640.0f;
                y_max *= 640.0f;
            }

            float x = (x_min - dw) / scale;
            float y = (y_min - dh) / scale;
            float w = (x_max - x_min) / scale;
            float h = (y_max - y_min) / scale;

            x = std::max(0.0f, x);
            y = std::max(0.0f, y);
            w = std::min(w, (float)frame.cols - x);
            h = std::min(h, (float)frame.rows - y);

            YOLO26Result det;
            det.class_id = class_id;
            det.confidence = score;
            det.box = cv::Rect2f(x, y, w, h);
            detections.push_back(det);
        }

        // 绘制并发布 Debug 图像
        cv::Mat debug_frame = frame.clone(); // 克隆原图进行绘制
        for (const auto& det : detections) {
            // 绘制目标边框 (鲜绿色，线宽 2)
            cv::rectangle(debug_frame, det.box, cv::Scalar(0, 255, 0), 2);
            
            // 生成标签文字
            std::string label_text = "ID:" + std::to_string(det.class_id) + " P:" + std::to_string(det.confidence).substr(0, 4);
            
            int baseLine;
            cv::Size label_size = cv::getTextSize(label_text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            cv::Point text_org(det.box.x, det.box.y - 5 >= 0 ? det.box.y - 5 : det.box.y + 15);

            // 绘制标签背景遮罩，使文字在亮色背景下依然清晰可见
            cv::rectangle(debug_frame, cv::Point(text_org.x, text_org.y - label_size.height),
                          cv::Point(text_org.x + label_size.width, text_org.y + baseLine), cv::Scalar(0, 255, 0), cv::FILLED);
            
            // 写入黑字标签
            cv::putText(debug_frame, label_text, text_org, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        }

        // 将 cv::Mat 转化为 ROS2 图像消息并发布
        sensor_msgs::msg::Image::SharedPtr debug_img_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_frame).toImageMsg();
        image_pub_.publish(debug_img_msg);
        // ==============================================================================

        // ---- 6. 封装并分发 Autoware 感知消息包 ----
        autoware_perception_msgs::msg::DetectedObjects output_msg;
        output_msg.header = msg->header;

        // 【新增】：初始化并赋值 tier4 特征消息包的 header
        tier4_perception_msgs::msg::DetectedObjectsWithFeature output_rois0_msg;
        output_rois0_msg.header = msg->header;
        output_rois0_msg.header.frame_id = "camera";

        for (const auto& det : detections) {
            autoware_perception_msgs::msg::DetectedObject obj;
            obj.existence_probability = det.confidence;

            autoware_perception_msgs::msg::ObjectClassification classification;
            classification.label = convertClassIdToAutowareLabel(det.class_id);
            classification.probability = det.confidence;
            obj.classification.push_back(classification);

            // 1. 运动学参数清零初始化（完全对齐标准输出的 0.0 和 w: 1.0）
            obj.kinematics.pose_with_covariance.pose.position.x = 0.0;
            obj.kinematics.pose_with_covariance.pose.position.y = 0.0;
            obj.kinematics.pose_with_covariance.pose.position.z = 0.0;
            obj.kinematics.pose_with_covariance.pose.orientation.x = 0.0;
            obj.kinematics.pose_with_covariance.pose.orientation.y = 0.0;
            obj.kinematics.pose_with_covariance.pose.orientation.z = 0.0;
            obj.kinematics.pose_with_covariance.pose.orientation.w = 1.0;
            obj.kinematics.has_position_covariance = false;
            obj.kinematics.orientation_availability = 0;
            obj.kinematics.has_twist = false;
            obj.kinematics.has_twist_covariance = false;
            
            // 2. 形状参数对齐（标准输出中 type 为 0，且 dimensions 全为 0）
            obj.shape.type = 0; 
            obj.shape.dimensions.x = 0.0;
            obj.shape.dimensions.y = 0.0;
            obj.shape.dimensions.z = 0.0;

            output_msg.objects.push_back(obj);

            // ==================== 🔥 【核心新增：构造并填入 rois0 消息数据】 ====================
            tier4_perception_msgs::msg::DetectedObjectWithFeature obj_with_feature;
            
            // 1. 填入标准的 DetectedObject
            obj_with_feature.object = obj;

            // 2. 填入特征数据 Feature 中的 2D ROI 区域 (在图像上的像素级 Bounding Box)
            obj_with_feature.feature.roi.x_offset = static_cast<uint32_t>(det.box.x);
            obj_with_feature.feature.roi.y_offset = static_cast<uint32_t>(det.box.y);
            obj_with_feature.feature.roi.width    = static_cast<uint32_t>(det.box.width);
            obj_with_feature.feature.roi.height   = static_cast<uint32_t>(det.box.height);
            obj_with_feature.feature.roi.do_rectify = false; // 通常相机内部已去畸变

            // 4. 点云聚类 cluster 保持空字段（与你的输出完全一致）
            obj_with_feature.feature.cluster.header.frame_id = "";
            obj_with_feature.feature.cluster.data.clear();

            // 3. 将单条目标追加至消息列表中
            output_rois0_msg.feature_objects.push_back(obj_with_feature);
            // ==================================================================================
        }

        objects_pub_->publish(output_msg);
        rois0_pub_->publish(output_rois0_msg);
    }

    uint8_t convertClassIdToAutowareLabel(int yolo_id) {
        switch (yolo_id) {
            case 0: return autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
            case 1: return autoware_perception_msgs::msg::ObjectClassification::BICYCLE;
            case 2: return autoware_perception_msgs::msg::ObjectClassification::CAR;
            case 3: return autoware_perception_msgs::msg::ObjectClassification::TRUCK;
            case 5: return autoware_perception_msgs::msg::ObjectClassification::BUS;
            default: return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
        }
    }

    std::string model_path_;
    double score_threshold_;
    
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::shared_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;

    void* device_buffers_[2] = {nullptr, nullptr}; 
    std::vector<float> output_host_buffer_; 

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
    
    // 【新增句柄】图像标准传输发布者
    image_transport::CameraPublisher debug_camera_pub_;
    image_transport::Publisher image_pub_;
    // 【新增】：rois0 话题发布者句柄声明
    rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr rois0_pub_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YOLO26DetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


