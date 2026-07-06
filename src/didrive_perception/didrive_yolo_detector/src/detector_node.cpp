#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <autoware_perception_msgs/msg/object_classification.hpp>

// 引入 Tier4 感知特有消息头文件和图像传输库
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
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

// TensorRT 日志接收器
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
    cv::Rect2f box;
};

class YOLO26DetectorNode : public rclcpp::Node {
public:
    YOLO26DetectorNode() : Node("yolo26_detector_node") {
        // ---- 1. 参数声明与读取 ----
        this->declare_parameter<std::string>("model_path", "yolo26n.engine");
        this->declare_parameter<std::string>("input_topic", "~/in/image"); // 【对齐 YoloX 节点】推荐使用相对路径
        this->declare_parameter<double>("score_threshold", 0.25);

        model_path_ = this->get_parameter("model_path").as_string();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        score_threshold_ = this->get_parameter("score_threshold").as_double();

        // ---- 2. 初始化 TensorRT 引擎 ----
        if (!initTensorRT()) {
            RCLCPP_ERROR(this->get_logger(), "TensorRT Engine 初始化失败！");
            return;
        }

        // ---- 3. ROS2 订阅与发布 ----
        // 【对齐 YoloX 节点】YoloX 使用的是 rmw_qos_profile_sensor_data (Best Effort)
        image_sub_ = image_transport::create_subscription(
            this, input_topic, std::bind(&YOLO26DetectorNode::imageCallback, this, std::placeholders::_1), 
            "raw", rmw_qos_profile_sensor_data);

        // 【对齐 YoloX 节点】核心修改 1：将 objects_pub_ 的类型和话题名完全对齐 YoloX
        objects_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
            "~/out/objects", 1);

        // 【对齐 YoloX 节点】核心修改 2：将 image_pub_ 的话题名对齐为 ~/out/image
        image_pub_ = image_transport::create_publisher(this, "~/out/image");

        RCLCPP_INFO(this->get_logger(), "YOLO26 节点启动，话题格式已完美对齐 YoloX。");
    }

    ~YOLO26DetectorNode() {
        if (device_buffers_[0]) cudaFree(device_buffers_[0]);
        if (device_buffers_[1]) cudaFree(device_buffers_[1]);
    }

private:
    bool initTensorRT() {
        std::ifstream file(model_path_, std::ios::binary);
        if (!file.good()) return false;
        file.seekg(0, std::ios::end);
        size_t size = file.tellg();
        file.seekg(0, std::ios::beg);
        std::vector<char> engine_data(size);
        file.read(engine_data.data(), size);
        file.close();

        runtime_ = std::unique_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(gLogger));
        engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
            runtime_->deserializeCudaEngine(engine_data.data(), size),
            std::default_delete<nvinfer1::ICudaEngine>()
        );
        context_ = std::unique_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());

        cudaMalloc(&device_buffers_[0], 1 * 3 * 640 * 640 * sizeof(float));
        cudaMalloc(&device_buffers_[1], 1 * 300 * 6 * sizeof(float));
        output_host_buffer_.resize(1 * 300 * 6);
        return true;
    }

    cv::Mat preprocessLetterbox(const cv::Mat& src, cv::Size target_size, float& scale, int& dw, int& dh) {
        int w = src.cols;
        int h = src.rows;
        scale = std::min((float)target_size.width / w, (float)target_size.height / h);
        int new_w = std::round(w * scale);
        int new_h = std::round(h * scale);
        dw = (target_size.width - new_w) / 2;
        dh = (target_size.height - new_h) / 2;
        cv::Mat resized_img;
        if (w != new_w || h != new_h) cv::resize(src, resized_img, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
        else resized_img = src;
        cv::Mat dst;
        cv::copyMakeBorder(resized_img, dst, dh, target_size.height - new_h - dh, dw, target_size.width - new_w - dw, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
        return dst;
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            return;
        }

        cv::Mat frame = cv_ptr->image;
        float scale; int dw, dh;
        cv::Mat letterbox_img = preprocessLetterbox(frame, cv::Size(640, 640), scale, dw, dh);
        cv::Mat blob;
        cv::dnn::blobFromImage(letterbox_img, blob, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false);

        cudaMemcpy(device_buffers_[0], blob.ptr<float>(), 1 * 3 * 640 * 640 * sizeof(float), cudaMemcpyHostToDevice);
        
        // ====== 【核心修复：针对动态 Engine 显式设置当前输入的 Shape】 ======
        nvinfer1::Dims input_dims;
        input_dims.nbDims = 4;
        input_dims.d[0] = 1;   // 当前实际推理的 Batch Size 为 1
        input_dims.d[1] = 3;   // Channels
        input_dims.d[2] = 640; // Height
        input_dims.d[3] = 640; // Width
        
        if (!context_->setInputShape("images", input_dims)) {
            RCLCPP_ERROR(this->get_logger(), "无法为 TensorRT 设置输入 Shape！");
            return;
        }
        // ==================================================================
        context_->setInputTensorAddress("images", device_buffers_[0]);
        context_->setOutputTensorAddress("output0", device_buffers_[1]);
        context_->enqueueV3(0);
        cudaMemcpy(output_host_buffer_.data(), device_buffers_[1], 1 * 300 * 6 * sizeof(float), cudaMemcpyDeviceToHost);

        std::vector<YOLO26Result> detections;
        for (int i = 0; i < 300; ++i) {
            int index = i * 6;
            float score = output_host_buffer_[index + 4];
            if (score < score_threshold_) continue;

            float x_min = output_host_buffer_[index + 0];
            float y_min = output_host_buffer_[index + 1];
            float x_max = output_host_buffer_[index + 2];
            float y_max = output_host_buffer_[index + 3];
            int class_id = static_cast<int>(output_host_buffer_[index + 5]);

            if (x_max <= 1.01f && y_max <= 1.01f) {
                x_min *= 640.0f; y_min *= 640.0f; x_max *= 640.0f; y_max *= 640.0f;
            }
            float x = (x_min - dw) / scale;
            float y = (y_min - dh) / scale;
            float w = (x_max - x_min) / scale;
            float h = (y_max - y_min) / scale;

            YOLO26Result det;
            det.class_id = class_id;
            det.confidence = score;
            det.box = cv::Rect2f(std::max(0.0f, x), std::max(0.0f, y), std::min(w, (float)frame.cols - x), std::min(h, (float)frame.rows - y));
            detections.push_back(det);
        }

        // 绘制 Debug 图像
        cv::Mat debug_frame = frame.clone();
        for (const auto& det : detections) {
            cv::rectangle(debug_frame, det.box, cv::Scalar(0, 0, 255), 3); // 【对齐 YoloX 节点】改用红色(0,0,255)和线宽 3
        }

        // 发布与 YoloX 完全一致的图像消息
        sensor_msgs::msg::Image::SharedPtr debug_img_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_frame).toImageMsg();
        image_pub_.publish(debug_img_msg);

        // ---- 4. 封装并发布对齐 YoloX 的特征目标消息 ----
        tier4_perception_msgs::msg::DetectedObjectsWithFeature output_objects_msg;
        output_objects_msg.header = msg->header;

        for (const auto& det : detections) {
            tier4_perception_msgs::msg::DetectedObjectWithFeature obj_with_feature;
            
            // 基础 3D 结构体对齐（初始化零位）
            obj_with_feature.object.existence_probability = det.confidence;
            autoware_perception_msgs::msg::ObjectClassification classification;
            classification.label = convertClassIdToAutowareLabel(det.class_id);
            classification.probability = 1.0f; // 【对齐 YoloX 节点】概率直接给 1.0f
            obj_with_feature.object.classification.push_back(classification);

            obj_with_feature.object.kinematics.pose_with_covariance.pose.orientation.w = 1.0;
            obj_with_feature.object.shape.type = 0; 

            // 核心 2D ROI 像素坐标填充
            obj_with_feature.feature.roi.x_offset = static_cast<uint32_t>(det.box.x);
            obj_with_feature.feature.roi.y_offset = static_cast<uint32_t>(det.box.y);
            obj_with_feature.feature.roi.width    = static_cast<uint32_t>(det.box.width);
            obj_with_feature.feature.roi.height   = static_cast<uint32_t>(det.box.height);
            obj_with_feature.feature.roi.do_rectify = false;

            output_objects_msg.feature_objects.push_back(obj_with_feature);
        }

        // 【对齐 YoloX 节点】核心发布
        objects_pub_->publish(output_objects_msg);
    }

    uint8_t convertClassIdToAutowareLabel(int yolo_id) {
        switch (yolo_id) {
            case 0: return 0; // PEDESTRIAN
            case 1: return 1; // BICYCLE
            case 2: return 2; // CAR
            case 3: return 3; // TRUCK
            case 5: return 5; // BUS
            default: return 0; // UNKNOWN
        }
    }

    std::string model_path_;
    double score_threshold_;
    
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::shared_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;

    void* device_buffers_[2] = {nullptr, nullptr}; 
    std::vector<float> output_host_buffer_; 

    // 【对齐 YoloX 节点】修改订阅和发布者句柄
    image_transport::Subscriber image_sub_;
    rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr objects_pub_;
    image_transport::Publisher image_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YOLO26DetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}