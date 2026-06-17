
#include "ufld_ros/ufld_detector.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <cassert>
#include <eigen3/Eigen/Dense>
#include <chrono>

// CUDA检查宏
#define CHECK_CUDA(call) \
do { \
    cudaError_t status = call; \
    if (status != cudaSuccess) { \
        std::cerr << "CUDA error at " << __FILE__ << ":" << __LINE__ << ": " << cudaGetErrorString(status) << std::endl; \
        throw std::runtime_error("CUDA error"); \
    } \
} while(0)

// TensorRT日志器
class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cout << "[TensorRT] " << msg << std::endl;
        }
    }
} logger;

UFLDDetector::UFLDDetector() 
    : runtime_(nullptr), engine_(nullptr), context_(nullptr), stream_(nullptr) {
    for (int i = 0; i < 3; ++i) {
        gpu_buffers_[i] = nullptr;
    }
}

UFLDDetector::~UFLDDetector() {
    for (int i = 0; i < 3; ++i) {
        if (gpu_buffers_[i]) {
            cudaFree(gpu_buffers_[i]);
        }
    }
    
    if (context_) {
        context_ = nullptr;
    }
    if (engine_) {
        engine_ = nullptr;
    }
    if (runtime_) {
        runtime_ = nullptr;
    }
    
    if (stream_) {
        cudaStreamDestroy(stream_);
    }
}

bool UFLDDetector::initialize(const std::string& engine_path, 
                             const std::vector<float>& camera_K,
                             int target_width, int target_height,
                             const std::vector<float>& mean,
                             const std::vector<float>& std,
                             int griding_num, int num_lanes, int num_styles,
                             float camera_height, float pitch_angle,
                             const std::vector<int>& row_anchors) {
    
    target_width_ = target_width;
    target_height_ = target_height;
    griding_num_ = griding_num;
    num_lanes_ = num_lanes;
    num_styles_ = num_styles;
    mean_ = mean;
    std_ = std;
    camera_K_ = camera_K;
    camera_height_ = camera_height;
    pitch_angle_ = pitch_angle;
    row_anchors_ = row_anchors;
    
    CHECK_CUDA(cudaStreamCreate(&stream_));
    
    if (!loadEngine(engine_path)) {
        std::cerr << "Failed to load TensorRT engine" << std::endl;
        return false;
    }
    
    std::cout << "UFLDDetector initialized successfully" << std::endl;
    // 预分配输出缓冲区
    output_loc_size_ = 1 * (griding_num_ + 1) * row_anchors_.size() * num_lanes_;
    output_style_size_ = 1 * num_styles_ * num_lanes_;
    
    output_loc_cpu_.resize(output_loc_size_);
    output_style_cpu_.resize(output_style_size_);
    
    std::cout << "Output buffers pre-allocated: loc=" << output_loc_size_ 
              << ", style=" << output_style_size_ << std::endl;
    
    style_data_.resize(num_lanes_, std::vector<float>(num_styles_));
    style_probs_.resize(num_lanes_, std::vector<float>(num_styles_));
    style_preds_.resize(num_lanes_);
    style_confs_.resize(num_lanes_);
    
    return true;
}

bool UFLDDetector::loadEngine(const std::string& engine_path) {
    auto start = std::chrono::high_resolution_clock::now();
    
    std::ifstream engine_file(engine_path, std::ios::binary);
    if (!engine_file) {
        std::cerr << "Failed to open engine file: " << engine_path << std::endl;
        return false;
    }
    
    engine_file.seekg(0, std::ios::end);
    size_t engine_size = engine_file.tellg();
    engine_file.seekg(0, std::ios::beg);
    
    std::vector<char> engine_data(engine_size);
    engine_file.read(engine_data.data(), engine_size);
    
    runtime_ = nvinfer1::createInferRuntime(logger);
    if (!runtime_) {
        std::cerr << "Failed to create TensorRT runtime" << std::endl;
        return false;
    }
    
    engine_ = runtime_->deserializeCudaEngine(engine_data.data(), engine_size);
    if (!engine_) {
        std::cerr << "Failed to deserialize CUDA engine" << std::endl;
        return false;
    }
    
    context_ = engine_->createExecutionContext();
    if (!context_) {
        std::cerr << "Failed to create execution context" << std::endl;
        return false;
    }
    
    input_index_ = 0;
    output_indices_.clear();
    
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
        const char* tensor_name = engine_->getIOTensorName(i);
        if (engine_->getTensorIOMode(tensor_name) == nvinfer1::TensorIOMode::kINPUT) {
            input_index_ = i;
        } else {
            output_indices_.push_back(i);
        }
    }
    
    auto input_dims = engine_->getTensorShape(engine_->getIOTensorName(input_index_));
    size_t input_size = 1;
    for (int i = 0; i < input_dims.nbDims; ++i) {
        input_size *= input_dims.d[i];
    }
    input_size *= sizeof(float);
    
    CHECK_CUDA(cudaMalloc(&gpu_buffers_[0], input_size));
    
    for (size_t i = 0; i < output_indices_.size(); ++i) {
        auto output_dims = engine_->getTensorShape(engine_->getIOTensorName(output_indices_[i]));
        size_t output_size = 1;
        for (int j = 0; j < output_dims.nbDims; ++j) {
            output_size *= output_dims.d[j];
        }
        output_size *= sizeof(float);
        CHECK_CUDA(cudaMalloc(&gpu_buffers_[i + 1], output_size));
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Engine loading time: " << duration.count() << "ms" << std::endl;
    
    return true;
}

void UFLDDetector::preprocess(const cv::Mat& image, float* gpu_input) {
    auto start = std::chrono::high_resolution_clock::now();
    
    cv::Mat input_image;
    if (image.channels() == 3) {
        if (image.type() == CV_8UC3) {
            input_image = image;
        } else {
            image.convertTo(input_image, CV_8UC3);
        }
    } else {
        std::cerr << "Error: Input image is not 3-channel" << std::endl;
        return;
    }

    // 预分配所有临时Mat对象
    static cv::Mat resized, rgb_img, float_img, normalized_img, chw_img;
    
    // Resize
    if (input_image.size() != cv::Size(target_width_, target_height_)) {
        cv::resize(input_image, resized, cv::Size(target_width_, target_height_), 0, 0, cv::INTER_LINEAR);
    } else {
        resized = input_image;
    }

    // BGR转RGB
    //cv::cvtColor(resized, rgb_img, cv::COLOR_BGR2RGB);

    // 转换为float并归一化
    resized.convertTo(float_img, CV_32FC3, 1.0f / 255.0f);

    // 标准化 - 使用预计算的逆标准差
    static float inv_std[3];
    static bool inv_std_computed = false;
    if (!inv_std_computed) {
        inv_std[0] = 1.0f / std_[0];
        inv_std[1] = 1.0f / std_[1];
        inv_std[2] = 1.0f / std_[2];
        inv_std_computed = true;
    }
    
    // 直接在原图上操作避免额外拷贝
    std::vector<cv::Mat> channels(3);
    cv::split(float_img, channels);
    
    for (int c = 0; c < 3; ++c) {
        channels[c] = (channels[c] - mean_[c]) * inv_std[c];
    }
    
    cv::merge(channels, normalized_img);

    // HWC转CHW优化
    int height = normalized_img.rows;
    int width = normalized_img.cols;
    int channel_size = height * width;
    
    // 预分配CHW矩阵
    if (chw_img.empty() || chw_img.rows != 3 || chw_img.cols != channel_size) {
        chw_img.create(3, channel_size, CV_32FC1);
    }
    
    float* chw_data = chw_img.ptr<float>();
    float* norm_data = normalized_img.ptr<float>();
    
    // 优化内存访问模式
    for (int c = 0; c < 3; ++c) {
        float* channel_dst = chw_data + c * channel_size;
        for (int i = 0; i < height; ++i) {
            const float* src_row = norm_data + (i * width * 3) + c;
            float* dst_row = channel_dst + i * width;
            
            // 使用memcpy进行批量拷贝
            for (int j = 0; j < width; ++j) {
                dst_row[j] = src_row[j * 3];
            }
        }
    }

    // 异步拷贝到GPU
    CHECK_CUDA(cudaMemcpyAsync(gpu_input, chw_data, 
                              channel_size * 3 * sizeof(float),
                              cudaMemcpyHostToDevice, stream_));
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Preprocessing time: " << duration.count() << "ms" << std::endl;
}

LaneDetectionResult UFLDDetector::processImage(const cv::Mat& image) {
    LaneDetectionResult result;
    auto total_start = std::chrono::high_resolution_clock::now();
    
    int original_width = image.cols;
    int original_height = image.rows;
    
    // 预处理
    auto preprocess_start = std::chrono::high_resolution_clock::now();
    preprocess(image, static_cast<float*>(gpu_buffers_[0]));
    auto preprocess_end = std::chrono::high_resolution_clock::now();
    result.preprocess_time = preprocess_end - preprocess_start;
    
    // 设置Tensor地址
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
        const char* tensor_name = engine_->getIOTensorName(i);
        context_->setTensorAddress(tensor_name, gpu_buffers_[i == input_index_ ? 0 : 1]);
    }
    
    // 推理执行
    auto inference_start = std::chrono::high_resolution_clock::now();
    if (!context_->executeV2(gpu_buffers_)) {
        std::cerr << "Failed to execute inference" << std::endl;
        return result;
    }
    cudaStreamSynchronize(stream_);
    auto inference_end = std::chrono::high_resolution_clock::now();
    result.inference_time = inference_end - inference_start;
    auto inference_duration = std::chrono::duration_cast<std::chrono::milliseconds>(inference_end - inference_start);
    std::cout << "inference time: " << inference_duration.count() << "ms" << std::endl;
    // 使用预分配缓冲区进行后处理
    auto postprocess_start = std::chrono::high_resolution_clock::now();
    
    // 异步拷贝到预分配的CPU缓冲区
    CHECK_CUDA(cudaMemcpyAsync(output_loc_cpu_.data(), gpu_buffers_[1], 
                              output_loc_size_ * sizeof(float),
                              cudaMemcpyDeviceToHost, stream_));
    CHECK_CUDA(cudaMemcpyAsync(output_style_cpu_.data(), gpu_buffers_[2], 
                              output_style_size_ * sizeof(float),
                              cudaMemcpyDeviceToHost, stream_));
    cudaStreamSynchronize(stream_);
    
    auto lanes_2d = postprocess2D(output_loc_cpu_.data(), output_style_cpu_.data(), 
                                 result, original_width, original_height);
    
    result.lanes_2d = lanes_2d;
    result.lanes_3d = ipm2DTo3D_V2(lanes_2d);
    result.lanes_3d = deleteFarPointsAndResample(result.lanes_3d, 40, 40.0);
    
    auto postprocess_end = std::chrono::high_resolution_clock::now();
    result.postprocess_time = postprocess_end - postprocess_start;
    
    auto total_end = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start);
    std::cout << "Total processing time: " << total_duration.count() << "ms" << std::endl;
    
    return result;
}

std::vector<float> UFLDDetector::softmax(const float* data, int size, int axis_size) {
    std::vector<float> result(size);
    for (int i = 0; i < size / axis_size; ++i) {
        const float* slice = data + i * axis_size;
        float max_val = *std::max_element(slice, slice + axis_size);
        float sum = 0.0f;
        
        for (int j = 0; j < axis_size; ++j) {
            result[i * axis_size + j] = std::exp(slice[j] - max_val);
            sum += result[i * axis_size + j];
        }
        
        for (int j = 0; j < axis_size; ++j) {
            result[i * axis_size + j] /= sum;
        }
    }
    return result;
}

std::vector<uint8_t> UFLDDetector::argmax(const float* data, int size, int axis_size) {
    std::vector<uint8_t> result(size / axis_size);
    for (int i = 0; i < size / axis_size; ++i) {
        const float* slice = data + i * axis_size;
        result[i] = static_cast<uint8_t>(std::max_element(slice, slice + axis_size) - slice);
    }
    return result;
}

std::vector<std::vector<cv::Point2f>> UFLDDetector::postprocess2D(
    float* output_loc, float* output_style, 
    LaneDetectionResult& result,
    int original_width, int original_height) {
    
    auto start = std::chrono::high_resolution_clock::now();
    
    std::vector<std::vector<cv::Point2f>> lanes_2d;

    float scale_x = static_cast<float>(original_width) / target_width_;
    float scale_y = static_cast<float>(original_height) / target_height_;
    
    // ========== 主要修改开始 ==========
    // 修正维度理解：output_style 的形状是 [1, num_styles_, num_lanes_]
    // 所以在样式维度(axis=1)做softmax
    
    // 重新组织数据：从 [num_styles_ * num_lanes_] 到 [num_lanes_, num_styles_]
    std::vector<std::vector<float>> style_data(num_lanes_, std::vector<float>(num_styles_));
    for (int lane = 0; lane < num_lanes_; ++lane) {
        for (int style = 0; style < num_styles_; ++style) {
            // 索引计算：style * num_lanes_ + lane
            int idx = style * num_lanes_ + lane;
            style_data[lane][style] = output_style[idx];
        }
    }
    
    // 对每个车道线的样式概率做softmax
    std::vector<std::vector<float>> style_probs(num_lanes_, std::vector<float>(num_styles_));
    std::vector<uint8_t> style_preds(num_lanes_);
    std::vector<float> style_confs(num_lanes_);
    
    for (int lane = 0; lane < num_lanes_; ++lane) {
        // softmax计算
        float max_val = *std::max_element(style_data[lane].begin(), style_data[lane].end());
        float sum = 0.0f;
        
        for (int style = 0; style < num_styles_; ++style) {
            style_probs[lane][style] = std::exp(style_data[lane][style] - max_val);
            sum += style_probs[lane][style];
        }
        
        for (int style = 0; style < num_styles_; ++style) {
            style_probs[lane][style] /= sum;
        }
        
        // 获取预测类型和置信度
        auto max_it = std::max_element(style_probs[lane].begin(), style_probs[lane].end());
        style_preds[lane] = static_cast<uint8_t>(max_it - style_probs[lane].begin());
        style_confs[lane] = *max_it;
    }
    // ========== 主要修改结束 ==========
    
    int cls_num_per_lane = row_anchors_.size();
    float col_sample_w = static_cast<float>(target_width_) / griding_num_;
    
    std::vector<std::vector<std::vector<float>>> output_3d(
        griding_num_ + 1, 
        std::vector<std::vector<float>>(
            cls_num_per_lane, 
            std::vector<float>(num_lanes_)
        )
    );
    
    for (int i = 0; i < griding_num_ + 1; ++i) {
        for (int j = 0; j < cls_num_per_lane; ++j) {
            for (int k = 0; k < num_lanes_; ++k) {
                int idx = i * (cls_num_per_lane * num_lanes_) + j * num_lanes_ + k;
                output_3d[i][j][k] = output_loc[idx];
            }
        }
    }
    
    for (int i = 0; i < griding_num_ + 1; ++i) {
        std::reverse(output_3d[i].begin(), output_3d[i].end());
    }
    
    std::vector<std::vector<float>> loc(cls_num_per_lane, std::vector<float>(num_lanes_, 0.0f));
    std::vector<std::vector<int>> out_j_argmax(cls_num_per_lane, std::vector<int>(num_lanes_, 0));
    
    for (int j = 0; j < cls_num_per_lane; ++j) {
        for (int k = 0; k < num_lanes_; ++k) {
            float max_val = output_3d[0][j][k];
            int max_idx = 0;
            for (int i = 0; i < griding_num_ + 1; ++i) {
                if (output_3d[i][j][k] > max_val) {
                    max_val = output_3d[i][j][k];
                    max_idx = i;
                }
            }
            out_j_argmax[j][k] = max_idx;
        }
    }
    
    for (int j = 0; j < cls_num_per_lane; ++j) {
        for (int k = 0; k < num_lanes_; ++k) {
            std::vector<float> prob_slice(griding_num_);
            for (int i = 0; i < griding_num_; ++i) {
                prob_slice[i] = output_3d[i][j][k];
            }
            
            auto prob_softmax = softmax(prob_slice.data(), griding_num_, griding_num_);
            
            float sum = 0.0f;
            for (int i = 0; i < griding_num_; ++i) {
                sum += prob_softmax[i] * (i + 1);
            }
            loc[j][k] = sum;
            
            if (out_j_argmax[j][k] == griding_num_) {
                loc[j][k] = 0;
            }
        }
    }
    
    for (int k = 0; k < num_lanes_; ++k) {
        std::vector<cv::Point2f> lane_points;
        int valid_points = 0;
        
        for (int j = 0; j < cls_num_per_lane; ++j) {
            if (loc[j][k] != 0) {
                valid_points++;
            }
        }
        
        if (valid_points > 4) {
            for (int j = 0; j < cls_num_per_lane; ++j) {
                if (loc[j][k] > 0) {
                    float u = loc[j][k] * col_sample_w * scale_x;
                    float v = row_anchors_[cls_num_per_lane - 1 - j] * scale_y;
                    
                    lane_points.emplace_back(u, v);
                }
            }
            
            lanes_2d.push_back(lane_points);
            // ========== 修改：使用修正后的样式预测 ==========
            result.lane_types.push_back(style_preds[k]);
            result.lane_confidences.push_back(style_confs[k]);
            // ========== 修改结束 ==========
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "2D postprocessing time: " << duration.count() << "μs" << std::endl;
    
    return lanes_2d;
}


std::vector<std::vector<LanePoint>> UFLDDetector::ipm2DTo3D_V2(const std::vector<std::vector<cv::Point2f>>& lanes_2d)
{
    auto start = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<LanePoint>> lanes_3d;

    // 检查内参矩阵是否有效
    if (camera_K_.size() != 9) {
        std::cerr << "Invalid camera intrinsic matrix: expected 9 elements, got " 
                  << camera_K_.size() << std::endl;
        return lanes_3d;
    }

    // 构造相机内参矩阵 K 和其逆矩阵
    Eigen::Matrix3f K;
    K << camera_K_[0], camera_K_[1], camera_K_[2],
         camera_K_[3], camera_K_[4], camera_K_[5],
         camera_K_[6], camera_K_[7], camera_K_[8];

    Eigen::Matrix3f K_inv = K.inverse();

    // 角度转弧度
    float pitch_rad = pitch_angle_ * M_PI / 180.0f;
    float cos_p = std::cos(pitch_rad);
    float sin_p = std::sin(pitch_rad);

    // 相机坐标系 → 车辆坐标系的基础旋转（无pitch）
    Eigen::Matrix3f R_camera_to_vehicle;
    R_camera_to_vehicle << 0,  0,  1,
                          -1,  0,  0,
                           0, -1,  0;

    // 俯仰角修正旋转（绕相机x轴向下倾斜）
    Eigen::Matrix3f R_pitch;
    R_pitch << 1,      0,        0,
               0,  cos_p,   -sin_p,
               0,  sin_p,    cos_p;

    // 总旋转：相机坐标系 → 车辆坐标系
    Eigen::Matrix3f R_total = R_camera_to_vehicle * R_pitch;

    // R_inv = R_total^T：从车辆坐标系转回相机坐标系
    Eigen::Matrix3f R_inv = R_total.transpose();

    for (const auto& lane_2d : lanes_2d) {
        std::vector<LanePoint> lane_3d;

        for (const auto& point_2d : lane_2d) {
            // Step 1: 反投影到归一化平面（相机坐标系方向）
            Eigen::Vector3f uv1(point_2d.x, point_2d.y, 1.0f);
            Eigen::Vector3f d_c = K_inv * uv1;

            // 单位化可选（不影响结果，但数值更稳定）
            d_c.normalize();

            // Step 2: 将射线方向变换到车辆坐标系
            Eigen::Vector3f d_v = R_total * d_c;

            // Step 3: 射线方程 P(t) = camera_origin_in_vehicle + t * d_v
            // 相机在车辆系中位置：[0, 0, camera_height]
            // 地面：Z = 0
            if (std::abs(d_v.z()) < 1e-6f) {
                continue; // 射线几乎平行于地面
            }

            float t = -camera_height_ / d_v.z(); // 因为 z(t) = camera_height + t * d_v.z() = 0

            // 交点在车辆坐标系中
            Eigen::Vector3f point_vehicle(t * d_v.x(), t * d_v.y(), 0.0f);

            // Step 4: 计算该点相对于相机中心的向量（仍在车辆系）
            Eigen::Vector3f relative_in_vehicle = point_vehicle - Eigen::Vector3f(0.0f, 0.0f, camera_height_);

            // 转换到相机坐标系
            Eigen::Vector3f point_camera = R_inv * relative_in_vehicle;

            // 构造 LanePoint 并赋值
            LanePoint lane_point;
            lane_point.x = point_camera.x();
            lane_point.y = point_camera.y();
            lane_point.z = point_camera.z();
            lane_point.type = 0; // 默认类型

            lane_3d.push_back(lane_point);
        }

        // 只有非空才加入结果
        if (!lane_3d.empty()) {
            lanes_3d.push_back(lane_3d);
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "IPM 2D to Camera Coord conversion time: " << duration.count() << " μs" << std::endl;

    return lanes_3d;
}

std::vector<std::vector<LanePoint>> UFLDDetector::ipm2DTo3D(const std::vector<std::vector<cv::Point2f>>& lanes_2d) {
    auto start = std::chrono::high_resolution_clock::now();
    
    std::vector<std::vector<LanePoint>> lanes_3d;
    
    if (camera_K_.size() != 9) {
        std::cerr << "Invalid camera intrinsic matrix" << std::endl;
        return lanes_3d;
    }
    
    Eigen::Matrix3f K;
    K << camera_K_[0], camera_K_[1], camera_K_[2],
         camera_K_[3], camera_K_[4], camera_K_[5],
         camera_K_[6], camera_K_[7], camera_K_[8];
    
    Eigen::Matrix3f K_inv = K.inverse();
    
    float pitch_rad = pitch_angle_ * M_PI / 180.0f;
    float cos_pitch = std::cos(pitch_rad);
    float sin_pitch = std::sin(pitch_rad);
    
    Eigen::Matrix3f R_camera_to_vehicle;
    R_camera_to_vehicle << 0, 0, 1,
                          -1, 0, 0,
                           0, -1, 0;
    
    Eigen::Matrix3f R_pitch;
    R_pitch << 1, 0, 0,
               0, cos_pitch, -sin_pitch,
               0, sin_pitch, cos_pitch;
    
    Eigen::Matrix3f R_total = R_camera_to_vehicle * R_pitch;
    
    Eigen::Vector3f normal_camera(0, -cos_pitch, sin_pitch);
    Eigen::Vector3f point_on_ground_camera(0, camera_height_, 0);
    
    for (const auto& lane_2d : lanes_2d) {
        std::vector<LanePoint> lane_3d;
        
        for (const auto& point_2d : lane_2d) {
            Eigen::Vector3f uv1(point_2d.x, point_2d.y, 1.0f);
            Eigen::Vector3f point_normalized = K_inv * uv1;
            
            float denominator = normal_camera.dot(point_normalized);
            // if (std::abs(denominator) < 1e-6f) {
            //     continue;
            // }
            
            float t = normal_camera.dot(point_on_ground_camera) / denominator;
            Eigen::Vector3f point_camera = t * point_normalized;
            
            Eigen::Vector3f point_vehicle = R_total * (point_camera - point_on_ground_camera);
            
            LanePoint lane_point;
            lane_point.x = point_vehicle.x();
            lane_point.y = point_vehicle.y();
            lane_point.z = point_vehicle.z();
            lane_point.type = 0;
            
            lane_3d.push_back(lane_point);
        }
        
        if (!lane_3d.empty()) {
            lanes_3d.push_back(lane_3d);
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "3D conversion time: " << duration.count() << "μs" << std::endl;
    
    return lanes_3d;
}

std::vector<std::vector<LanePoint>> UFLDDetector::deleteFarPointsAndResample(
    const std::vector<std::vector<LanePoint>>& lanes_3d, 
    int num_points, float max_distance) {
    
    auto start = std::chrono::high_resolution_clock::now();
    
    std::vector<std::vector<LanePoint>> result;
    
    for (const auto& lane : lanes_3d) {
        std::vector<LanePoint> filtered_points;
        for (const auto& point : lane) {
            if (point.x <= max_distance) {
                filtered_points.push_back(point);
            }
        }
        
        if (filtered_points.size() < 2) {
            continue;
        }
        
        std::sort(filtered_points.begin(), filtered_points.end(),
                 [](const LanePoint& a, const LanePoint& b) { return a.x < b.x; });
        
        std::vector<LanePoint> resampled_points;
        std::vector<float> x_original, y_original, z_original;
        
        for (const auto& point : filtered_points) {
            x_original.push_back(point.x);
            y_original.push_back(point.y);
            z_original.push_back(point.z);
        }
        
        float x_min = x_original.front();
        float x_max = x_original.back();
        
        for (int i = 0; i < num_points; ++i) {
            float x_new = x_min + (x_max - x_min) * i / (num_points - 1);
            
            LanePoint new_point;
            new_point.x = x_new;
            
            auto it = std::lower_bound(x_original.begin(), x_original.end(), x_new);
            if (it == x_original.begin()) {
                new_point.y = y_original[0];
                new_point.z = z_original[0];
            } else if (it == x_original.end()) {
                new_point.y = y_original.back();
                new_point.z = z_original.back();
            } else {
                size_t idx = it - x_original.begin();
                float x0 = x_original[idx - 1];
                float x1 = x_original[idx];
                float t = (x_new - x0) / (x1 - x0);
                
                new_point.y = y_original[idx - 1] + t * (y_original[idx] - y_original[idx - 1]);
                new_point.z = z_original[idx - 1] + t * (z_original[idx] - z_original[idx - 1]);
            }
            
            new_point.type = filtered_points[0].type;
            resampled_points.push_back(new_point);
        }
        
        result.push_back(resampled_points);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Filtering and resampling time: " << duration.count() << "μs" << std::endl;
    
    return result;
}

bool UFLDDetector::saveVisualization(const cv::Mat& image, 
                                   const LaneDetectionResult& result,
                                   const std::string& output_path) {
    //cv::Mat vis_image = image.clone();
    cv::Mat vis_image;
    cv::cvtColor(image, vis_image, cv::COLOR_RGB2BGR);
    std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0),
        cv::Scalar(0, 255, 0),  
        cv::Scalar(0, 0, 255),
        cv::Scalar(255, 255, 0),
        cv::Scalar(255, 0, 255),
        cv::Scalar(0, 255, 255)
    };
    
    std::map<uint8_t, std::string> type_names = {
        {0, "unknown"}, {1, "white-dash"}, {2, "white-solid"}, {3, "double-white-dash"},
        {4, "double-white-solid"}, {5, "white-ldash-rsolid"}, {6, "white-solid-rdash"},
        {7, "yellow-dash"}, {8, "yellow-solid"}, {9, "double-yellow-dash"},
        {10, "double-yellow-solid"}, {11, "yellow-ldash-rsolid"}, {12, "yellow-lsolid-rdash"},
        {13, "left-curbside"}, {14, "right-curbside"}
    };
    
    float scale_x = static_cast<float>(image.cols) / target_width_;
    float scale_y = static_cast<float>(image.rows) / target_height_;
    
    for (size_t i = 0; i < result.lanes_2d.size(); ++i) {
        const auto& lane_2d = result.lanes_2d[i];
        cv::Scalar color = colors[i % colors.size()];
        uint8_t lane_type = (i < result.lane_types.size()) ? result.lane_types[i] : 0;
        float conf = (i < result.lane_confidences.size()) ? result.lane_confidences[i] : 0.0f;
        
        std::string type_name = type_names.count(lane_type) ? type_names[lane_type] : "unknown";
        std::string label = "Lane " + std::to_string(i+1) + ": " + type_name + " (" + std::to_string(conf).substr(0, 4) + ")";
        
        if (!lane_2d.empty()) {
            cv::putText(vis_image, label, 
                       cv::Point(10, 30 + static_cast<int>(i) * 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
            
            for (size_t j = 0; j < lane_2d.size(); ++j) {
                const auto& point = lane_2d[j];
                int x = static_cast<int>(point.x);
                int y = static_cast<int>(point.y);
                
                x = std::max(0, std::min(x, vis_image.cols - 1));
                y = std::max(0, std::min(y, vis_image.rows - 1));
                
                cv::circle(vis_image, cv::Point(x, y), 6, color, 5);
            }
            
            const auto& first_point = lane_2d[0];
            int x = static_cast<int>(first_point.x);
            int y = static_cast<int>(first_point.y);
            x = std::max(0, std::min(x, vis_image.cols - 1));
            y = std::max(0, std::min(y, vis_image.rows - 1));
            
            cv::putText(vis_image, std::to_string(i + 1), 
                       cv::Point(x + 15, y), 
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, color, 3);
        }
    }
    
    if (result.lanes_2d.empty()) {
        cv::putText(vis_image, "No lanes detected", 
                   cv::Point(vis_image.cols / 2 - 100, vis_image.rows / 2),
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    }
    
    bool success = cv::imwrite(output_path, vis_image);
    if (success) {
        std::cout << "Visualization saved to: " << output_path << std::endl;
    } else {
        std::cerr << "Failed to save visualization to: " << output_path << std::endl;
    }
    
    return success;
}

bool UFLDDetector::save3DLanesPlot(const cv::Mat& image,
                                  const LaneDetectionResult& result,
                                  const std::string& output_path,
                                  const std::string& title) {
    try {
        // 创建一个大画布：左侧原图，右侧XY投影
        int canvas_width = image.cols * 2;
        int canvas_height = image.rows;
        cv::Mat canvas = cv::Mat::zeros(canvas_height, canvas_width, CV_8UC3);
        
        // 1. 左侧放置原图
        cv::Mat left_roi = canvas(cv::Rect(0, 0, image.cols, image.rows));
        image.copyTo(left_roi);
        
        // 2. 右侧创建XY投影图（俯视图）
        cv::Mat right_roi = canvas(cv::Rect(image.cols, 0, image.cols, image.rows));
        right_roi.setTo(cv::Scalar(255, 255, 255)); // 白色背景
        
        // 定义不同颜色
        std::vector<cv::Scalar> colors = {
            cv::Scalar(255, 0, 0),     // 红色
            cv::Scalar(0, 255, 0),     // 绿色
            cv::Scalar(0, 0, 255),     // 蓝色
            cv::Scalar(255, 255, 0),   // 青色
            cv::Scalar(255, 0, 255),   // 洋红
            cv::Scalar(0, 255, 255),   // 黄色
            cv::Scalar(128, 0, 128),   // 紫色
            cv::Scalar(128, 128, 0)    // 橄榄色
        };
        
        // 车道线类型名称映射
        std::map<uint8_t, std::string> type_names = {
            {0, "unknown"}, {1, "white-dash"}, {2, "white-solid"}, {3, "double-white-dash"},
            {4, "double-white-solid"}, {5, "white-ldash-rsolid"}, {6, "white-solid-rdash"},
            {7, "yellow-dash"}, {8, "yellow-solid"}, {9, "double-yellow-dash"},
            {10, "double-yellow-solid"}, {11, "yellow-ldash-rsolid"}, {12, "yellow-lsolid-rdash"},
            {13, "left-curbside"}, {14, "right-curbside"}
        };
        
        // 计算所有3D点的范围，用于坐标映射
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();
        
        for (const auto& lane_3d : result.lanes_3d) {
            for (const auto& point : lane_3d) {
                min_x = std::min(min_x, point.x);
                max_x = std::max(max_x, point.x);
                min_y = std::min(min_y, point.y);
                max_y = std::max(max_y, point.y);
            }
        }
        
        // 如果没有任何点，设置默认范围
        if (result.lanes_3d.empty()) {
            min_x = -10.0f; max_x = 10.0f;
            min_y = -10.0f; max_y = 10.0f;
        }
        
        // 添加边界裕量
        float margin_x = (max_x - min_x) * 0.1f;
        float margin_y = (max_y - min_y) * 0.1f;
        min_x -= margin_x; max_x += margin_x;
        min_y -= margin_y; max_y += margin_y;
        
        // 坐标映射函数：将3D坐标映射到图像坐标
        auto mapToImage = [&](float x, float y) -> cv::Point {
            int img_x = static_cast<int>((x - min_x) / (max_x - min_x) * (image.cols - 100) + 50);
            int img_y = static_cast<int>((max_y - y) / (max_y - min_y) * (image.rows - 100) + 50);
            return cv::Point(img_x, img_y);
        };
        
        // 绘制坐标轴
        cv::Point origin = mapToImage(0, 0);
        cv::arrowedLine(right_roi, origin, mapToImage(5, 0), cv::Scalar(0, 0, 0), 2, 8, 0, 0.1);
        cv::arrowedLine(right_roi, origin, mapToImage(0, 5), cv::Scalar(0, 0, 0), 2, 8, 0, 0.1);
        cv::putText(right_roi, "X", mapToImage(6, 0), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
        cv::putText(right_roi, "Y", mapToImage(0, 6), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
        
        // 绘制网格
        for (float x = std::ceil(min_x); x <= max_x; x += 5.0f) {
            cv::Point p1 = mapToImage(x, min_y);
            cv::Point p2 = mapToImage(x, max_y);
            cv::line(right_roi, p1, p2, cv::Scalar(200, 200, 200), 1);
            if (x != 0) {
                cv::putText(right_roi, std::to_string(static_cast<int>(x)) + "m", 
                           mapToImage(x, min_y + 1), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 100), 1);
            }
        }
        for (float y = std::ceil(min_y); y <= max_y; y += 5.0f) {
            cv::Point p1 = mapToImage(min_x, y);
            cv::Point p2 = mapToImage(max_x, y);
            cv::line(right_roi, p1, p2, cv::Scalar(200, 200, 200), 1);
            if (y != 0) {
                cv::putText(right_roi, std::to_string(static_cast<int>(y)) + "m", 
                           mapToImage(min_x + 1, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 100), 1);
            }
        }
        
        // 绘制每个车道线
        for (size_t i = 0; i < result.lanes_3d.size(); ++i) {
            const auto& lane_3d = result.lanes_3d[i];
            cv::Scalar color = colors[i % colors.size()];
            
            uint8_t lane_type = (i < result.lane_types.size()) ? result.lane_types[i] : 0;
            float conf = (i < result.lane_confidences.size()) ? result.lane_confidences[i] : 0.0f;
            std::string type_name = type_names.count(lane_type) ? type_names[lane_type] : "unknown";
            
            // 在右侧绘制XY投影
            std::vector<cv::Point> points_2d;
            for (const auto& point : lane_3d) {
                points_2d.push_back(mapToImage(point.x, point.y));
            }
            
            // 绘制连线
            if (points_2d.size() >= 2) {
                for (size_t j = 0; j < points_2d.size() - 1; ++j) {
                    cv::line(right_roi, points_2d[j], points_2d[j+1], color, 3, cv::LINE_AA);
                }
            }
            
            // 绘制点
            for (const auto& point : points_2d) {
                cv::circle(right_roi, point, 4, color, -1);
            }
            
            // 添加标签
            if (!points_2d.empty()) {
                std::string label = "L" + std::to_string(i+1) + ": " + type_name;
                cv::putText(right_roi, label, points_2d[0] + cv::Point(10, 0), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
            }
            
            // 在左侧原图上也绘制2D检测结果
            const auto& lane_2d = result.lanes_2d[i];
            for (size_t j = 0; j < lane_2d.size(); ++j) {
                const auto& point = lane_2d[j];
                int x = static_cast<int>(point.x);
                int y = static_cast<int>(point.y);
                x = std::max(0, std::min(x, image.cols - 1));
                y = std::max(0, std::min(y, image.rows - 1));
                
                cv::circle(left_roi, cv::Point(x, y), 6, color, -1);
                if (j < lane_2d.size() - 1) {
                    const auto& next_point = lane_2d[j+1];
                    int x2 = static_cast<int>(next_point.x);
                    int y2 = static_cast<int>(next_point.y);
                    x2 = std::max(0, std::min(x2, image.cols - 1));
                    y2 = std::max(0, std::min(y2, image.rows - 1));
                    cv::line(left_roi, cv::Point(x, y), cv::Point(x2, y2), color, 2);
                }
            }
        }
        
        // 添加标题和说明
        cv::putText(canvas, "Original Image with 2D Detection", 
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
        cv::putText(canvas, "XY Projection (Top View)", 
                   cv::Point(image.cols + 10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
        
        // 添加统计信息
        std::string stats = "Detected Lanes: " + std::to_string(result.lanes_3d.size());
        cv::putText(canvas, stats, cv::Point(10, canvas.rows - 20), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        
        // 保存图片
        bool success = cv::imwrite(output_path, canvas);
        if (success) {
            std::cout << "3D lanes visualization saved to: " << output_path << std::endl;
        } else {
            std::cerr << "Failed to save 3D visualization to: " << output_path << std::endl;
        }
        
        return success;
        
    } catch (const std::exception& e) {
        std::cerr << "Error in save3DLanesPlot: " << e.what() << std::endl;
        return false;
    }
}
