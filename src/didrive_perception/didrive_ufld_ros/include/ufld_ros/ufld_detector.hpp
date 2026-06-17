#ifndef UFLD_DETECTOR_HPP
#define UFLD_DETECTOR_HPP

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <string>
#include <chrono>

struct LanePoint {
    float x, y, z;
    uint8_t type;  // 车道线类型
};

struct LaneDetectionResult {
    std::vector<std::vector<LanePoint>> lanes_3d;
    std::vector<std::vector<cv::Point2f>> lanes_2d;  // 新增：2D车道线点
    std::vector<uint8_t> lane_types;
    std::vector<float> lane_confidences;
    std::chrono::duration<double> preprocess_time;
    std::chrono::duration<double> inference_time;
    std::chrono::duration<double> postprocess_time;
};

class UFLDDetector {
public:
    UFLDDetector();
    ~UFLDDetector();
    
    bool initialize(const std::string& engine_path, 
                   const std::vector<float>& camera_K,
                   int target_width, int target_height,
                   const std::vector<float>& mean,
                   const std::vector<float>& std,
                   int griding_num, int num_lanes, int num_styles,
                   float camera_height, float pitch_angle,
                   const std::vector<int>& row_anchors);
    
    LaneDetectionResult processImage(const cv::Mat& image);
    
    bool saveVisualization(const cv::Mat& image, 
                          const LaneDetectionResult& result,
                          const std::string& output_path);

    // 新增：3D可视化方法
    bool save3DLanesPlot(const cv::Mat& image,
                        const LaneDetectionResult& result,
                        const std::string& output_path,
                        const std::string& title = "车道线3D可视化（车辆坐标系）");

private:
    // TensorRT相关
    nvinfer1::IRuntime* runtime_;
    nvinfer1::ICudaEngine* engine_;
    nvinfer1::IExecutionContext* context_;
    void* gpu_buffers_[3];  // input, output0, output1
    int input_index_;
    std::vector<int> output_indices_;
    
    // 模型参数
    int target_width_, target_height_;
    int griding_num_, num_lanes_, num_styles_;
    std::vector<float> mean_, std_;
    std::vector<int> row_anchors_;
    
    // 相机参数
    std::vector<float> camera_K_;
    float camera_height_, pitch_angle_;
    
    // CUDA流
    cudaStream_t stream_;

    // 预分配缓冲区（新增）
    std::vector<float> output_loc_cpu_;
    std::vector<float> output_style_cpu_;
    size_t output_loc_size_;
    size_t output_style_size_;
    std::vector<std::vector<float>> style_data_;
    std::vector<std::vector<float>> style_probs_;
    std::vector<uint8_t> style_preds_;
    std::vector<float> style_confs_;

    // 内部方法
    bool loadEngine(const std::string& engine_path);
    void preprocess(const cv::Mat& image, float* gpu_input);
    std::vector<std::vector<cv::Point2f>> postprocess2D(
    float* output_loc, float* output_style, 
    LaneDetectionResult& result, 
    int original_width, int original_height);
    std::vector<std::vector<LanePoint>> ipm2DTo3D(const std::vector<std::vector<cv::Point2f>>& lanes_2d);
    std::vector<std::vector<LanePoint>> ipm2DTo3D_V2(const std::vector<std::vector<cv::Point2f>>& lanes_2d);
    std::vector<std::vector<LanePoint>> deleteFarPointsAndResample(
        const std::vector<std::vector<LanePoint>>& lanes_3d, 
        int num_points, float max_distance);
    
    // 工具方法
    std::vector<float> softmax(const float* data, int size, int axis_size);
    std::vector<uint8_t> argmax(const float* data, int size, int axis_size);
};


#endif // UFLD_DETECTOR_HPP