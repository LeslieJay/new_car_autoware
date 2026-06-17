// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef V4L2_CAMERA__V4L2_CAMERA_HPP_
#define V4L2_CAMERA__V4L2_CAMERA_HPP_

#include "v4l2_camera/v4l2_camera.hpp"
#include "v4l2_camera/v4l2_camera_device.hpp"

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <ostream>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>

#include <memory>
#include <string>
#include <map>
#include <vector>
#include <optional>
#include <mutex>
#include <chrono>

#include "v4l2_camera/visibility_control.h"

#ifdef ENABLE_CUDA
#include <nppdefs.h>
#include <nppi_support_functions.h>

#include <npp.h>
#include "v4l2_camera/utility.hpp"
#include "v4l2_camera/undistort_kernel.h"
#endif

namespace v4l2_camera
{
#ifdef ENABLE_CUDA
struct GPUMemoryManager
{
 public:
  Npp8u * dev_ptr;
  int step_bytes;
  size_t allocated_size;
  int channels;

  explicit GPUMemoryManager()
      : dev_ptr(nullptr), step_bytes(0), allocated_size(0), channels(3)
  {}

  ~GPUMemoryManager()
  {
    if (dev_ptr != nullptr) {
      nppiFree(dev_ptr);
    }
  }

  void Allocate(int width, int height, int ch = 3)
  {
    channels = ch;
    if (dev_ptr == nullptr || allocated_size < static_cast<size_t>(height * step_bytes)) {
      if (channels == 3) {
        dev_ptr = nppiMalloc_8u_C3(width, height, &step_bytes);
      } else if (channels == 2) {
        dev_ptr = nppiMalloc_8u_C2(width, height, &step_bytes);
      } else {
        throw std::invalid_argument("Unsupported number of channels");
      }
      allocated_size = static_cast<size_t>(height * step_bytes);
    }
  }

  void Allocate(int width, int height, size_t custom_step, int ch = 3)
  {
    channels = ch;
    size_t required_size = custom_step * static_cast<size_t>(height);
    if (dev_ptr == nullptr || allocated_size < required_size) {
      if (dev_ptr != nullptr) {
        nppiFree(dev_ptr);
      }
      cudaError_t err = cudaMalloc(&dev_ptr, required_size);
      if (err != cudaSuccess) {
        throw std::runtime_error("cudaMalloc failed: " + std::string(cudaGetErrorString(err)));
      }
      step_bytes = custom_step;
      allocated_size = required_size;
    }
  }
};


void cudaErrorCheck(const cudaError_t e)
{
  if (e != cudaSuccess) {
    std::stringstream ss;
    ss << cudaGetErrorName(e) << " : " << cudaGetErrorString(e);
    throw std::runtime_error{ss.str()};
  }
}
#endif

inline rclcpp::Time getSteadyTime() {
    const auto now_chrono = std::chrono::steady_clock::now();
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now_chrono.time_since_epoch()
    );
    const uint64_t sec = now_ns.count() / 1'000'000'000;
    const uint64_t nsec = now_ns.count() % 1'000'000'000;
    return rclcpp::Time(sec, nsec, RCL_STEADY_TIME);
}

class V4L2Camera : public rclcpp::Node
{
public:
  explicit V4L2Camera(rclcpp::NodeOptions const & options);

  virtual ~V4L2Camera();

private:
  using ImageSize = std::vector<int64_t>;
  using TimePerFrame = std::vector<int64_t>;

  std::shared_ptr<V4l2CameraDevice> camera_;

  // Publisher used for intra process comm
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;

  // Publisher used for inter process comm
  image_transport::CameraPublisher camera_transport_pub_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  std::thread capture_thread_;
  std::atomic<bool> canceled_;

  std::string camera_frame_id_;
  std::string output_encoding_;
  std::string image_topic_;

  std::map<std::string, int32_t> control_name_to_id_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_;

  double publish_rate_;
  rclcpp::TimerBase::SharedPtr image_pub_timer_;

  bool publish_next_frame_;
  bool use_image_transport_;

  std::shared_ptr<diagnostic_updater::Updater> diag_updater_;
  std::shared_ptr<diagnostic_updater::CompositeDiagnosticTask> diag_composer_;

  // To keep publishing diagnostics even after the constuctor of V4L2Camera class fail,
  // this FunctionDiagnosticTask should be declared as a class member due to lifetime
  std::shared_ptr<diagnostic_updater::FunctionDiagnosticTask> device_node_existence_diag_;

  std::optional<TimePerFrame> time_per_frame_;
  std::optional<double> min_ok_rate_;
  std::optional<double> max_ok_rate_;
  std::optional<double> min_warn_rate_;
  std::optional<double> max_warn_rate_;
  int observed_frames_transition_;

  std::mutex lock_;

#ifdef ENABLE_CUDA
  // Memory region to communicate with GPU
  std::allocator<GPUMemoryManager> allocator_;
  std::shared_ptr<GPUMemoryManager> src_dev_;
  std::shared_ptr<GPUMemoryManager> dst_dev_;
#endif

  void createParameters();
  bool handleParameter(rclcpp::Parameter const & param);

  bool requestPixelFormat(std::string const & fourcc);
  bool requestImageSize(ImageSize const & size);

  bool requestTimePerFrame(TimePerFrame const & timePerFrame);

  sensor_msgs::msg::Image::UniquePtr convert(sensor_msgs::msg::Image const & img) const;
#ifdef ENABLE_CUDA
  sensor_msgs::msg::Image::UniquePtr convertOnGpu(sensor_msgs::msg::Image const & img);

  // gpu去畸变
  bool undistort_enabled_ = false;
  void init_undistorters();
  sensor_msgs::msg::Image::UniquePtr undistortOnGpu(sensor_msgs::msg::Image const & img);
  cudaStream_t stream_;
  NppStreamContext nppStreamCtx_;
  // 前处理去畸变
  unsigned char *d_dst_;
  float *d_mapX_, *d_mapY_;
  // define undistort params
  UndistParams params_800w_, params_300w_;
  bool is_params_800w_ = false;
  bool is_camera_30_ = false;
  const int channels_ = 3;
  // define map size
  size_t map_size_;
  // define img size
  size_t img_size_;

#endif

  bool checkCameraInfo(
    sensor_msgs::msg::Image const & img,
    sensor_msgs::msg::CameraInfo const & ci);
};

}  // namespace v4l2_camera

#endif  // V4L2_CAMERA__V4L2_CAMERA_HPP_
