#include "can_driver/can_receiver.hpp"
#include "can_driver/can_node.hpp"

#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "can_driver/sixents_sdk.h"
#include "can_driver/sixents_types.h"

using namespace can_driver;

extern std::atomic<bool> g_six_sdk_running;

namespace
{

std::string buildGGAString(const ins_pos_can_t & data)
{
  double lng = static_cast<double>(data.pos_ins[0]) / INT_POS;
  double lat = static_cast<double>(data.pos_ins[1]) / INT_POS;
  double alt = static_cast<double>(data.pos_ins[2]) / INT_POS;

  auto toDegMin = [](double deg, char & hemi, bool isLon) -> std::string {
    hemi = (deg >= 0) ? (isLon ? 'E' : 'N') : (isLon ? 'W' : 'S');
    deg = fabs(deg);
    int d = static_cast<int>(deg);
    double m = (deg - d) * 60.0;
    char buf[32];
    if (isLon) {
      snprintf(buf, sizeof(buf), "%03d%09.6f", d, m);
    } else {
      snprintf(buf, sizeof(buf), "%02d%09.6f", d, m);
    }
    return std::string(buf);
  };

  char ns, ew;
  std::string lat_str = toDegMin(lat, ns, false);
  std::string lon_str = toDegMin(lng, ew, true);

  double total_sec = data.seconds / 10000.0;
  int hour = static_cast<int>(total_sec / 3600);
  int minute = static_cast<int>((total_sec - hour * 3600) / 60);
  double second = total_sec - hour * 3600 - minute * 60;
  char utc[16];
  snprintf(utc, sizeof(utc), "%02d%02d%05.2f", hour, minute, second);

  int quality = data.state;
  int sats = 0;
  double hdop = 0.0;
  double geoid = 0.0;

  char gga[256];
  snprintf(
    gga, sizeof(gga), "$GPGGA,%s,%s,%c,%s,%c,%d,%02d,%.1f,%.3f,M,%.1f,M,,", utc, lat_str.c_str(),
    ns, lon_str.c_str(), ew, quality, sats, hdop, alt, geoid);

  unsigned char cksum = 0;
  for (int i = 1; gga[i] != '\0'; ++i) {
    cksum ^= gga[i];
  }
  char nmea[300];
  snprintf(nmea, sizeof(nmea), "%s*%02X\r\n", gga, cksum);
  return std::string(nmea);
}

builtin_interfaces::msg::Time toRosTime(
  uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint32_t milliseconds)
{
  std::tm tm_zero{};
  tm_zero.tm_year = year - 1900;
  tm_zero.tm_mon = month - 1;
  tm_zero.tm_mday = day;
  tm_zero.tm_hour = 0;
  tm_zero.tm_min = 0;
  tm_zero.tm_sec = 0;

  std::time_t day_start = timegm(&tm_zero);
  std::time_t total_sec = day_start + hour * 3600 + minute * 60 + milliseconds / 1000;
  uint32_t nanosec = (milliseconds % 1000) * 1000000ULL;

  builtin_interfaces::msg::Time ros_time;
  ros_time.sec = static_cast<int32_t>(total_sec);
  ros_time.nanosec = nanosec;
  return ros_time;
}

double quaternionNorm(const double q[4])
{
  return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

geometry_msgs::msg::Quaternion attiToQuaternion(const int16_t atti[2], const uint16_t atti3)
{
  const double DEG_TO_RAD = M_PI / 180.0;

  // Protocol scale: [pitch, roll, yaw_north_cw]
  const double pitch = static_cast<double>(atti[0]) / INT_ATTI * DEG_TO_RAD;
  const double roll = static_cast<double>(atti[1]) / INT_ATTI * DEG_TO_RAD;
  const double yaw_north_cw = static_cast<double>(atti3) / INT_ATTI * DEG_TO_RAD;

  // Vendor intrinsic order y -> x -> -z, quaternion array [w, x, y, z]
  const double sina = std::sin(pitch * 0.5);
  const double sinb = std::sin(roll * 0.5);
  const double sinc = std::sin(yaw_north_cw * 0.5);
  const double cosa = std::cos(pitch * 0.5);
  const double cosb = std::cos(roll * 0.5);
  const double cosc = std::cos(yaw_north_cw * 0.5);

  double q_vendor_array[4];
  q_vendor_array[0] = cosa * cosb * cosc + sina * sinb * sinc;
  q_vendor_array[1] = sina * cosb * cosc + cosa * sinb * sinc;
  q_vendor_array[2] = cosa * sinb * cosc - sina * cosb * sinc;
  q_vendor_array[3] = sina * sinb * cosc - cosa * cosb * sinc;

  const double norm = quaternionNorm(q_vendor_array);
  if (norm > 1e-12) {
    q_vendor_array[0] /= norm;
    q_vendor_array[1] /= norm;
    q_vendor_array[2] /= norm;
    q_vendor_array[3] /= norm;
  }

  tf2::Quaternion q_vendor(
    q_vendor_array[1], q_vendor_array[2], q_vendor_array[3], q_vendor_array[0]);

  // North-CW -> ROS ENU East-CCW
  tf2::Quaternion q_ref;
  q_ref.setRotation(tf2::Vector3(0.0, 0.0, 1.0), M_PI_2);

  tf2::Quaternion q_ros = q_ref * q_vendor;
  q_ros.normalize();

  geometry_msgs::msg::Quaternion ros_q;
  ros_q.x = q_ros.x();
  ros_q.y = q_ros.y();
  ros_q.z = q_ros.z();
  ros_q.w = q_ros.w();
  return ros_q;
}

}  // namespace

namespace can_driver
{

CanReceiver::CanReceiver(std::shared_ptr<rclcpp::Node> node) : node_(node), running_(false)
{
  auto can_node = std::dynamic_pointer_cast<CanNode>(node_);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.best_effort();

  rtk_NavSatFix_publisher_ =
    node_->create_publisher<sensor_msgs::msg::NavSatFix>("/sensing/gnss/rtk/nav_sat_fix", qos);
  imu_publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>("/rtk_imu/data_raw", qos);
  rtk_GnssInsOrientationStamped_publisher_ =
    node_->create_publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(
      "/autoware_orientation", 10);
  rtk_velocity_twist_publisher_ =
    node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/sensing/gnss/rtk/velocity_twist", qos);
  rtk_velocity_vector_publisher_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/sensing/gnss/rtk/velocity_enu", qos);

  initRawLogging();
}

CanReceiver::~CanReceiver()
{
  running_ = false;
  nav_cv_.notify_all();
  gnss_cv_.notify_all();
  if (raw_log_file_.is_open()) {
    raw_log_file_.close();
  }
}

bool CanReceiver::isRunning() const
{
  return running_;
}

void CanReceiver::stop()
{
  running_ = false;
  nav_cv_.notify_all();
  gnss_cv_.notify_all();
}

bool CanReceiver::initRawLogging()
{
  // Ensure parent directory exists
  const auto slash = raw_log_basename_.find_last_of('/');
  if (slash != std::string::npos) {
    const std::string dir = raw_log_basename_.substr(0, slash);
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
  }

  std::string filename = raw_log_basename_ + ".log";
  std::ifstream infile(filename);
  std::vector<std::string> lines;

  if (infile.is_open()) {
    std::string line;
    while (std::getline(infile, line)) {
      lines.push_back(line);
    }
    infile.close();

    RCLCPP_INFO(node_->get_logger(), "已有日志 %zu 行", lines.size());

    if (lines.size() >= static_cast<size_t>(MAX_LOG_LINES)) {
      std::ofstream truncate_file(filename, std::ios::out | std::ios::trunc);
      if (!truncate_file.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "日志清理失败");
        return false;
      }
      for (size_t i = TRIM_LINES; i < lines.size(); ++i) {
        truncate_file << lines[i] << "\n";
      }
      truncate_file.close();
      current_log_lines_ = static_cast<int>(lines.size() - TRIM_LINES);
      RCLCPP_WARN(
        node_->get_logger(), "日志超过%d行，删除前%d行，剩余%d行", MAX_LOG_LINES, TRIM_LINES,
        current_log_lines_.load());
    } else {
      current_log_lines_ = static_cast<int>(lines.size());
    }
  } else {
    std::ofstream create_file(filename, std::ios::out);
    if (!create_file.is_open()) {
      RCLCPP_ERROR(node_->get_logger(), "无法创建日志文件 %s", filename.c_str());
      return false;
    }
    create_file.close();
    current_log_lines_ = 0;
    RCLCPP_INFO(node_->get_logger(), "创建新日志文件 %s", filename.c_str());
  }

  raw_log_file_.open(filename, std::ios::out | std::ios::app);
  if (!raw_log_file_.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "打开日志失败");
    return false;
  }
  return true;
}

void CanReceiver::writeRawFrame(const CanFrame & frame, const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(raw_log_mutex_);

  if (current_log_lines_ >= MAX_LOG_LINES) {
    raw_log_file_.close();
    std::string filename = raw_log_basename_ + ".log";
    std::ifstream infile(filename);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(infile, line)) {
      lines.push_back(line);
    }
    infile.close();

    std::ofstream outfile(filename, std::ios::out | std::ios::trunc);
    for (size_t i = TRIM_LINES; i < lines.size(); ++i) {
      outfile << lines[i] << "\n";
    }
    outfile.close();
    current_log_lines_ = static_cast<int>(lines.size() - TRIM_LINES);
    raw_log_file_.open(filename, std::ios::out | std::ios::app);
  }

  std::ostringstream line_stream;
  line_stream << stamp.seconds() << "." << std::setfill('0') << std::setw(9) << stamp.nanoseconds()
              << ", 0x" << std::hex << frame.frameId << std::dec << ", "
              << static_cast<int>(frame.dataLen);

  for (int i = 0; i < frame.dataLen; ++i) {
    line_stream << ", 0x" << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(frame.data[i]);
  }
  line_stream << "\n";

  raw_log_file_ << line_stream.str();
  current_log_lines_++;
  if (current_log_lines_ % 1000 == 0) {
    raw_log_file_.flush();
  }
}

void CanReceiver::receiveTask(int handle, const std::string & /*interface_name*/)
{
  running_ = true;
  struct canfd_frame frame{};

  while (running_ && rclcpp::ok()) {
    int nbytes = read(handle, &frame, sizeof(frame));
    if (nbytes <= 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    CanFrame can_frame;
    can_frame.frameId = frame.can_id;
    can_frame.dataLen = frame.len;
    std::memcpy(can_frame.data, frame.data, frame.len);
    writeRawFrame(can_frame, node_->now());

    if (frame.can_id == 0x603) {
      std::lock_guard<std::mutex> lock(nav_queue_mutex_);
      nav_queue_.push_back(can_frame);
      nav_cv_.notify_one();
    } else if (frame.can_id == 0x604) {
      std::lock_guard<std::mutex> lock(gnss_queue_mutex_);
      gnss_queue_.push_back(can_frame);
      gnss_cv_.notify_one();
    }
  }
}

void CanReceiver::publishNavSatFixTask()
{
  std::deque<CanFrame> frames_to_publish;
  auto last_status_time = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lock(nav_queue_mutex_);
      nav_cv_.wait(lock, [this] { return !nav_queue_.empty() || !running_; });
      if (!running_ && nav_queue_.empty()) {
        break;
      }
      nav_queue_.swap(frames_to_publish);
    }

    for (auto & frame : frames_to_publish) {
      ins_pos_can_t dataPos;
      if (frame.dataLen < sizeof(dataPos)) {
        continue;
      }
      std::memcpy(&dataPos, frame.data, sizeof(dataPos));

      sensor_msgs::msg::NavSatFix msg;
      auto stamp = toRosTime(
        dataPos.year, dataPos.month, dataPos.day, dataPos.hour, dataPos.minute, dataPos.seconds);
      msg.header.stamp = stamp;
      msg.header.frame_id = "base_link";
      msg.longitude = static_cast<double>(dataPos.pos_ins[0]) / INT_POS;
      msg.latitude = static_cast<double>(dataPos.pos_ins[1]) / INT_POS;
      msg.altitude = static_cast<double>(dataPos.pos_ins[2]) / INT_POS;
      msg.status.status = dataPos.state;

      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_status_time).count() >= 1) {
        last_status_time = now;
        if (dataPos.state == 4) {
          RCLCPP_INFO(node_->get_logger(), "RTK状态正常(state=%d)", dataPos.state);
        } else {
          RCLCPP_WARN(node_->get_logger(), "RTK状态异常(state=%d)", dataPos.state);
        }
      }

      if (g_six_sdk_running) {
        std::string gga_str = buildGGAString(dataPos);
        sixents_sdkSendGGAStr(gga_str.c_str(), gga_str.size());
      }

      if (rtk_NavSatFix_publisher_) {
        rtk_NavSatFix_publisher_->publish(msg);
      }

      autoware_sensing_msgs::msg::GnssInsOrientationStamped gnss_msg;
      gnss_msg.header.stamp = stamp;
      gnss_msg.header.frame_id = "imu_link";
      // Copy atti to avoid packed-member address warning
      int16_t atti_copy[2] = {dataPos.atti[0], dataPos.atti[1]};
      gnss_msg.orientation.orientation = attiToQuaternion(atti_copy, dataPos.atti3);
      if (rtk_GnssInsOrientationStamped_publisher_) {
        rtk_GnssInsOrientationStamped_publisher_->publish(gnss_msg);
      }

      const double vel_east = static_cast<double>(dataPos.vel[0]) / 1000.0;
      const double vel_north = static_cast<double>(dataPos.vel[1]) / 1000.0;
      const double vel_up = static_cast<double>(dataPos.vel[2]) / 1000.0;

      const double yaw = static_cast<double>(dataPos.atti3) / INT_ATTI * (M_PI / 180.0);
      const double vx = vel_east * std::sin(yaw) + vel_north * std::cos(yaw);
      const double vy = -vel_east * std::cos(yaw) + vel_north * std::sin(yaw);
      const double vz = vel_up;

      if (rtk_velocity_vector_publisher_) {
        geometry_msgs::msg::Vector3Stamped vel_vec_msg;
        vel_vec_msg.header.stamp = node_->now();
        vel_vec_msg.header.frame_id = "base_link";
        vel_vec_msg.vector.x = vx;
        vel_vec_msg.vector.y = vy;
        vel_vec_msg.vector.z = vz;
        rtk_velocity_vector_publisher_->publish(vel_vec_msg);
      }

      if (rtk_velocity_twist_publisher_) {
        geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
        twist_msg.header.stamp = node_->now();
        twist_msg.header.frame_id = "base_link";
        twist_msg.twist.twist.linear.x = vx;
        twist_msg.twist.twist.linear.y = vy;
        twist_msg.twist.twist.linear.z = vz;
        twist_msg.twist.covariance.fill(0.0);
        rtk_velocity_twist_publisher_->publish(twist_msg);
      }
    }
    frames_to_publish.clear();
  }
}

void CanReceiver::publishGnssInsTask()
{
  std::deque<CanFrame> frames_to_publish;

  while (rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lock(gnss_queue_mutex_);
      gnss_cv_.wait(lock, [this] { return !gnss_queue_.empty() || !running_; });
      if (!running_ && gnss_queue_.empty()) {
        break;
      }
      gnss_queue_.swap(frames_to_publish);
    }

    for (auto & frame : frames_to_publish) {
      ins_atti_can_t dataPos;
      if (frame.dataLen < sizeof(dataPos)) {
        continue;
      }
      std::memcpy(&dataPos, frame.data, sizeof(dataPos));

      sensor_msgs::msg::Imu imu_msg;
      imu_msg.header.stamp = node_->now();
      imu_msg.header.frame_id = "imu_link";
      imu_msg.orientation.x = 0.0;
      imu_msg.orientation.y = 0.0;
      imu_msg.orientation.z = 0.0;
      imu_msg.orientation.w = 1.0;
      imu_msg.orientation_covariance.fill(0.0);
      imu_msg.orientation_covariance[0] = -1.0;

      const double DEG_TO_RAD = M_PI / 180.0;
      imu_msg.angular_velocity.x =
        static_cast<double>(dataPos.gyro_xyz[1]) / INT_IMU * DEG_TO_RAD;
      imu_msg.angular_velocity.y =
        -static_cast<double>(dataPos.gyro_xyz[0]) / INT_IMU * DEG_TO_RAD;
      imu_msg.angular_velocity.z =
        static_cast<double>(dataPos.gyro_xyz[2]) / INT_IMU * DEG_TO_RAD;
      imu_msg.linear_acceleration.x = dataPos.acc_xyz[1] / acc_IMU * 9.8;
      imu_msg.linear_acceleration.y = -dataPos.acc_xyz[0] / acc_IMU * 9.8;
      imu_msg.linear_acceleration.z = dataPos.acc_xyz[2] / acc_IMU * 9.8;

      if (imu_publisher_) {
        imu_publisher_->publish(imu_msg);
      }
    }
    frames_to_publish.clear();
  }
}

}  // namespace can_driver
