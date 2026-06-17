/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Copyright (c) 2019, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 * tag_serial_driver.cpp
 * Tamagawa IMU Driver
 * Author MapIV Sekino
 * Ver 1.00 2019/4/4
 */

#include <cmath>
#include <string>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <iostream>

#include <boost/asio.hpp>

#ifdef USE_AGNOCAST_ENABLED
#include <agnocast/agnocast_callback_isolated_executor.hpp>
#include <agnocast_cie_thread_configurator/cie_thread_configurator.hpp>
#endif

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include "rate_bound_status.hpp"


using namespace boost::asio;


std::shared_ptr<custom_diagnostic_tasks::RateBoundStatus> rate_bound_status;
std::unique_ptr<diagnostic_updater::Updater> diag_updater;

io_service io;
std::shared_ptr<serial_port> g_serial_port;

void stop_io()
{
  g_serial_port->cancel();
  io.stop();
}

int restart_io()
{
  if (!rclcpp::ok()) {
    return -1;
  }
  io.restart();
  return 0;
}

void loop_process(
  std::string imu_frame_id,
  rclcpp::Node::SharedPtr node,
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub,
  int timeout
)
{
  boost::system::error_code timer_result;
  boost::system::error_code read_result;
  std::size_t bytes_transferred = 0;
  int raw_data;
  std::string rbuf;
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;

  while (rclcpp::ok()) {
    boost::asio::streambuf response;
    read_result = boost::asio::error::would_block;
    bytes_transferred = 0;

    boost::asio::async_read_until(*g_serial_port, response, "\n",
      [&](const boost::system::error_code& ec, std::size_t size) {
        read_result = ec;
        bytes_transferred = size;
      });

    if (restart_io() < 0) return;
    io.run_for(std::chrono::milliseconds(timeout));

    if (bytes_transferred > 0 && read_result == boost::system::errc::success) {
      rbuf = std::string(
        boost::asio::buffers_begin(response.data()), boost::asio::buffers_end(response.data()));
      if (rbuf[5] == 'B' && rbuf[6] == 'I' && rbuf[7] == 'N' && rbuf[8] == ',' && bytes_transferred == 58) {
        imu_msg.header.frame_id = imu_frame_id;

        raw_data = ((((rbuf[15] << 8) & 0xFFFFFF00) | (rbuf[16] & 0x000000FF)));
        imu_msg.angular_velocity.x =
          raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
        raw_data = ((((rbuf[17] << 8) & 0xFFFFFF00) | (rbuf[18] & 0x000000FF)));
        imu_msg.angular_velocity.y =
          raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
        raw_data = ((((rbuf[19] << 8) & 0xFFFFFF00) | (rbuf[20] & 0x000000FF)));
        imu_msg.angular_velocity.z =
          raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
        raw_data = ((((rbuf[21] << 8) & 0xFFFFFF00) | (rbuf[22] & 0x000000FF)));
        imu_msg.linear_acceleration.x = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
        raw_data = ((((rbuf[23] << 8) & 0xFFFFFF00) | (rbuf[24] & 0x000000FF)));
        imu_msg.linear_acceleration.y = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
        raw_data = ((((rbuf[25] << 8) & 0xFFFFFF00) | (rbuf[26] & 0x000000FF)));
        imu_msg.linear_acceleration.z = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]

        if (rclcpp::ok()) {
          imu_msg.header.stamp = node->now();
          pub->publish(imu_msg);
          rate_bound_status->tick();
        }
      } else if (rbuf[5] == 'V' && rbuf[6] == 'E' && rbuf[7] == 'R' && rbuf[8] == ',') {
        RCLCPP_DEBUG(rclcpp::get_logger("tag_serial_driver"), "%s", rbuf.c_str());
      }
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tag_serial_driver");
  auto pub = node->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1000);

  std::string imu_frame_id = node->declare_parameter<std::string>("imu_frame_id", "imu");
  std::string port = node->declare_parameter<std::string>("port", "/dev/ttyUSB0");

  g_serial_port = std::make_shared<serial_port>(io);
  try {
    g_serial_port->open(port);
    g_serial_port->set_option(serial_port_base::baud_rate(115200));
    g_serial_port->set_option(serial_port_base::character_size(8));
    g_serial_port->set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
    g_serial_port->set_option(serial_port_base::parity(serial_port_base::parity::none));
    g_serial_port->set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
  } catch (boost::system::system_error &e) {
    RCLCPP_ERROR(rclcpp::get_logger("tag_serial_driver"), "Error opening serial port: %s", e.what());
    return 1;
  }

  std::string wbuf = "$TSC,BIN,30\x0d\x0a";
  g_serial_port->write_some(buffer(wbuf));

  auto frequency_reference = node->declare_parameter<double>("frequency_reference", 200.0);
  auto ok_min_freq = node->declare_parameter<double>(
    "diagnostics.rate_bound_status.frequency_ok.min_hz", 100.0);
  auto ok_max_freq = node->declare_parameter<double>(
    "diagnostics.rate_bound_status.frequency_ok.max_hz", 10000.0);
  auto warn_min_freq = node->declare_parameter<double>(
    "diagnostics.rate_bound_status.frequency_warn.min_hz", 50.0);
  auto warn_max_freq = node->declare_parameter<double>(
    "diagnostics.rate_bound_status.frequency_warn.max_hz", 100000.0);
  node->declare_parameter<bool>("diagnostic_updater.use_fqn", true);  // read by diagnostic updater
  rate_bound_status = std::make_shared<custom_diagnostic_tasks::RateBoundStatus>(
    node.get(), custom_diagnostic_tasks::RateBoundStatusParam(ok_min_freq, ok_max_freq),
    custom_diagnostic_tasks::RateBoundStatusParam(warn_min_freq, warn_max_freq), 2, false);
  diag_updater = std::make_unique<diagnostic_updater::Updater>(node);
  diag_updater->setHardwareID(imu_frame_id);
  diag_updater->setPeriod(1.0 / frequency_reference);
  diag_updater->add(*rate_bound_status);
  const int timeout = static_cast<int>(1000.0 / (warn_min_freq * 0.1));  // ms

#ifdef USE_AGNOCAST_ENABLED
  const std::string thread_name = "tag_serial_driver:" + port + "_loop_thread";
  std::thread loop_thread = agnocast_cie_thread_configurator::spawn_non_ros2_thread(
    thread_name.c_str(), loop_process, imu_frame_id, node, pub, timeout);
  auto executor = std::make_shared<agnocast::CallbackIsolatedAgnocastExecutor>();
  executor->add_node(node);
  executor->spin();
#else
  std::thread loop_thread(loop_process, imu_frame_id, node, pub, timeout);
  rclcpp::spin(node);
#endif

  stop_io();

  if (loop_thread.joinable()) {
    loop_thread.join();
  }

  return 0;
}
