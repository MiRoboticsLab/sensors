// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#ifndef SENSOR_MANAGER__SENSOR_MANAGER_HPP_
#define SENSOR_MANAGER__SENSOR_MANAGER_HPP_
#include <sensor_msgs/msg/range.hpp>
#include <functional>
#include <mutex>
#include <thread>
#include <string>
#include <memory>
#include "pluginlib/class_loader.hpp"
#include "bcmgps_base/bcmgps_base.hpp"
#include "protocol/msg/gps_payload.hpp"
#include "lidar_base/lidar_base.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "manager_base/manager_base.hpp"
#include "ultrasonic_base/ultrasonic_base.hpp"
#include "tof_base/tof_base.hpp"
#include "protocol/msg/single_tof_payload.hpp"
#include "protocol/msg/multiple_tof_payload.hpp"
#include "rclcpp/rclcpp.hpp"


namespace cyberdog
{
namespace sensor
{
class SensorManager final : public manager::ManagerBase
{
public:
  explicit SensorManager(const std::string & name);
  ~SensorManager();

  void Config() override;
  bool Init() override;
  void Run() override;
  bool SelfCheck() override;

public:
  void OnError() override;
  void OnLowPower() override;
  void OnSuspend() override;
  void OnProtected() override;
  void OnActive() override;

private:
  bool IsStateValid();

private:
  std::string name_;
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  void gps_payload_callback(std::shared_ptr<protocol::msg::GpsPayload> msg);
  void lidar_payload_callback(std::shared_ptr<sensor_msgs::msg::LaserScan> msg);
  void ultrasonic_payload_callback(std::shared_ptr<sensor_msgs::msg::Range> msg);
  void tof_payload_callback(std::shared_ptr<protocol::msg::MultipleTofPayload> msg);

private:
  std::shared_ptr<cyberdog::sensor::GpsBase> gps_;
  std::shared_ptr<cyberdog::sensor::LidarBase> lidar_;
  rclcpp::Publisher<protocol::msg::GpsPayload>::SharedPtr gps_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;

  std::shared_ptr<cyberdog::sensor::UltrasonicBase> ultrasonic_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_publisher_;

  std::shared_ptr<cyberdog::sensor::TofBase> tof_;
  rclcpp::Publisher<protocol::msg::MultipleTofPayload>::SharedPtr tof_publisher_;
};  // class SensorManager
}  // namespace sensor
}  // namespace cyberdog

#endif  // SENSOR_MANAGER__SENSOR_MANAGER_HPP_
