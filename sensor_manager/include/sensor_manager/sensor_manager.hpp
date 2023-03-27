// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#include <vector>
#include <memory>
#include <unordered_map>
#include <future>
#include "pluginlib/class_loader.hpp"
#include "bcmgps_base/bcmgps_base.hpp"
#include "protocol/msg/gps_payload.hpp"
#include "lidar_base/lidar_base.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "manager_base/manager_base.hpp"
#include "ultrasonic_base/ultrasonic_base.hpp"
#include "tof_base/tof_base.hpp"
#include "protocol/msg/single_tof_payload.hpp"
#include "protocol/msg/head_tof_payload.hpp"
#include "protocol/msg/rear_tof_payload.hpp"
#include "rclcpp/rclcpp.hpp"
#include "protocol/srv/sensor_operation.hpp"
#include "cyberdog_machine/cyberdog_fs_machine.hpp"
#include "cyberdog_machine/cyberdog_heartbeats.hpp"
#include "sensor_manager/self_check.hpp"

namespace cyberdog
{
namespace sensor
{
namespace SYS = cyberdog::system;

enum class SensorErrorCode : int32_t
{
  kDemoError1 = 21,
  kDemoError2 = 22,
  kDemoError3 = 23
};

class SensorManager final : public cyberdog::machine::MachineActuator
{
  using ScanMsg = sensor_msgs::msg::LaserScan;                        // [topic 类型]激光数据

public:
  explicit SensorManager(const std::string & name);
  ~SensorManager();

  void Config();
  bool Init();
  void Run();
  int32_t SelfCheck();

public:
  int32_t OnError();
  int32_t OnLowPower();
  int32_t OnSuspend();
  int32_t OnProtected();
  int32_t OnActive();
  int32_t OnDeActive();
  int32_t OnSetUp();
  int32_t ONTearDown();
  int32_t OnOTA();

private:
  bool IsStateValid();

  template<typename T>
  bool SensorOperation(T elem, uint8_t oper_id);

private:
  std::string name_;
  std::vector<std::string> simulator_;
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  std::unique_ptr<cyberdog::machine::HeartBeatsActuator> heart_beats_ptr_ {nullptr};
  std::shared_ptr<SYS::CyberdogCode<SensorErrorCode>> code_ptr_ {nullptr};
  std::unique_ptr<SensorSelfCheck> sensor_self_check_ptr {nullptr};
  rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};

private:
  std::shared_ptr<GpsBase> gps_;
  rclcpp::Publisher<protocol::msg::GpsPayload>::SharedPtr gps_publisher_;
  void gps_payload_callback(std::shared_ptr<protocol::msg::GpsPayload> msg);

  std::shared_ptr<LidarBase> lidar_;
  rclcpp::Publisher<ScanMsg>::SharedPtr lidar_publisher_;
  void lidar_payload_callback(std::shared_ptr<ScanMsg> msg);

  std::shared_ptr<UltrasonicBase> ultrasonic_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_publisher_;
  void ultrasonic_payload_callback(std::shared_ptr<sensor_msgs::msg::Range> msg);
  std::mutex ultrasonic_lock_;

  std::shared_ptr<TofBase> tof_;
  rclcpp::Publisher<protocol::msg::HeadTofPayload>::SharedPtr head_tof_publisher_;
  rclcpp::Publisher<protocol::msg::RearTofPayload>::SharedPtr rear_tof_publisher_;
  std::shared_ptr<protocol::msg::HeadTofPayload> head_tof_payload;
  std::shared_ptr<protocol::msg::RearTofPayload> rear_tof_payload;
  std::unordered_map<std::string, std::atomic<bool>> head_tof_;
  std::unordered_map<std::string, std::atomic<bool>> rear_tof_;
  std::mutex head_tof_lock_;
  std::mutex rear_tof_lock_;
  void SingleTofPayloadCallback(std::shared_ptr<protocol::msg::SingleTofPayload> msg);

  rclcpp::Service<protocol::srv::SensorOperation>::SharedPtr sensor_operation_srv_;
  void sensor_operation(
    const protocol::srv::SensorOperation::Request::SharedPtr request,
    protocol::srv::SensorOperation::Response::SharedPtr response);

private:
  const std::string Uninitialized_V = std::string("Uninitialized");
  const std::string SetUp_V = std::string("SetUp");
  const std::string TearDown_V = std::string("TearDown");
  const std::string SelfCheck_V = std::string("SelfCheck");
  const std::string Active_V = std::string("Active");
  const std::string DeActive_V = std::string("DeActive");
  const std::string Protected_V = std::string("Protected");
  const std::string LowPower_V = std::string("LowPower");
  const std::string OTA_V = std::string("OTA");
  const std::string Error_V = std::string("Error");
};  // class SensorManager
}  // namespace sensor
}  // namespace cyberdog

#endif  // SENSOR_MANAGER__SENSOR_MANAGER_HPP_
