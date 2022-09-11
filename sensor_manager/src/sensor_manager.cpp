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

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "sensor_manager/sensor_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

cyberdog::sensor::SensorManager::SensorManager(const std::string & name)
: manager::ManagerBase(name),
  name_(name)
{
  node_ptr_ = rclcpp::Node::make_shared(name_);
}

cyberdog::sensor::SensorManager::~SensorManager()
{}

void cyberdog::sensor::SensorManager::Config()
{
  INFO("sensor manager Configuring begin");

  // gps
  INFO("gps Configuring beginning");
  gps_publisher_ = node_ptr_->create_publisher<protocol::msg::GpsPayload>(
    "gps_payload",
    rclcpp::SystemDefaultsQoS());
  std::shared_ptr<pluginlib::ClassLoader<cyberdog::sensor::GpsBase>> gps_classloader;
  gps_classloader = std::make_shared<pluginlib::ClassLoader<cyberdog::sensor::GpsBase>>(
    "cyberdog_gps", "cyberdog::sensor::GpsBase");
  gps_ = gps_classloader->createSharedInstance("cyberdog::sensor::GpsCarpo");
  gps_->SetPayloadCallback(
    std::bind(
      &SensorManager::gps_payload_callback, this,
      std::placeholders::_1));

  // lidar
  INFO("sensor manager Configuring begin");
  INFO("lidar Configuring beginning");
  lidar_publisher_ = node_ptr_->create_publisher<ScanMsg>(
    "scan",
    rclcpp::SystemDefaultsQoS());
  std::shared_ptr<pluginlib::ClassLoader<cyberdog::sensor::LidarBase>> lidar_classloader;
  lidar_classloader = std::make_shared<pluginlib::ClassLoader<cyberdog::sensor::LidarBase>>(
    "cyberdog_lidar", "cyberdog::sensor::LidarBase");
  lidar_ = lidar_classloader->createSharedInstance("cyberdog::sensor::YdlidarCarpo");
  lidar_->SetPayloadCallback(
    std::bind(
      &SensorManager::lidar_payload_callback, this,
      std::placeholders::_1));

  // ultrasonic
  INFO("ultrasonic Configuring beginning");
  ultrasonic_publisher_ = node_ptr_->create_publisher<sensor_msgs::msg::Range>(
    "ultrasonic_payload", rclcpp::SystemDefaultsQoS());
  std::shared_ptr<pluginlib::ClassLoader<cyberdog::sensor::UltrasonicBase>> ultrasonic_classloader;

  ultrasonic_classloader =
    std::make_shared<pluginlib::ClassLoader<cyberdog::sensor::UltrasonicBase>>(
    "cyberdog_ultrasonic", "cyberdog::sensor::UltrasonicBase");
  ultrasonic_ = ultrasonic_classloader->createSharedInstance("cyberdog::sensor::UltrasonicCarpo");
  ultrasonic_->SetPayloadCallback(
    std::bind(
      &SensorManager::ultrasonic_payload_callback, this,
      std::placeholders::_1));

  // tof
  INFO("tof Configuring beginning");
  head_tof_publisher_ = node_ptr_->create_publisher<protocol::msg::HeadTofPayload>(
    "head_tof_payload", rclcpp::SystemDefaultsQoS());
  rear_tof_publisher_ = node_ptr_->create_publisher<protocol::msg::RearTofPayload>(
    "rear_tof_payload", rclcpp::SystemDefaultsQoS());
  std::shared_ptr<pluginlib::ClassLoader<cyberdog::sensor::TofBase>> tof_classloader;
  tof_classloader = std::make_shared<pluginlib::ClassLoader<cyberdog::sensor::TofBase>>(
    "cyberdog_tof", "cyberdog::sensor::TofBase");
  tof_ = tof_classloader->createSharedInstance("cyberdog::sensor::TofCarpo");
  tof_->SetHeadPayloadCallback(
    std::bind(
      &SensorManager::head_tof_payload_callback, this,
      std::placeholders::_1));
  tof_->SetRearPayloadCallback(
    std::bind(
      &SensorManager::rear_tof_payload_callback, this,
      std::placeholders::_1));

  INFO("sensor_manager Configuring,success");
}

bool cyberdog::sensor::SensorManager::Init()
{
  INFO("SensorManager Initing begin");

  this->node_ptr_->declare_parameter("simulator", std::vector<std::string>{});
  this->node_ptr_->get_parameter("simulator", this->simulator_);
  auto is_simulator = [this](std::string sensor_name) -> bool {
      return static_cast<bool>(std::find(
               this->simulator_.begin(), this->simulator_.end(),
               sensor_name) != this->simulator_.end());
    };
  INFO("gps is_simulator %d", is_simulator("gps"));
  INFO("ultrasonic_ is_simulator %d", is_simulator("ultrasonic"));
  INFO("tof_ is_simulator %d", is_simulator("tof"));
  INFO("lidar_ is_simulator %d", is_simulator("lidar"));
  return bool(
    gps_->Init(1) && gps_->Open() &&
    ultrasonic_->Init(1) && ultrasonic_->Open() &&
    tof_->Init(is_simulator("tof")) && tof_->Open() &&
    lidar_->Init(is_simulator("tof")) && lidar_->Open()
  );
}

void cyberdog::sensor::SensorManager::Run()
{
  INFO("SensorManager Running begin");
  if (this->lidar_->Start() &&
    this->gps_->Start() &&
    this->ultrasonic_->Start() &&
    this->tof_->Start())
  {
    rclcpp::spin(node_ptr_);
  }
  rclcpp::shutdown();
}

bool cyberdog::sensor::SensorManager::SelfCheck()
{
  // check all sensors from config
  return true;
}

bool cyberdog::sensor::SensorManager::IsStateValid()
{
  // check state from behavior tree
  return true;
}

void cyberdog::sensor::SensorManager::OnError()
{
}

void cyberdog::sensor::SensorManager::OnLowPower()
{
}

void cyberdog::sensor::SensorManager::OnSuspend()
{
}

void cyberdog::sensor::SensorManager::OnProtected()
{
}

void cyberdog::sensor::SensorManager::OnActive()
{
  ultrasonic_->Stop();
  ultrasonic_->Close();
  tof_->Stop();
  tof_->Close();
}

void cyberdog::sensor::SensorManager::gps_payload_callback(
  std::shared_ptr<protocol::msg::GpsPayload> msg)
{
  gps_publisher_->publish(*msg);
}

void cyberdog::sensor::SensorManager::lidar_payload_callback(
  std::shared_ptr<ScanMsg> msg)
{
  lidar_publisher_->publish(*msg);
}

void cyberdog::sensor::SensorManager::ultrasonic_payload_callback(
  std::shared_ptr<sensor_msgs::msg::Range> msg)
{
  ultrasonic_publisher_->publish(*msg);
}

void cyberdog::sensor::SensorManager::head_tof_payload_callback(
  std::shared_ptr<protocol::msg::HeadTofPayload> msg)
{
  head_tof_publisher_->publish(*msg);
}

void cyberdog::sensor::SensorManager::rear_tof_payload_callback(
  std::shared_ptr<protocol::msg::RearTofPayload> msg)
{
  rear_tof_publisher_->publish(*msg);
}
