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
#include <memory>
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
  // gps
  INFO("sensor manager Configuring begin");
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
  /*
  // lidar
  INFO("sensor manager Configuring begin");
  INFO("lidar Configuring beginning");
  lidar_publisher_ = node_ptr_->create_publisher<sensor_msgs::msg::LaserScan>(
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
  */
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
  tof_publisher_ = node_ptr_->create_publisher<protocol::msg::MultipleTofPayload>(
    "tof_payload", rclcpp::SystemDefaultsQoS());
  std::shared_ptr<pluginlib::ClassLoader<cyberdog::sensor::TofBase>> tof_classloader;
  tof_classloader = std::make_shared<pluginlib::ClassLoader<cyberdog::sensor::TofBase>>(
    "cyberdog_tof", "cyberdog::sensor::TofBase");
  tof_ = tof_classloader->createSharedInstance("cyberdog::sensor::TofCarpo");
  tof_->SetPayloadCallback(
    std::bind(
      &SensorManager::tof_payload_callback, this,
      std::placeholders::_1));
  INFO("sensor_manager Configuring,success");
}

bool cyberdog::sensor::SensorManager::Init()
{
  // register manager base functions
  INFO("SensorManager Initing begin");
  INFO("gps open beginning");
  bool gps_opened = gps_->Open();
  // INFO("lidar open beginning");
  // bool lidar_opened = lidar_->Open();
  INFO("ultrasonic open beginning");
  bool ultrasonic_opened = ultrasonic_->Open();
  INFO("tof open beginning");
  bool tof_opened = tof_->Open();
  bool init_result = gps_opened && ultrasonic_opened && tof_opened;
  (void)init_result;


  // bool init_result = gps_opened && lidar_opened && ultrasonic_opened && tof_opened;
  return true;
}

void cyberdog::sensor::SensorManager::Run()
{
  INFO("SensorManager Running begin");
  INFO("gps start beginning");
  bool gps_started = gps_->Start();
  // INFO("lidar start beginning");
  // bool lidar_started = lidar_->Start();
  INFO("ultrasonic start beginning");
  bool ultrasonic_started = ultrasonic_->Start();
  INFO("tof start beginning");
  bool tof_started = tof_->Start();
  (void)tof_started;
  (void)ultrasonic_started;
  (void)gps_started;
  rclcpp::spin(node_ptr_);
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
  INFO("hello_gps");
}

void cyberdog::sensor::SensorManager::lidar_payload_callback(
  std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
{
  lidar_publisher_->publish(*msg);
  INFO("hello_lidar");
}

void cyberdog::sensor::SensorManager::ultrasonic_payload_callback(
  std::shared_ptr<sensor_msgs::msg::Range> msg)
{
  ultrasonic_publisher_->publish(*msg);
  INFO("hello_ultrasonic");
}

void cyberdog::sensor::SensorManager::tof_payload_callback(
  std::shared_ptr<protocol::msg::MultipleTofPayload> msg)
{
  tof_publisher_->publish(*msg);
  INFO("hello_tof");
}
