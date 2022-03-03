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

  // ultrasonic
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

  tof_publisher_ = node_ptr_->create_publisher<protocol::msg::MultipleTof>(
    "tof_payload", rclcpp::SystemDefaultsQoS());
  std::shared_ptr<pluginlib::ClassLoader<cyberdog::sensor::TofBase>> tof_classloader;
  tof_classloader = std::make_shared<pluginlib::ClassLoader<cyberdog::sensor::TofBase>>(
    "cyberdog_tof", "cyberdog::sensor::TofBase");
  tof_ = tof_classloader->createSharedInstance("cyberdog::sensor::TofCarpo");
  tof_->SetPayloadCallback(
    std::bind(
      &SensorManager::tof_payload_callback, this,
      std::placeholders::_1));

  RCLCPP_INFO(rclcpp::get_logger("sensor_manager"), "Configuring,success");
}

bool cyberdog::sensor::SensorManager::Init()
{
  // register manager base functions
  bool gps_opened = gps_->Open();
  bool ultrasonic_opened = ultrasonic_->Open();
  bool tof_opened = tof_->Open();
  bool init_result = gps_opened && ultrasonic_opened && tof_opened;
  return true;
}

void cyberdog::sensor::SensorManager::Run()
{
  bool gps_started = gps_->Start();
  bool ultrasonic_started = ultrasonic_->Start();
  bool tof_started = tof_->Start();
  bool run_result = gps_started && ultrasonic_started && tof_started;
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
  std::cout << "on error\n";
}

void cyberdog::sensor::SensorManager::OnLowPower()
{
  std::cout << "on lowpower\n";
}

void cyberdog::sensor::SensorManager::OnSuspend()
{
  std::cout << "on suspend\n";
}

void cyberdog::sensor::SensorManager::OnProtected()
{
  std::cout << "on protect\n";
}

void cyberdog::sensor::SensorManager::OnActive()
{
  std::cout << "on active\n";
}

void cyberdog::sensor::SensorManager::gps_payload_callback(
  std::shared_ptr<protocol::msg::GpsPayload> msg)
{
  gps_publisher_->publish(*msg);
  std::cout << "hello_gps " << std::endl;
}


void cyberdog::sensor::SensorManager::ultrasonic_payload_callback(
  std::shared_ptr<sensor_msgs::msg::Range> msg)
{
  ultrasonic_publisher_->publish(*msg);
  std::cout << "hello_ultrasonic" << std::endl;
}

void cyberdog::sensor::SensorManager::tof_payload_callback(
  std::shared_ptr<protocol::msg::MultipleTof> msg)
{
  tof_publisher_->publish(*msg);
  std::cout << "hello_tof" << std::endl;
}
