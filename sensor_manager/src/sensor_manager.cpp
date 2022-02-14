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
  // TODO: get info from configure
}

bool cyberdog::sensor::SensorManager::Init()
{
  // TODO: register manager base functions
  return true;
}

void cyberdog::sensor::SensorManager::Run()
{
  rclcpp::spin(node_ptr_);
  rclcpp::shutdown();
}

bool cyberdog::sensor::SensorManager::SelfCheck()
{
  // TODO: check all sensors from config
  return true;
}

bool cyberdog::sensor::SensorManager::IsStateValid()
{
  // TODO: check state from behavior tree
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
