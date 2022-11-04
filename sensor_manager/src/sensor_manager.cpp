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
: cyberdog::machine::MachineActuator(name),
  name_(name)
{
  node_ptr_ = rclcpp::Node::make_shared(name_);
  executor.add_node(node_ptr_);
  heart_beats_ptr_ = std::make_unique<cyberdog::machine::HeartBeatsActuator>("sensor");
  code_ptr_ = std::make_shared<cyberdog::system::CyberdogCode<SensorErrorCode>>(
    cyberdog::system::ModuleCode::kSensorManager);
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


  sensor_operation_srv_ = node_ptr_->create_service<protocol::srv::SensorOperation>(
    "sensor_operation",
    std::bind(
      &SensorManager::sensor_operation, this, std::placeholders::_1,
      std::placeholders::_2));

  INFO("sensor_manager Configuring,success");
}

bool cyberdog::sensor::SensorManager::Init()
{
  INFO("SensorManager Initing begin");

  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto path = local_share_dir + std::string("/toml_config/manager/state_machine_config.toml");
  if (!this->MachineActuatorInit(
      path,
      node_ptr_))
  {
    ERROR("Init failed, actuator init error.");
    return false;
  }
  this->RegisterStateCallback(SetUp_V, std::bind(&SensorManager::OnSetUp, this));
  this->RegisterStateCallback(TearDown_V, std::bind(&SensorManager::ONTearDown, this));
  this->RegisterStateCallback(SelfCheck_V, std::bind(&SensorManager::SelfCheck, this));
  this->RegisterStateCallback(Active_V, std::bind(&SensorManager::OnActive, this));
  this->RegisterStateCallback(DeActive_V, std::bind(&SensorManager::OnDeActive, this));
  this->RegisterStateCallback(Protected_V, std::bind(&SensorManager::OnProtected, this));
  this->RegisterStateCallback(LowPower_V, std::bind(&SensorManager::OnLowPower, this));
  this->RegisterStateCallback(OTA_V, std::bind(&SensorManager::OnOTA, this));
  this->RegisterStateCallback(Error_V, std::bind(&SensorManager::OnError, this));
  heart_beats_ptr_->HeartBeatRun();
  INFO(">>>init:heart run and state actuator start");
  return this->ActuatorStart();
}

void cyberdog::sensor::SensorManager::Run()
{
  executor.spin();
  rclcpp::shutdown();
}

int32_t cyberdog::sensor::SensorManager::SelfCheck()
{
  // check all sensors from config
  INFO("SensorManager SelfCheck begin");
  if (!this->lidar_->SelfCheck()) {
    ERROR("Lidar selfcheck fail.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kSelfCheckFailed);
  }
  INFO("Lidar selfcheck success.");
  if (!this->gps_->SelfCheck()) {
    ERROR("Gps selfcheck fail.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kSelfCheckFailed);
  }
  INFO("Gps selfcheck success.");
  if (!this->ultrasonic_->SelfCheck()) {
    ERROR("Ultrasonic selfcheck fail.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kSelfCheckFailed);
  }
  INFO("Ultrasonic selfcheck success.");
  if (!this->tof_->SelfCheck()) {
    ERROR("Tof selfcheck fail.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kSelfCheckFailed);
  }
  INFO("Tof selfcheck success.");
  return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}

bool cyberdog::sensor::SensorManager::IsStateValid()
{
  // check state from behavior tree
  return true;
}

template<typename T>
bool cyberdog::sensor::SensorManager::SensorOperation(
  T elem, uint8_t oper_id)
{
  bool result = false;
  switch (oper_id) {
    case protocol::srv::SensorOperation::Request::OPR_OPEN:
      {
        result = elem->Open();
      } break;
    case protocol::srv::SensorOperation::Request::OPR_START:
      {
        result = elem->Start();
      } break;
    case protocol::srv::SensorOperation::Request::OPR_STOP:
      {
        result = elem->Stop();
      } break;
    case protocol::srv::SensorOperation::Request::OPR_CLOSE:
      {
        result = elem->Close();
      } break;
    default:
      break;
  }
  return result;
}

int32_t cyberdog::sensor::SensorManager::OnError()
{
  return 0;
}

int32_t cyberdog::sensor::SensorManager::OnLowPower()
{
  return 0;
}

int32_t cyberdog::sensor::SensorManager::OnSuspend()
{
  return 0;
}

int32_t cyberdog::sensor::SensorManager::OnProtected()
{
  return 0;
}

int32_t cyberdog::sensor::SensorManager::OnActive()
{
  // ultrasonic_->Stop();
  // ultrasonic_->Close();
  // tof_->Stop();
  // tof_->Close();
  // INFO("SensorManager Running begin");
  // if (!this->lidar_->Start()) {
  //   ERROR("Lidar start fail.");
  //   return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kFailed);
  // }
  // INFO("Lidar start success.");
  // if (!this->gps_->Start()) {
  //   ERROR("Gps start fail.");
  //   return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kFailed);
  // }
  // INFO("Gps start success.");
  // if (!this->ultrasonic_->Start()) {
  //   ERROR("Ultrasonic start fail.");
  //   return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kFailed);
  // }
  // INFO("Ultrasonic start success.");
  // if (!this->tof_->Start()) {
  //   ERROR("Tof start fail.");
  //   return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kFailed);
  // }
  // INFO("Tof start success.");
  // INFO("Sensor manager start success.");
  return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}

int32_t cyberdog::sensor::SensorManager::OnDeActive()
{
  return 0;
}

int32_t cyberdog::sensor::SensorManager::OnSetUp()
{
  INFO("sensor on setup");
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
  if (!(ultrasonic_->Init(is_simulator("ultrasonic")) && ultrasonic_->Open())) {
    ERROR("Ultrasonic init or open fail.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kFailed);
  }
  if (!(tof_->Init(is_simulator("tof")) && tof_->Open())) {
    ERROR("Tof init or open fail.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kFailed);
  }
  if (!(lidar_->Init(is_simulator("lidar")) && lidar_->Open())) {
    ERROR("Lidar init or open fail.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kFailed);
  }
  if (!(gps_->Init(is_simulator("gps")) && gps_->Open())) {
    ERROR("Gps init or open fail.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kFailed);
  }
  INFO("SensorManager Running begin");
  if (!this->lidar_->Start()) {
    ERROR("Lidar start fail.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kFailed);
  }
  INFO("Lidar start success.");
  if (!this->gps_->Start()) {
    ERROR("Gps start fail.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kFailed);
  }
  INFO("Gps start success.");
  if (!this->ultrasonic_->Start()) {
    ERROR("Ultrasonic start fail.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kFailed);
  }
  INFO("Ultrasonic start success.");
  if (!this->tof_->Start()) {
    ERROR("Tof start fail.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kFailed);
  }
  INFO("Tof start success.");
  INFO("Sensor manager start success.");
  INFO("sensor setup success");
  return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}

int32_t cyberdog::sensor::SensorManager::ONTearDown()
{
  return 0;
}

int32_t cyberdog::sensor::SensorManager::OnOTA()
{
  return 0;
}

void cyberdog::sensor::SensorManager::gps_payload_callback(
  std::shared_ptr<protocol::msg::GpsPayload> msg)
{
  gps_publisher_->publish(*msg);
}

void cyberdog::sensor::SensorManager::lidar_payload_callback(
  std::shared_ptr<ScanMsg> msg_ptr)
{
  msg_ptr->header.stamp = this->node_ptr_->now();
  lidar_publisher_->publish(*msg_ptr);
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

void cyberdog::sensor::SensorManager::sensor_operation(
  const protocol::srv::SensorOperation::Request::SharedPtr request,
  protocol::srv::SensorOperation::Response::SharedPtr response)
{
  auto sensor_tuple = std::make_tuple(lidar_, ultrasonic_, tof_, gps_);
  response->success = true;
  switch (request->sensor_id) {
    case protocol::srv::SensorOperation::Request::ID_ALL:
      {
        response->success &=
          SensorOperation<decltype(std::get<protocol::srv::SensorOperation::Request::ID_LIDAR -
            1>(sensor_tuple))>(
          std::get<protocol::srv::SensorOperation::Request::ID_LIDAR -
          1>(sensor_tuple), request->operation);
        response->success &=
          SensorOperation<decltype(std::get<protocol::srv::SensorOperation::Request::ID_ULTRA -
            1>(sensor_tuple))>(
          std::get<protocol::srv::SensorOperation::Request::ID_ULTRA -
          1>(sensor_tuple), request->operation);
        response->success &=
          SensorOperation<decltype(std::get<protocol::srv::SensorOperation::Request::ID_TOF -
            1>(sensor_tuple))>(
          std::get<protocol::srv::SensorOperation::Request::ID_TOF -
          1>(sensor_tuple), request->operation);
        response->success &=
          SensorOperation<decltype(std::get<protocol::srv::SensorOperation::Request::ID_GPS -
            1>(sensor_tuple))>(
          std::get<protocol::srv::SensorOperation::Request::ID_GPS -
          1>(sensor_tuple), request->operation);
      } break;

    case protocol::srv::SensorOperation::Request::ID_LIDAR:
      {
        response->success =
          SensorOperation<decltype(std::get<protocol::srv::SensorOperation::Request::ID_LIDAR -
            1>(sensor_tuple))>(
          std::get<protocol::srv::SensorOperation::Request::ID_LIDAR -
          1>(sensor_tuple), request->operation);
      } break;

    case protocol::srv::SensorOperation::Request::ID_ULTRA:
      {
        response->success =
          SensorOperation<decltype(std::get<protocol::srv::SensorOperation::Request::ID_ULTRA -
            1>(sensor_tuple))>(
          std::get<protocol::srv::SensorOperation::Request::ID_ULTRA -
          1>(sensor_tuple), request->operation);
      } break;

    case protocol::srv::SensorOperation::Request::ID_TOF:
      {
        response->success =
          SensorOperation<decltype(std::get<protocol::srv::SensorOperation::Request::ID_TOF -
            1>(sensor_tuple))>(
          std::get<protocol::srv::SensorOperation::Request::ID_TOF -
          1>(sensor_tuple), request->operation);
      } break;

    case protocol::srv::SensorOperation::Request::ID_GPS:
      {
        response->success =
          SensorOperation<decltype(std::get<protocol::srv::SensorOperation::Request::ID_GPS -
            1>(sensor_tuple))>(
          std::get<protocol::srv::SensorOperation::Request::ID_GPS -
          1>(sensor_tuple), request->operation);
      } break;

    default:
      {
        response->success = false;
      } break;
  }
}
