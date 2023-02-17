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

#define IS_OK(code)  ((code & 0xFF) ? false : true)

cyberdog::sensor::SensorManager::SensorManager(const std::string & name)
: cyberdog::machine::MachineActuator(name),
  name_(name)
{
  node_ptr_ = rclcpp::Node::make_shared(name_);
  executor.add_node(node_ptr_);
  heart_beats_ptr_ = std::make_unique<cyberdog::machine::HeartBeatsActuator>("sensor");
  code_ptr_ = std::make_shared<SYS::CyberdogCode<SensorErrorCode>>(
    SYS::ModuleCode::kSensorManager);
  sensor_self_check_ptr = std::make_unique<SensorSelfCheck>();
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
  ultrasonic_->SetSinglePayloadCallback(
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
  head_tof_["left_head_tof"] = false;
  head_tof_["right_head_tof"] = false;
  rear_tof_["left_rear_tof"] = false;
  rear_tof_["right_rear_tof"] = false;
  head_tof_payload = std::make_shared<protocol::msg::HeadTofPayload>();
  rear_tof_payload = std::make_shared<protocol::msg::RearTofPayload>();
  tof_->SetSinglePayloadCallback(
    std::bind(
      &SensorManager::SingleTofPayloadCallback, this,
      std::placeholders::_1));

  callback_group_ =
    node_ptr_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  sensor_operation_srv_ = node_ptr_->create_service<protocol::srv::SensorOperation>(
    "sensor_operation",
    std::bind(
      &SensorManager::sensor_operation, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);

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
  int32_t return_code = code_ptr_->GetKeyCode(SYS::KeyCode::kOK);
  try {
    // check all sensors from config
    INFO("SensorManager SelfCheck begin");
    return_code = this->lidar_->SelfCheck();
    if (!IS_OK(return_code)) {
      ERROR("Lidar selfcheck fail.");
      if (!sensor_self_check_ptr->IsJump("lidar")) {
        return return_code;
      } else {
        INFO("Jump lidar selfcheck error.");
      }
    } else {
      INFO("Lidar selfcheck success.");
    }

    return_code = this->gps_->SelfCheck();
    if (!IS_OK(return_code)) {
      ERROR("Gps selfcheck fail.");
      if (!sensor_self_check_ptr->IsJump("gps")) {
        return return_code;
      } else {
        INFO("Jump gps selfcheck error.");
      }
    } else {
      INFO("Gps selfcheck success.");
    }

    return_code = this->ultrasonic_->SelfCheck();
    if (!IS_OK(return_code)) {
      ERROR("Ultrasonic selfcheck fail.");
      if (!sensor_self_check_ptr->IsJump("ultrasonic")) {
        return return_code;
      } else {
        INFO("Jump ultrasonic selfcheck error.");
      }
    } else {
      INFO("Ultrasonic selfcheck success.");
    }

    return_code = this->tof_->SelfCheck();
    if (!IS_OK(return_code)) {
      ERROR("Tof selfcheck fail.");
      if (!sensor_self_check_ptr->IsJump("tof")) {
        return return_code;
      } else {
        INFO("Jump tof selfcheck error.");
      }
    } else {
      INFO("Tof selfcheck success.");
    }
  } catch (const std::bad_function_call & e) {
    ERROR("bad function:%s", e.what());
    return code_ptr_->GetKeyCode(SYS::KeyCode::kSelfCheckFailed);
  } catch (const std::exception & e) {
    ERROR("exception:%s", e.what());
    return code_ptr_->GetKeyCode(SYS::KeyCode::kSelfCheckFailed);
  } catch (...) {
    ERROR("self check unkown exception!");
    return code_ptr_->GetKeyCode(SYS::KeyCode::kSelfCheckFailed);
  }
  return code_ptr_->GetKeyCode(SYS::KeyCode::kOK);
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
  int32_t code;
  switch (oper_id) {
    case protocol::srv::SensorOperation::Request::OPR_OPEN:
      {
        code = elem->Open();
        result = (IS_OK(code) > 0) ? false : true;
        break;
      }
    case protocol::srv::SensorOperation::Request::OPR_START:
      {
        code = elem->Start();
        result = (IS_OK(code) > 0) ? false : true;
        break;
      }
    case protocol::srv::SensorOperation::Request::OPR_STOP:
      {
        code = elem->Stop();
        result = (IS_OK(code) > 0) ? false : true;
        break;
      }
    case protocol::srv::SensorOperation::Request::OPR_CLOSE:
      {
        code = elem->Close();
        result = (IS_OK(code) > 0) ? false : true;
        break;
      }
    default:
      break;
  }
  return result;
}

int32_t cyberdog::sensor::SensorManager::OnError()
{
  return code_ptr_->GetKeyCode(SYS::KeyCode::kOK);
}

int32_t cyberdog::sensor::SensorManager::OnLowPower()
{
  int32_t return_code = code_ptr_->GetKeyCode(SYS::KeyCode::kOK);

  return_code = this->lidar_->LowPowerOn();
  if (!IS_OK(return_code)) {
    ERROR("Lidar set low power fail.");
    return return_code;
  } else {
    INFO("Lidar set low power success.");
  }

  return_code = this->ultrasonic_->LowPowerOn();
  if (!IS_OK(return_code)) {
    ERROR("Ultrasonic set low power fail.");
    return return_code;
  } else {
    INFO("Ultrasonic set low power success.");
  }

  return_code = this->tof_->LowPowerOn();
  if (!IS_OK(return_code)) {
    ERROR("Tof set low power fail.");
    return return_code;
  } else {
    INFO("Tof set low power success.");
  }

  return code_ptr_->GetKeyCode(SYS::KeyCode::kOK);
}

int32_t cyberdog::sensor::SensorManager::OnSuspend()
{
  return code_ptr_->GetKeyCode(SYS::KeyCode::kOK);
}

int32_t cyberdog::sensor::SensorManager::OnProtected()
{
  return code_ptr_->GetKeyCode(SYS::KeyCode::kOK);
}

int32_t cyberdog::sensor::SensorManager::OnActive()
{
  int32_t return_code = code_ptr_->GetKeyCode(SYS::KeyCode::kOK);

  INFO("SensorManager Running begin");
  return_code = this->lidar_->Start();
  if (!IS_OK(return_code)) {
    ERROR("Lidar start fail.");
    return return_code;
  } else {
    INFO("Lidar start success.");
  }
  // GPS TODO
  // if (!this->gps_->Start()) {
  //   ERROR("Gps start fail.");
  //   return code_ptr_->GetKeyCode(SYS::KeyCode::kFailed);
  // }
  // INFO("Gps start success.");
  return_code = this->ultrasonic_->LowPowerOff();
  if (!IS_OK(return_code)) {
    ERROR("Ultrasonic start fail.");
    return return_code;
  } else {
    INFO("Ultrasonic start success.");
  }

  return_code = this->tof_->LowPowerOff();
  if (!IS_OK(return_code)) {
    ERROR("Tof start fail.");
    return return_code;
  } else {
    INFO("Tof start success.");
  }

  INFO("Sensor manager start success.");
  return code_ptr_->GetKeyCode(SYS::KeyCode::kOK);
}

int32_t cyberdog::sensor::SensorManager::OnDeActive()
{
  return code_ptr_->GetKeyCode(SYS::KeyCode::kOK);
}

int32_t cyberdog::sensor::SensorManager::OnSetUp()
{
  int32_t return_code;
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
  auto IsJump = [&](std::string name, std::string step) {
      if (!sensor_self_check_ptr->IsJump("ultrasonic")) {
        INFO("Jump %s at %s error.", name.c_str(), step.c_str());
        return true;
      } else {
        return false;
      }
    };

  return_code = ultrasonic_->Init(is_simulator("ultrasonic"));
  if (IS_OK(return_code)) {
    return_code = ultrasonic_->Open();
    if (!IS_OK(return_code)) {
      if (!IsJump("ultrasonic", "open")) {return return_code;}
    }
  } else {
    if (!IsJump("ultrasonic", "init")) {return return_code;}
  }

  return_code = tof_->Init(is_simulator("tof"));
  if (IS_OK(return_code)) {
    return_code = tof_->Open();
    if (!IS_OK(return_code)) {
      if (!IsJump("tof", "open")) {return return_code;}
    }
  } else {
    if (!IsJump("tof", "init")) {return return_code;}
  }

  return_code = lidar_->Init(is_simulator("lidar"));
  if (IS_OK(return_code)) {
    return_code = lidar_->Open();
    if (!IS_OK(return_code)) {
      if (!IsJump("lidar", "open")) {return return_code;}
    }
  } else {
    if (!IsJump("lidar", "init")) {return return_code;}
  }

  return_code = gps_->Init(is_simulator("gps"));
  if (IS_OK(return_code)) {
    return_code = gps_->Open();
    if (!IS_OK(return_code)) {
      if (!IsJump("gps", "open")) {return return_code;}
    }
  } else {
    if (!IsJump("gps", "init")) {return return_code;}
  }

  INFO("SensorManager Running begin");

  return_code = this->lidar_->Start();
  if (!IS_OK(return_code)) {
    ERROR("Lidar start fail.");
    if (!IsJump("lidar", "start")) {return return_code;}
  } else {
    INFO("Lidar start success.");
  }

  return_code = this->gps_->Start();
  if (!IS_OK(return_code)) {
    ERROR("Gps start fail.");
    if (!IsJump("gps", "start")) {return return_code;}
  } else {
    INFO("Gps start success.");
  }

  return_code = this->ultrasonic_->Start();
  if (!IS_OK(return_code)) {
    ERROR("Ultrasonic start fail.");
    if (!IsJump("ultrasonic", "start")) {return return_code;}
  } else {
    INFO("Ultrasonic start success.");
  }

  return_code = this->tof_->Start();
  if (!IS_OK(return_code)) {
    ERROR("Tof start fail.");
    if (!IsJump("tof", "start")) {return return_code;}
  } else {
    INFO("Tof start success.");
  }

  INFO("Sensor manager start success.");
  INFO("sensor setup success");
  return code_ptr_->GetKeyCode(SYS::KeyCode::kOK);
}

int32_t cyberdog::sensor::SensorManager::ONTearDown()
{
  return code_ptr_->GetKeyCode(SYS::KeyCode::kOK);
}

int32_t cyberdog::sensor::SensorManager::OnOTA()
{
  return code_ptr_->GetKeyCode(SYS::KeyCode::kOK);
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
  std::unique_lock<std::mutex> lock(ultrasonic_lock_);
  ultrasonic_publisher_->publish(*msg);
}

void cyberdog::sensor::SensorManager::SingleTofPayloadCallback(
  std::shared_ptr<protocol::msg::SingleTofPayload> msg)
{
  if (msg->header.frame_id == "left_rear_tof" || msg->header.frame_id == "right_rear_tof") {
    if (rear_tof_.find(msg->header.frame_id) == head_tof_.end()) {
      WARN("rear tof map no name [%s] tof msg", msg->header.frame_id.c_str());
      return;
    }
    std::unique_lock<std::mutex> lock(rear_tof_lock_);
    if (msg->header.frame_id == "left_rear_tof") {
      rear_tof_.at(msg->header.frame_id) = true;
      rear_tof_payload->left_rear = *msg;
      rear_tof_payload->left_rear.header.frame_id = "left_rear";
      rear_tof_payload->left_rear.tof_position = protocol::msg::SingleTofPayload::LEFT_REAR;
    } else {
      rear_tof_.at(msg->header.frame_id) = true;
      rear_tof_payload->right_rear = *msg;
      rear_tof_payload->right_rear.header.frame_id = "right_rear";
      rear_tof_payload->right_rear.tof_position = protocol::msg::SingleTofPayload::RIGHT_REAR;
    }
    if (rear_tof_.at("left_rear_tof") && rear_tof_.at("right_rear_tof")) {
      rear_tof_publisher_->publish(*rear_tof_payload);
      rear_tof_.at("left_rear_tof") = false;
      rear_tof_.at("right_rear_tof") = false;
    }
  } else if (msg->header.frame_id == "left_head_tof" || msg->header.frame_id == "right_head_tof") {
    if (head_tof_.find(msg->header.frame_id) == head_tof_.end()) {
      WARN("head tof map no name [%s] tof msg", msg->header.frame_id.c_str());
      return;
    }
    std::unique_lock<std::mutex> lock(rear_tof_lock_);
    if (msg->header.frame_id == "left_head_tof") {
      head_tof_.at(msg->header.frame_id) = true;
      head_tof_payload->left_head = *msg;
      head_tof_payload->left_head.header.frame_id = "left_head";
      head_tof_payload->left_head.tof_position = protocol::msg::SingleTofPayload::LEFT_HEAD;
    } else {
      head_tof_.at(msg->header.frame_id) = true;
      head_tof_payload->right_head = *msg;
      head_tof_payload->right_head.header.frame_id = "right_head";
      head_tof_payload->right_head.tof_position = protocol::msg::SingleTofPayload::RIGHT_HEAD;
    }
    if (head_tof_.at("left_head_tof") && head_tof_.at("right_head_tof")) {
      head_tof_publisher_->publish(*head_tof_payload);
      head_tof_.at("left_head_tof") = false;
      head_tof_.at("right_head_tof") = false;
    }
  }
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
