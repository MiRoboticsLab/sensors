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

#include <unistd.h>
#include <time.h>
#include <chrono>
#include <memory>
#include <ctime>
#include <vector>
#include <string>
#include "tof_plugin/tof_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_common/cyberdog_log.hpp"


bool cyberdog::sensor::TofCarpo::Init(bool simulator)
{
  this->state_msg_.insert({SwitchState::open, "Open"});
  this->state_msg_.insert({SwitchState::start, "Start"});
  this->state_msg_.insert({SwitchState::stop, "Stop"});
  this->state_msg_.insert({SwitchState::close, "Close"});

  if (!simulator) {
    this->Open = std::bind(&cyberdog::sensor::TofCarpo::Open_, this);
    this->Start = std::bind(&cyberdog::sensor::TofCarpo::Start_, this);
    this->Stop = std::bind(&cyberdog::sensor::TofCarpo::Stop_, this);
    this->Close = std::bind(&cyberdog::sensor::TofCarpo::Close_, this);
  } else {
    auto Simulator = [this](SwitchState now_state) -> bool {
        INFO("%s cyberdog_tof ...", this->state_msg_[now_state].c_str());
        switch (now_state) {
          case SwitchState::open:
          case SwitchState::stop:
            break;
          case SwitchState::start:
            head_tof_payload = std::make_shared<protocol::msg::HeadTofPayload>();
            rear_tof_payload = std::make_shared<protocol::msg::RearTofPayload>();

            tof_pub_thread_simulator =
              std::thread(std::bind(&cyberdog::sensor::TofCarpo::UpdateSimulationData, this));
            break;
          case SwitchState::close:
            if ((&tof_pub_thread_simulator != nullptr) &&
              tof_pub_thread_simulator.joinable())
            {
              tof_pub_thread_simulator.join();
            }
            break;
          default:
            WARN("cyberdog_tof not recognized state");
            break;
        }

        INFO("cyberdog_tof %s ok", this->state_msg_[now_state].c_str());
        return true;
      };
    this->Open = std::bind(Simulator, SwitchState::open);
    this->Start = std::bind(Simulator, SwitchState::start);
    this->Stop = std::bind(Simulator, SwitchState::stop);
    this->Close = std::bind(Simulator, SwitchState::close);
  }
  return true;
}


bool cyberdog::sensor::TofCarpo::Open_()
{
  head_tof_payload = std::make_shared<protocol::msg::HeadTofPayload>();
  rear_tof_payload = std::make_shared<protocol::msg::RearTofPayload>();

  if (SingleOpen(protocol::msg::SingleTofPayload::HEAD)) {
    INFO("head tofs opened successfully");

  } else {
    FATAL("head tofs opened failed");
  }

  if (SingleOpen(protocol::msg::SingleTofPayload::REAR)) {
    INFO("rear tofs opened successfully");
  } else {
    FATAL("rear tofs opened failed");
  }

  opened_ = tof_opened_head && tof_opened_rear;
  INFO(
    "all tofs opened status = %d ",
    static_cast<int>(opened_));
  return opened_;
}


bool cyberdog::sensor::TofCarpo::Start_()
{
  if (SingleStart(protocol::msg::SingleTofPayload::HEAD)) {
    INFO("head tofs started successfully");
  } else {
    FATAL("head tofs started failed");
  }

  if (SingleStart(protocol::msg::SingleTofPayload::REAR)) {
    INFO("rear tofs started successfully");
  } else {
    FATAL("rear tofs started failed");
  }
  started_ = tof_started_head && tof_started_rear;
  return started_;
}

bool cyberdog::sensor::TofCarpo::Stop_()
{
  if (SingleStop(protocol::msg::SingleTofPayload::HEAD)) {
    INFO("head tofs stoped successfully");
  } else {
    FATAL("head tofs stoped failed");
  }
  if (SingleStop(protocol::msg::SingleTofPayload::REAR)) {
    INFO("rear tofs stoped successfully");
  } else {
    FATAL("rear tofs stoped failed");
  }
  stopped_ = tof_started_head || tof_started_rear;
  return !stopped_;
}

bool cyberdog::sensor::TofCarpo::Close_()
{
  closed_ = stopped_;
  if (closed_ == true) {
    INFO("tofs closed successfully");
  } else {
    FATAL("tofs closed failed");
  }
  return closed_;
}


bool cyberdog::sensor::TofCarpo::SingleStart(uint8_t serial_number)
{
  switch (serial_number) {
    // head
    case protocol::msg::SingleTofPayload::HEAD: {
        time_t time_started_delay = time(nullptr);
        while (tof_started_head == false &&
          difftime(time(nullptr), time_started_delay) < 2.0f)
        {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          INFO(
            "difftime = %2f ",
            difftime(time(nullptr), time_started_delay));
        }
        if (tof_started_head == false) {
          FATAL("head tofs  started failed ");
        } else {
          INFO("head tofs  started successfully ");
        }
        return tof_started_head;
      }
    // rear
    case protocol::msg::SingleTofPayload::REAR: {
        time_t time_started_delay = time(nullptr);
        while (tof_started_rear == false &&
          difftime(time(nullptr), time_started_delay) < 2.0f)
        {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          INFO(
            "difftime = %2f ",
            difftime(time(nullptr), time_started_delay));
        }
        if (tof_started_rear == false) {
          FATAL("rear tofs  started failed ");
        } else {
          INFO("rear tofs  started successfully ");
        }
        return tof_started_rear;
      }
    default: {
        return false;
      }
  }
}


bool cyberdog::sensor::TofCarpo::SingleStop(uint8_t serial_number)
{
  std::this_thread::sleep_for(std::chrono::microseconds(10000000));  // for test
  switch (serial_number) {
    // head
    case protocol::msg::SingleTofPayload::HEAD: {
        tof_can_head->BREAK_VAR(tof_can_head->GetData()->left_tof_data_array);
        tof_can_head->BREAK_VAR(tof_can_head->GetData()->left_tof_data_clock);
        tof_can_head->BREAK_VAR(tof_can_head->GetData()->left_tof_intensity_array);
        tof_can_head->BREAK_VAR(tof_can_head->GetData()->right_tof_data_array);
        tof_can_head->BREAK_VAR(tof_can_head->GetData()->right_tof_data_clock);
        tof_can_head->BREAK_VAR(tof_can_head->GetData()->right_tof_intensity_array);
        tof_can_head->LINK_VAR(tof_can_head->GetData()->enable_off_ack);
        tof_can_head->Operate(
          "enable_off", std::vector<uint8_t>{});
        time_t time_head = time(nullptr);
        while (tof_started_head == true && difftime(time(nullptr), time_head) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // INFO("difftime = %2f "
          // ,difftime(time(nullptr), time_head));
        }
        INFO(
          "head tofs stoped successfully !tof_started_head %d ",
          static_cast<int>(!tof_started_head));
        return !tof_started_head;
      }
    // rear
    case protocol::msg::SingleTofPayload::LEFT_REAR: {
        tof_can_rear->BREAK_VAR(tof_can_rear->GetData()->left_tof_data_array);
        tof_can_rear->BREAK_VAR(tof_can_rear->GetData()->left_tof_data_clock);
        tof_can_rear->BREAK_VAR(tof_can_rear->GetData()->left_tof_intensity_array);
        tof_can_rear->BREAK_VAR(tof_can_rear->GetData()->right_tof_data_array);
        tof_can_rear->BREAK_VAR(tof_can_rear->GetData()->right_tof_data_clock);
        tof_can_rear->BREAK_VAR(tof_can_rear->GetData()->right_tof_intensity_array);
        tof_can_rear->LINK_VAR(tof_can_rear->GetData()->enable_off_ack);
        tof_can_rear->Operate(
          "enable_off", std::vector<uint8_t>{});
        time_t time_rear = time(nullptr);
        while (tof_started_rear == true && difftime(time(nullptr), time_rear) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // INFO("difftime = %2f "
          // ,difftime(time(nullptr), time_rear));
        }
        INFO(
          "rear tofs stoped successfully !tof_started_rear %d ",
          static_cast<int>(!tof_started_rear));
        return !tof_started_rear;
      }
    default: {
        return false;
      }
  }
}

bool cyberdog::sensor::TofCarpo::SingleOpen(uint8_t serial_number)
{
  switch (serial_number) {
    // head
    case protocol::msg::SingleTofPayload::HEAD: {
        tof_opened_head = false;
        auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
        auto path = local_share_dir + std::string("/toml_config/sensors/tof_head.toml");
        if (access(path.c_str(), F_OK) != 0) {
          ERROR("%s do not exist!", path.c_str());
          ERROR(
            "fail to open tof,tof_opened_head=   %d ",
            static_cast<int>(tof_opened_head));
          return tof_opened_head;
        }
        tof_can_head = std::make_shared<EVM::Protocol<tof_can>>(path, false);
        tof_can_head->SetDataCallback(
          std::bind(
            &cyberdog::sensor::TofCarpo::
            head_callback, this, std::placeholders::_1, std::placeholders::_2));
        tof_can_head->Operate(
          "enable_on", std::vector<uint8_t>{});
        tof_can_head->LINK_VAR(tof_can_head->GetData()->enable_on_ack);
        time_t time_head = time(nullptr);
        while (tof_opened_head == false && difftime(time(nullptr), time_head) < 4.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"),
          // "difftime = %2f ",difftime(time(nullptr), time_head));
        }

        INFO(
          "tof opened successfully tof_opened_head= %d ",
          static_cast<int>(tof_opened_head));
        return tof_opened_head;
      }
    // rear
    case protocol::msg::SingleTofPayload::REAR: {
        tof_opened_rear = false;
        auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
        auto path = local_share_dir + std::string("/toml_config/sensors/tof_rear.toml");
        if (access(path.c_str(), F_OK) != 0) {
          ERROR("%s do not exist!", path.c_str());
          ERROR(
            "fail to open tof,tof_opened_rear=   %d ",
            static_cast<int>(tof_opened_rear));
          return tof_opened_rear;
        }
        tof_can_rear = std::make_shared<EVM::Protocol<tof_can>>(path, false);
        tof_can_rear->SetDataCallback(
          std::bind(
            &cyberdog::sensor::TofCarpo::rear_callback,
            this, std::placeholders::_1, std::placeholders::_2));
        tof_can_rear->Operate(
          "enable_on", std::vector<uint8_t>{});
        tof_can_rear->LINK_VAR(tof_can_rear->GetData()->enable_on_ack);
        time_t time_rear = time(nullptr);
        while (tof_opened_rear == false && difftime(time(nullptr), time_rear) < 4.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"),
          // "difftime = %2f ",difftime(time(nullptr), time_rear));
        }
        INFO(
          "tof opened successfully tof_opened_rear= %d ",
          static_cast<int>(tof_opened_rear));
        return tof_opened_rear;
      }
    default: {
        return false;
      }
  }
}

void cyberdog::sensor::TofCarpo::head_callback(
  std::string & name,
  std::shared_ptr<cyberdog::sensor::tof_can> data)
{
  INFO_STREAM("~~~~ head callback ~~~~~ ");
  INFO_STREAM("    name ==   " << name);
  if (name == "enable_on_ack") {
    INFO_STREAM(" got head tofs callback " << name);
    tof_opened_head = true;
    tof_can_head->BREAK_VAR(tof_can_head->GetData()->enable_on_ack);
    tof_can_head->LINK_VAR(tof_can_head->GetData()->left_tof_data_array);
    tof_can_head->LINK_VAR(tof_can_head->GetData()->left_tof_data_clock);
    tof_can_head->LINK_VAR(tof_can_head->GetData()->left_tof_intensity_array);
    tof_can_head->LINK_VAR(tof_can_head->GetData()->right_tof_data_array);
    tof_can_head->LINK_VAR(tof_can_head->GetData()->right_tof_data_clock);
    tof_can_head->LINK_VAR(tof_can_head->GetData()->right_tof_intensity_array);
  } else if (name == "enable_off_ack") {
    INFO_STREAM("got head tofs callback" << name);
    tof_opened_head = false;
    tof_started_head = false;
  } else {
    tof_started_head = true;
    const int datanum = protocol::msg::SingleTofPayload::TOF_DATA_NUM;
    std::vector<float> obj_left_data;
    std::vector<float> obj_right_data;
    std::vector<float> obj_left_intensity;
    std::vector<float> obj_right_intensity;
    for (size_t i = 0; i < datanum; i++) {
      obj_left_data.push_back(
        (data->left_tof_data_array[i] * 2.0f + TOFOFFSET) *
        protocol::msg::SingleTofPayload::SCALE_FACTOR);
      obj_left_intensity.push_back(
        data->left_tof_intensity_array[i]);
      obj_right_data.push_back(
        (data->right_tof_data_array[i] * 2.0f + TOFOFFSET) *
        protocol::msg::SingleTofPayload::SCALE_FACTOR);
      obj_right_intensity.push_back(
        data->right_tof_intensity_array[i]);
    }
    auto tof_payload_left = std::make_shared<protocol::msg::SingleTofPayload>();
    auto tof_payload_right = std::make_shared<protocol::msg::SingleTofPayload>();

    struct timespec time_stu;
    clock_gettime(CLOCK_REALTIME, &time_stu);
    // left head
    tof_payload_left->header.frame_id = std::string("left_head");
    tof_payload_left->header.stamp.nanosec = time_stu.tv_nsec;
    tof_payload_left->header.stamp.sec = time_stu.tv_sec;
    tof_payload_left->tof_position = protocol::msg::SingleTofPayload::LEFT_HEAD;
    tof_payload_left->data = obj_left_data;
    tof_payload_left->intensity = obj_left_intensity;
    tof_payload_left->data_available = tof_started_head;
    head_tof_payload->left_head = *tof_payload_left;
    // right head
    tof_payload_right->header.frame_id = std::string("right_head");
    tof_payload_right->header.stamp.nanosec = time_stu.tv_nsec;
    tof_payload_right->header.stamp.sec = time_stu.tv_sec;
    tof_payload_right->tof_position = protocol::msg::SingleTofPayload::RIGHT_HEAD;
    tof_payload_right->data = obj_right_data;
    tof_payload_right->intensity = obj_right_intensity;
    tof_payload_right->data_available = tof_started_head;
    head_tof_payload->right_head = *tof_payload_right;
    // publish msg
    if (head_payload_callback_ != nullptr) {
      head_payload_callback_(head_tof_payload);
      INFO("head tofs published successfully");
    } else {
      ERROR("head tofs published unsuccessfully");
    }
  }
}

void cyberdog::sensor::TofCarpo::rear_callback(
  std::string & name,
  std::shared_ptr<cyberdog::sensor::tof_can> data)
{
  INFO_STREAM("+++++ rear callback ++++++ ");
  INFO_STREAM("    name ==   " << name);
  if (name == "enable_on_ack") {
    INFO_STREAM(" got rear tofs callback" << name);
    tof_opened_rear = true;
    tof_can_rear->BREAK_VAR(tof_can_rear->GetData()->enable_on_ack);
    tof_can_rear->LINK_VAR(tof_can_rear->GetData()->left_tof_data_array);
    tof_can_rear->LINK_VAR(tof_can_rear->GetData()->left_tof_data_clock);
    tof_can_rear->LINK_VAR(tof_can_rear->GetData()->left_tof_intensity_array);
    tof_can_rear->LINK_VAR(tof_can_rear->GetData()->right_tof_data_array);
    tof_can_rear->LINK_VAR(tof_can_rear->GetData()->right_tof_data_clock);
    tof_can_rear->LINK_VAR(tof_can_rear->GetData()->right_tof_intensity_array);

  } else if (name == "enable_off_ack") {
    INFO_STREAM("got rear tofs callback" << name);
    tof_opened_rear = false;
    tof_started_rear = false;
  } else {
    tof_started_rear = true;
    const int datanum = protocol::msg::SingleTofPayload::TOF_DATA_NUM;
    std::vector<float> obj_left_data;
    std::vector<float> obj_right_data;
    std::vector<float> obj_left_intensity;
    std::vector<float> obj_right_intensity;
    for (size_t i = 0; i < datanum; i++) {
      obj_left_data.push_back(
        (data->left_tof_data_array[i] * 2.0f + TOFOFFSET) *
        protocol::msg::SingleTofPayload::SCALE_FACTOR);
      obj_left_intensity.push_back(
        data->left_tof_intensity_array[i]);
      obj_right_data.push_back(
        (data->right_tof_data_array[i] * 2.0f + TOFOFFSET) *
        protocol::msg::SingleTofPayload::SCALE_FACTOR);
      obj_right_intensity.push_back(
        data->right_tof_intensity_array[i]);
    }
    auto tof_payload_left = std::make_shared<protocol::msg::SingleTofPayload>();
    auto tof_payload_right = std::make_shared<protocol::msg::SingleTofPayload>();

    struct timespec time_stu;
    clock_gettime(CLOCK_REALTIME, &time_stu);
    // left rear
    tof_payload_left->header.frame_id = std::string("left_rear");
    tof_payload_left->header.stamp.nanosec = time_stu.tv_nsec;
    tof_payload_left->header.stamp.sec = time_stu.tv_sec;
    tof_payload_left->tof_position = protocol::msg::SingleTofPayload::LEFT_REAR;
    tof_payload_left->data = obj_left_data;
    tof_payload_left->intensity = obj_left_intensity;
    tof_payload_left->data_available = tof_started_rear;
    rear_tof_payload->left_rear = *tof_payload_left;
    // right rear
    tof_payload_right->header.frame_id = std::string("right_rear");
    tof_payload_right->header.stamp.nanosec = time_stu.tv_nsec;
    tof_payload_right->header.stamp.sec = time_stu.tv_sec;
    tof_payload_right->tof_position = protocol::msg::SingleTofPayload::RIGHT_REAR;
    tof_payload_right->data = obj_right_data;
    tof_payload_right->intensity = obj_right_intensity;
    tof_payload_right->data_available = tof_started_rear;
    rear_tof_payload->right_rear = *tof_payload_right;
    // publish msg
    if (rear_payload_callback_ != nullptr) {
      rear_payload_callback_(rear_tof_payload);
      INFO("rear tofs published successfully");
    } else {
      ERROR("rear tofs published unsuccessfully");
    }
  }
}

void cyberdog::sensor::TofCarpo::UpdateSimulationData()
{
  while (true) {
    if (!rclcpp::ok()) {
      WARN("[cyberdog_tof]: !rclcpp::ok()");
      break;
    }
    INFO("[cyberdog_tof]: publish cyberdog_tof payload succeed");
    std::this_thread::sleep_for(std::chrono::microseconds(200000));
    const int datanum = protocol::msg::SingleTofPayload::TOF_DATA_NUM;
    std::vector<float> obj;
    for (size_t i = 0; i < datanum; i++) {
      obj.push_back(1.0f * protocol::msg::SingleTofPayload::SCALE_FACTOR);
    }
    auto tof_payload = std::make_shared<protocol::msg::SingleTofPayload>();
    struct timespec time_stu;
    clock_gettime(CLOCK_REALTIME, &time_stu);
    tof_payload->header.frame_id = std::string("simulator");
    tof_payload->header.stamp.nanosec = time_stu.tv_nsec;
    tof_payload->header.stamp.sec = time_stu.tv_sec;
    tof_payload->tof_position = protocol::msg::SingleTofPayload::LEFT_HEAD;
    tof_payload->data = obj;
    tof_payload->data_available = false;
    head_tof_payload->left_head = *tof_payload;
    head_tof_payload->right_head = *tof_payload;
    rear_tof_payload->left_rear = *tof_payload;
    rear_tof_payload->right_rear = *tof_payload;
    head_payload_callback_(head_tof_payload);
    rear_payload_callback_(rear_tof_payload);
    INFO("[cyberdog_tof]: publish cyberdog_tof payload succeed");
  }
}

PLUGINLIB_EXPORT_CLASS(cyberdog::sensor::TofCarpo, cyberdog::sensor::TofBase)
