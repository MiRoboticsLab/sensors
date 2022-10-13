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
  tof_payload_left_head = std::make_shared<protocol::msg::SingleTofPayload>();
  tof_payload_right_head = std::make_shared<protocol::msg::SingleTofPayload>();
  tof_payload_left_rear = std::make_shared<protocol::msg::SingleTofPayload>();
  tof_payload_right_rear = std::make_shared<protocol::msg::SingleTofPayload>();


  if (SingleOpen(protocol::msg::SingleTofPayload::LEFT_HEAD)) {
    INFO("left head tof opened successfully");

  } else {
    FATAL("left head tof opened failed");
  }

  if (SingleOpen(protocol::msg::SingleTofPayload::RIGHT_HEAD)) {
    INFO("right head tof opened successfully");

  } else {
    FATAL("right head tof opened failed");
  }

  if (SingleOpen(protocol::msg::SingleTofPayload::LEFT_REAR)) {
    INFO("left rear tof opened successfully");
  } else {
    FATAL("left rear tof opened failed");
  }

  if (SingleOpen(protocol::msg::SingleTofPayload::RIGHT_REAR)) {
    INFO("right rear tof opened successfully");
  } else {
    FATAL("right rear tof opened failed");
  }

  opened_ = tof_opened_left_head && tof_opened_right_head &&
    tof_opened_left_rear && tof_opened_right_rear;
  if (opened_ == true) {
    INFO("all tofs opened successfully");
  }
  return opened_;
}


bool cyberdog::sensor::TofCarpo::Start_()
{
  opened_ = tof_opened_left_head && tof_opened_right_head &&
    tof_opened_left_rear && tof_opened_right_rear;
  if (opened_ == false) {
    Open_();
  }
  if (SingleStart(protocol::msg::SingleTofPayload::LEFT_HEAD)) {
    INFO("left head tof started successfully");
  } else {
    FATAL("left head tof started failed");
  }

  if (SingleStart(protocol::msg::SingleTofPayload::RIGHT_HEAD)) {
    INFO("right head started successfully");
  } else {
    FATAL("right head started failed");
  }

  if (SingleStart(protocol::msg::SingleTofPayload::LEFT_REAR)) {
    INFO("left rear tof started successfully");
  } else {
    FATAL("left rear tof started failed");
  }

  if (SingleStart(protocol::msg::SingleTofPayload::RIGHT_REAR)) {
    INFO("right rear tof started successfully");
  } else {
    FATAL("right rear tof started failed");
  }

  started_ = tof_started_left_head && tof_started_right_head &&
    tof_started_left_rear && tof_started_right_rear;
  if (started_ == true) {
    INFO("all tofs started successfully");
  }
  return started_;
}

bool cyberdog::sensor::TofCarpo::Stop_()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));
  if (SingleStop(protocol::msg::SingleTofPayload::LEFT_HEAD)) {
    INFO("left head tof stoped successfully");
  } else {
    FATAL("left head tof stoped failed");
  }
  if (SingleStop(protocol::msg::SingleTofPayload::RIGHT_HEAD)) {
    INFO("right head tof stoped successfully");
  } else {
    FATAL("right head tofstoped failed");
  }

  if (SingleStop(protocol::msg::SingleTofPayload::LEFT_REAR)) {
    INFO("left rear tof stoped successfully");
  } else {
    FATAL("left rear tof stoped failed");
  }
  if (SingleStop(protocol::msg::SingleTofPayload::RIGHT_REAR)) {
    INFO("right rear tof stoped successfully");
  } else {
    FATAL("right rear tof stoped failed");
  }

  stopped_ = tof_started_left_head == 0 && tof_started_right_head == 0 &&
    tof_started_left_rear == 0 && tof_started_right_rear == 0;
  if (stopped_ == true) {
    INFO("all tofs stopped successfully");
  }
  return stopped_;
}

bool cyberdog::sensor::TofCarpo::Close_()
{
  closed_ = stopped_;
  if (closed_ == true) {
    INFO("all tofs closed successfully");
  } else {
    FATAL("all tofs closed failed");
  }
  return closed_;
}


bool cyberdog::sensor::TofCarpo::SingleStart(uint8_t serial_number)
{
  switch (serial_number) {
    // left head
    case protocol::msg::SingleTofPayload::LEFT_HEAD: {
        time_t time_started_delay = time(nullptr);
        while (tof_started_left_head == false &&
          difftime(time(nullptr), time_started_delay) < 2.0f)
        {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          INFO(
            "difftime = %2f ",
            difftime(time(nullptr), time_started_delay));
        }
        if (tof_started_left_head == false) {
          FATAL("left head tof  started failed ");
        } else {
          INFO("left head tof started successfully ");
        }
        return tof_started_left_head;
      }
    // right head
    case protocol::msg::SingleTofPayload::RIGHT_HEAD: {
        time_t time_started_delay = time(nullptr);
        while (tof_started_right_head == false &&
          difftime(time(nullptr), time_started_delay) < 2.0f)
        {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          INFO(
            "difftime = %2f ",
            difftime(time(nullptr), time_started_delay));
        }
        if (tof_started_right_head == false) {
          FATAL("right head tof  started failed ");
        } else {
          INFO("right head tof started successfully ");
        }
        return tof_started_right_head;
      }
    // left rear
    case protocol::msg::SingleTofPayload::LEFT_REAR: {
        time_t time_started_delay = time(nullptr);
        while (tof_started_left_rear == false &&
          difftime(time(nullptr), time_started_delay) < 2.0f)
        {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          INFO(
            "difftime = %2f ",
            difftime(time(nullptr), time_started_delay));
        }
        if (tof_started_left_rear == false) {
          FATAL("left rear tof  started failed ");
        } else {
          INFO("left rear tof started successfully ");
        }
        return tof_started_left_rear;
      }
    // right rear
    case protocol::msg::SingleTofPayload::RIGHT_REAR: {
        time_t time_started_delay = time(nullptr);
        while (tof_started_right_rear == false &&
          difftime(time(nullptr), time_started_delay) < 2.0f)
        {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          INFO(
            "difftime = %2f ",
            difftime(time(nullptr), time_started_delay));
        }
        if (tof_started_right_rear == false) {
          FATAL("right rear tof  started failed ");
        } else {
          INFO("right rear tof started successfully ");
        }
        return tof_started_right_rear;
      }
    default: {
        return false;
      }
  }
}

bool cyberdog::sensor::TofCarpo::SingleStop(uint8_t serial_number)
{
  switch (serial_number) {
    // left head
    case protocol::msg::SingleTofPayload::LEFT_HEAD: {
        tof_can_left_head->BREAK_VAR(tof_can_left_head->GetData()->data_array);
        tof_can_left_head->BREAK_VAR(tof_can_left_head->GetData()->data_clock);
        tof_can_left_head->BREAK_VAR(tof_can_left_head->GetData()->intensity_array);
        tof_can_left_head->LINK_VAR(tof_can_left_head->GetData()->enable_off_ack);
        tof_can_left_head->Operate(
          "enable_off", std::vector<uint8_t>{});
        INFO_STREAM("!!!!!!send left head tof enable_off cmd.!!!!!");
        time_t time_head = time(nullptr);
        while (tof_started_left_head == true && difftime(time(nullptr), time_head) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
        }
        return !tof_started_left_head;
      }
    // right head
    case protocol::msg::SingleTofPayload::RIGHT_HEAD: {
        tof_can_right_head->BREAK_VAR(tof_can_right_head->GetData()->data_array);
        tof_can_right_head->BREAK_VAR(tof_can_right_head->GetData()->data_clock);
        tof_can_right_head->BREAK_VAR(tof_can_right_head->GetData()->intensity_array);
        tof_can_right_head->LINK_VAR(tof_can_right_head->GetData()->enable_off_ack);
        tof_can_right_head->Operate(
          "enable_off", std::vector<uint8_t>{});
        INFO_STREAM("!!!!!!send right head tof enable_off cmd.!!!!!");
        time_t time_head = time(nullptr);
        while (tof_started_right_head == true && difftime(time(nullptr), time_head) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
        }
        return !tof_started_right_head;
      }

    // left rear
    case protocol::msg::SingleTofPayload::LEFT_REAR: {
        tof_can_left_rear->BREAK_VAR(tof_can_left_rear->GetData()->data_array);
        tof_can_left_rear->BREAK_VAR(tof_can_left_rear->GetData()->data_clock);
        tof_can_left_rear->BREAK_VAR(tof_can_left_rear->GetData()->intensity_array);
        tof_can_left_rear->LINK_VAR(tof_can_left_rear->GetData()->enable_off_ack);
        tof_can_left_rear->Operate(
          "enable_off", std::vector<uint8_t>{});
        INFO_STREAM("!!!!!!send left rear tof enable_off cmd.!!!!!");
        time_t time_head = time(nullptr);
        while (tof_started_left_rear == true && difftime(time(nullptr), time_head) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
        }
        return !tof_started_left_rear;
      }
    // right head
    case protocol::msg::SingleTofPayload::RIGHT_REAR: {
        tof_can_right_rear->BREAK_VAR(tof_can_right_rear->GetData()->data_array);
        tof_can_right_rear->BREAK_VAR(tof_can_right_rear->GetData()->data_clock);
        tof_can_right_rear->BREAK_VAR(tof_can_right_rear->GetData()->intensity_array);
        tof_can_right_rear->LINK_VAR(tof_can_right_rear->GetData()->enable_off_ack);
        tof_can_right_rear->Operate(
          "enable_off", std::vector<uint8_t>{});
        INFO_STREAM("!!!!!!send right rear tof enable_off cmd.!!!!!");
        time_t time_head = time(nullptr);
        while (tof_started_right_rear == true && difftime(time(nullptr), time_head) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
        }
        return !tof_started_right_rear;
      }


    default: {
        return false;
      }
  }
}

bool cyberdog::sensor::TofCarpo::SingleOpen(uint8_t serial_number)
{
  switch (serial_number) {
    // LEFT_HEAD
    case protocol::msg::SingleTofPayload::LEFT_HEAD: {
        tof_opened_left_head = false;
        auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
        auto path = local_share_dir + std::string("/toml_config/sensors/tof_left_head.toml");
        if (access(path.c_str(), F_OK) != 0) {
          ERROR("%s do not exist!", path.c_str());
          return tof_opened_left_head;
        }
        tof_can_left_head = std::make_shared<EVM::Protocol<tof_can>>(path, false);
        tof_can_left_head->SetDataCallback(
          std::bind(
            &cyberdog::sensor::TofCarpo::
            left_head_callback, this, std::placeholders::_1, std::placeholders::_2));
        tof_can_left_head->Operate(
          "enable_on", std::vector<uint8_t>{});
        tof_can_left_head->LINK_VAR(tof_can_left_head->GetData()->enable_on_ack);
        time_t time_head = time(nullptr);
        while (tof_opened_left_head == false && difftime(time(nullptr), time_head) < 4.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
        }
        return tof_opened_left_head;
      }
    // RIGHT_HEAD
    case protocol::msg::SingleTofPayload::RIGHT_HEAD: {
        tof_opened_right_head = false;
        auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
        auto path = local_share_dir + std::string("/toml_config/sensors/tof_right_head.toml");
        if (access(path.c_str(), F_OK) != 0) {
          ERROR("%s do not exist!", path.c_str());
          return tof_opened_right_head;
        }
        tof_can_right_head = std::make_shared<EVM::Protocol<tof_can>>(path, false);
        tof_can_right_head->SetDataCallback(
          std::bind(
            &cyberdog::sensor::TofCarpo::right_head_callback,
            this, std::placeholders::_1, std::placeholders::_2));
        tof_can_right_head->Operate(
          "enable_on", std::vector<uint8_t>{});
        tof_can_right_head->LINK_VAR(tof_can_right_head->GetData()->enable_on_ack);
        time_t time_rear = time(nullptr);
        while (tof_opened_right_head == false && difftime(time(nullptr), time_rear) < 4.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
        }
        return tof_opened_right_head;
      }
    case protocol::msg::SingleTofPayload::LEFT_REAR: {
        tof_opened_left_rear = false;
        auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
        auto path = local_share_dir + std::string("/toml_config/sensors/tof_left_rear.toml");
        if (access(path.c_str(), F_OK) != 0) {
          ERROR("%s do not exist!", path.c_str());
          ERROR(
            "fail to open left rear tof,tof_opened_left_rear=   %d ",
            static_cast<int>(tof_opened_left_rear));
          return tof_opened_left_rear;
        }
        tof_can_left_rear = std::make_shared<EVM::Protocol<tof_can>>(path, false);
        tof_can_left_rear->SetDataCallback(
          std::bind(
            &cyberdog::sensor::TofCarpo::left_rear_callback,
            this, std::placeholders::_1, std::placeholders::_2));
        tof_can_left_rear->Operate(
          "enable_on", std::vector<uint8_t>{});
        tof_can_left_rear->LINK_VAR(tof_can_left_rear->GetData()->enable_on_ack);
        time_t time_rear = time(nullptr);
        while (tof_opened_left_rear == false && difftime(time(nullptr), time_rear) < 4.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"),
          // "difftime = %2f ",difftime(time(nullptr), time_rear));
        }
        return tof_opened_left_rear;
      }
    case protocol::msg::SingleTofPayload::RIGHT_REAR: {
        tof_opened_right_rear = false;
        auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
        auto path = local_share_dir + std::string("/toml_config/sensors/tof_right_rear.toml");
        if (access(path.c_str(), F_OK) != 0) {
          ERROR("%s do not exist!", path.c_str());
          return tof_opened_right_rear;
        }
        tof_can_right_rear = std::make_shared<EVM::Protocol<tof_can>>(path, false);
        tof_can_right_rear->SetDataCallback(
          std::bind(
            &cyberdog::sensor::TofCarpo::right_rear_callback,
            this, std::placeholders::_1, std::placeholders::_2));
        tof_can_right_rear->Operate(
          "enable_on", std::vector<uint8_t>{});
        tof_can_right_rear->LINK_VAR(tof_can_right_rear->GetData()->enable_on_ack);
        time_t time_rear = time(nullptr);
        while (tof_opened_right_rear == false && difftime(time(nullptr), time_rear) < 4.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
        }
        return tof_opened_right_rear;
      }
    default: {
        return false;
      }
  }
}

void cyberdog::sensor::TofCarpo::left_head_callback(
  std::string & name,
  std::shared_ptr<cyberdog::sensor::tof_can> data)
{
  INFO_STREAM_ONCE("~~~~ left head callback ~~~~~ ");
  INFO_STREAM_ONCE("    name ==   " << name);
  if (name == "enable_on_ack") {
    INFO_STREAM_ONCE(" got left head tofs callback " << name);
    tof_opened_left_head = true;
    tof_can_left_head->BREAK_VAR(tof_can_left_head->GetData()->enable_on_ack);
    tof_can_left_head->LINK_VAR(tof_can_left_head->GetData()->data_array);
    tof_can_left_head->LINK_VAR(tof_can_left_head->GetData()->intensity_array);
    tof_can_left_head->LINK_VAR(tof_can_left_head->GetData()->data_clock);
  } else if (name == "enable_off_ack") {
    INFO_STREAM_ONCE("got left head tofs callback" << name);
    tof_opened_left_head = false;
    tof_started_left_head = false;
  } else {
    tof_started_left_head = true;
    const int datanum = protocol::msg::SingleTofPayload::TOF_DATA_NUM;
    std::vector<float> obj_data;
    std::vector<float> obj_intensity;
    for (size_t i = 0; i < datanum; i++) {
      obj_data.push_back(
        (data->data_array[i] * 2.0f + TOFOFFSET) *
        protocol::msg::SingleTofPayload::SCALE_FACTOR);
      obj_intensity.push_back(
        data->intensity_array[i]);
    }

    struct timespec time_stu;
    clock_gettime(CLOCK_REALTIME, &time_stu);
    // left head
    tof_payload_left_head->header.frame_id = std::string("left_head");
    tof_payload_left_head->header.stamp.nanosec = time_stu.tv_nsec;
    tof_payload_left_head->header.stamp.sec = time_stu.tv_sec;
    tof_payload_left_head->tof_position = protocol::msg::SingleTofPayload::LEFT_HEAD;
    tof_payload_left_head->data = obj_data;
    tof_payload_left_head->intensity = obj_intensity;
    tof_payload_left_head->data_available = tof_started_left_head;
    head_tof_payload->left_head = *tof_payload_left_head;
    head_tof_payload->right_head = *tof_payload_right_head;
    // publish msg
    if (head_payload_callback_ != nullptr) {
      head_payload_callback_(head_tof_payload);
      INFO_ONCE("head tofs published successfully");
    } else {
      ERROR_ONCE("head tofs published failed");
    }
  }
}

void cyberdog::sensor::TofCarpo::right_head_callback(
  std::string & name,
  std::shared_ptr<cyberdog::sensor::tof_can> data)
{
  INFO_STREAM_ONCE("~~~~ right head callback ~~~~~ ");
  INFO_STREAM_ONCE("    name ==   " << name);
  if (name == "enable_on_ack") {
    INFO_STREAM_ONCE(" got right head tofs callback " << name);
    tof_opened_right_head = true;
    tof_can_right_head->BREAK_VAR(tof_can_right_head->GetData()->enable_on_ack);
    tof_can_right_head->LINK_VAR(tof_can_right_head->GetData()->data_array);
    tof_can_right_head->LINK_VAR(tof_can_right_head->GetData()->intensity_array);
    tof_can_right_head->LINK_VAR(tof_can_right_head->GetData()->data_clock);
  } else if (name == "enable_off_ack") {
    INFO_STREAM_ONCE("got right head tofs callback" << name);
    tof_opened_right_head = false;
    tof_started_right_head = false;
  } else {
    tof_started_right_head = true;
    const int datanum = protocol::msg::SingleTofPayload::TOF_DATA_NUM;
    std::vector<float> obj_data;
    std::vector<float> obj_intensity;
    for (size_t i = 0; i < datanum; i++) {
      obj_data.push_back(
        (data->data_array[i] * 2.0f + TOFOFFSET) *
        protocol::msg::SingleTofPayload::SCALE_FACTOR);
      obj_intensity.push_back(
        data->intensity_array[i]);
    }

    struct timespec time_stu;
    clock_gettime(CLOCK_REALTIME, &time_stu);
    // right head
    tof_payload_right_head->header.frame_id = std::string("right_head");
    tof_payload_right_head->header.stamp.nanosec = time_stu.tv_nsec;
    tof_payload_right_head->header.stamp.sec = time_stu.tv_sec;
    tof_payload_right_head->tof_position = protocol::msg::SingleTofPayload::RIGHT_HEAD;
    tof_payload_right_head->data = obj_data;
    tof_payload_right_head->intensity = obj_intensity;
    tof_payload_right_head->data_available = tof_started_right_head;
    head_tof_payload->right_head = *tof_payload_right_head;
    // publish msg
  }
}

void cyberdog::sensor::TofCarpo::left_rear_callback(
  std::string & name,
  std::shared_ptr<cyberdog::sensor::tof_can> data)
{
  INFO_STREAM_ONCE("~~~~ left rear callback ~~~~~ ");
  INFO_STREAM_ONCE("    name ==   " << name);
  if (name == "enable_on_ack") {
    INFO_STREAM_ONCE(" got left rear tofs callback " << name);
    tof_opened_left_rear = true;
    tof_can_left_rear->BREAK_VAR(tof_can_left_rear->GetData()->enable_on_ack);
    tof_can_left_rear->LINK_VAR(tof_can_left_rear->GetData()->data_array);
    tof_can_left_rear->LINK_VAR(tof_can_left_rear->GetData()->intensity_array);
    tof_can_left_rear->LINK_VAR(tof_can_left_rear->GetData()->data_clock);

  } else if (name == "enable_off_ack") {
    INFO_STREAM_ONCE("got left rear tofs callback" << name);
    tof_opened_left_rear = false;
    tof_started_left_rear = false;
  } else {
    tof_started_left_rear = true;
    const int datanum = protocol::msg::SingleTofPayload::TOF_DATA_NUM;
    std::vector<float> obj_data;
    std::vector<float> obj_intensity;
    for (size_t i = 0; i < datanum; i++) {
      obj_data.push_back(
        (data->data_array[i] * 2.0f + TOFOFFSET) *
        protocol::msg::SingleTofPayload::SCALE_FACTOR);
      obj_intensity.push_back(
        data->intensity_array[i]);
    }

    struct timespec time_stu;
    clock_gettime(CLOCK_REALTIME, &time_stu);
    // left rear
    tof_payload_left_rear->header.frame_id = std::string("left_rear");
    tof_payload_left_rear->header.stamp.nanosec = time_stu.tv_nsec;
    tof_payload_left_rear->header.stamp.sec = time_stu.tv_sec;
    tof_payload_left_rear->tof_position = protocol::msg::SingleTofPayload::LEFT_REAR;
    tof_payload_left_rear->data = obj_data;
    tof_payload_left_rear->intensity = obj_intensity;
    tof_payload_left_rear->data_available = tof_started_left_rear;
    rear_tof_payload->left_rear = *tof_payload_left_rear;
    rear_tof_payload->right_rear = *tof_payload_right_rear;
    // publish msg
    if (rear_payload_callback_ != nullptr) {
      rear_payload_callback_(rear_tof_payload);
      INFO_ONCE("rear tofs published successfully");
    } else {
      ERROR_ONCE("rear tofs published failed");
    }
  }
}

void cyberdog::sensor::TofCarpo::right_rear_callback(
  std::string & name,
  std::shared_ptr<cyberdog::sensor::tof_can> data)
{
  INFO_STREAM_ONCE("~~~~ right rear callback ~~~~~ ");
  INFO_STREAM_ONCE("    name ==   " << name);
  if (name == "enable_on_ack") {
    INFO_STREAM_ONCE(" got right rear tofs callback " << name);
    tof_opened_right_rear = true;
    tof_can_right_rear->BREAK_VAR(tof_can_right_rear->GetData()->enable_on_ack);
    tof_can_right_rear->LINK_VAR(tof_can_right_rear->GetData()->data_array);
    tof_can_right_rear->LINK_VAR(tof_can_right_rear->GetData()->intensity_array);
    tof_can_right_rear->LINK_VAR(tof_can_right_rear->GetData()->data_clock);
  } else if (name == "enable_off_ack") {
    INFO_STREAM_ONCE("got right rear tofs callback" << name);
    tof_opened_right_rear = false;
    tof_started_right_rear = false;
  } else {
    tof_started_right_rear = true;
    const int datanum = protocol::msg::SingleTofPayload::TOF_DATA_NUM;
    std::vector<float> obj_data;
    std::vector<float> obj_intensity;
    for (size_t i = 0; i < datanum; i++) {
      obj_data.push_back(
        (data->data_array[i] * 2.0f + TOFOFFSET) *
        protocol::msg::SingleTofPayload::SCALE_FACTOR);
      obj_intensity.push_back(
        data->intensity_array[i]);
    }

    struct timespec time_stu;
    clock_gettime(CLOCK_REALTIME, &time_stu);
    // right rear
    tof_payload_right_rear->header.frame_id = std::string("right_rear");
    tof_payload_right_rear->header.stamp.nanosec = time_stu.tv_nsec;
    tof_payload_right_rear->header.stamp.sec = time_stu.tv_sec;
    tof_payload_right_rear->tof_position = protocol::msg::SingleTofPayload::RIGHT_REAR;
    tof_payload_right_rear->data = obj_data;
    tof_payload_right_rear->intensity = obj_intensity;
    tof_payload_right_rear->data_available = tof_started_right_rear;
    rear_tof_payload->right_rear = *tof_payload_right_rear;
    // publish msg
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
