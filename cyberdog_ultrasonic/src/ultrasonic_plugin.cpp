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
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include "ultrasonic_plugin/ultrasonic_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_common/cyberdog_log.hpp"


bool cyberdog::sensor::UltrasonicCarpo::Init(bool simulator)
{
  this->state_msg_.insert({SwitchState::open, "Open"});
  this->state_msg_.insert({SwitchState::start, "Start"});
  this->state_msg_.insert({SwitchState::stop, "Stop"});
  this->state_msg_.insert({SwitchState::close, "Close"});

  if (!simulator) {
    this->Open = std::bind(&cyberdog::sensor::UltrasonicCarpo::Open_, this);
    this->Start = std::bind(&cyberdog::sensor::UltrasonicCarpo::Start_, this);
    this->Stop = std::bind(&cyberdog::sensor::UltrasonicCarpo::Stop_, this);
    this->Close = std::bind(&cyberdog::sensor::UltrasonicCarpo::Close_, this);
  } else {
    auto Simulator = [this](SwitchState now_state) -> bool {
        INFO("%s Ultrasonic ...", this->state_msg_[now_state].c_str());
        switch (now_state) {
          case SwitchState::open:
          case SwitchState::stop:
            break;
          case SwitchState::start:
            ultrasonic_payload = std::make_shared<sensor_msgs::msg::Range>();
            ultrasonic_pub_thread_simulator =
              std::thread(
              std::bind(
                &cyberdog::sensor::UltrasonicCarpo::UpdateSimulationData,
                this));
            break;
          case SwitchState::close:
            if ((&ultrasonic_pub_thread_simulator != nullptr) &&
              ultrasonic_pub_thread_simulator.joinable())
            {
              ultrasonic_pub_thread_simulator.join();
            }
            break;
          default:
            WARN("Ultrasonic not recognized state");
            break;
        }

        INFO("Ultrasonic %s ok", this->state_msg_[now_state].c_str());
        return true;
      };
    this->Open = std::bind(Simulator, SwitchState::open);
    this->Start = std::bind(Simulator, SwitchState::start);
    this->Stop = std::bind(Simulator, SwitchState::stop);
    this->Close = std::bind(Simulator, SwitchState::close);
  }
  return true;
}

bool cyberdog::sensor::UltrasonicCarpo::Open_()
{
  ultrasonic_payload = std::make_shared<sensor_msgs::msg::Range>();
  opened_ = false;
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto path = local_share_dir + std::string("/toml_config/sensors/ultrasonic.toml");
  if (access(path.c_str(), F_OK) != 0) {
    ERROR("%s do not exist!", path.c_str());
    ERROR(
      " fail to start ultrasonic,started =   %d ",
      static_cast<int>(opened_));
    return opened_;
  }
  ultrasonic_can_ = std::make_shared<EVM::Protocol<ultrasonic_can>>(path, false);
  ultrasonic_can_->SetDataCallback(
    std::bind(
      &cyberdog::sensor::UltrasonicCarpo::recv_callback,
      this, std::placeholders::_1, std::placeholders::_2));
  ultrasonic_can_->Operate("enable_on", std::vector<uint8_t>{});
  ultrasonic_can_->LINK_VAR(ultrasonic_can_->GetData()->enable_on_ack);

  time_t time_opened_delay = time(nullptr);
  while (opened_ == false && difftime(time(nullptr), time_opened_delay) < 2.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
    INFO(
      " difftime = %2f ",
      difftime(time(nullptr), time_opened_delay));
  }
  if (opened_ == false) {
    ERROR(" ultrasonic open failed ");
  } else {
    INFO(" ultrasonic open successfully ");
  }
  return opened_;
}

bool cyberdog::sensor::UltrasonicCarpo::Start_()
{
  if (opened_ == false) {
    Open_();
  }
  time_t time_started_delay = time(nullptr);
  while (started_ == false && difftime(time(nullptr), time_started_delay) < 2.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
    INFO(
      " difftime = %2f ",
      difftime(time(nullptr), time_started_delay));
  }
  if (started_ == false) {
    ERROR(" ultrasonic Start failed ");
  } else {
    INFO(" ultrasonic Start successfully ");
  }
  return started_;
}

bool cyberdog::sensor::UltrasonicCarpo::Stop_()
{
  if (opened_ == false || started_ == false) {
    ERROR(" The ultrasonic sensor is not on, so the close fails");
    return false;
  }
  ultrasonic_can_->BREAK_VAR(ultrasonic_can_->GetData()->ultrasonic_data);
  ultrasonic_can_->BREAK_VAR(ultrasonic_can_->GetData()->ultrasonic_data_intensity);
  ultrasonic_can_->BREAK_VAR(ultrasonic_can_->GetData()->ultrasonic_data_clock);
  ultrasonic_can_->LINK_VAR(ultrasonic_can_->GetData()->enable_off_ack);
  ultrasonic_can_->Operate(
    "enable_off", std::vector<uint8_t>{});
  time_t time_stopped_delay = time(nullptr);
  while (started_ == true && difftime(time(nullptr), time_stopped_delay) < 2.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
    INFO(
      " difftime = %2f ",
      difftime(time(nullptr), time_stopped_delay));
  }
  if (started_ == true) {
    ERROR(" ultrasonic stop failed ");
  } else {
    INFO(" ultrasonic stop successfully ");
  }
  return !started_;
}

bool cyberdog::sensor::UltrasonicCarpo::Close_()
{
  if (opened_ == true) {
    ERROR(" The ultrasonic closed fails");
    return false;
  } else {
    INFO(" ultrasonic closed successfully");
  }
  return !opened_;
}

void cyberdog::sensor::UltrasonicCarpo::recv_callback(
  std::string & name,
  std::shared_ptr<cyberdog::sensor::ultrasonic_can> data)
{
  INFO_STREAM_ONCE("!!!!!! Ultrasonic callback !!!!!!! ");
  INFO_STREAM_ONCE("    name ==   " << name);
  ultrasonic_data_ = data;
  if (name == "enable_on_ack") {
    INFO(" got %s callback", name.c_str());
    opened_ = true;
    ultrasonic_can_->BREAK_VAR(ultrasonic_can_->GetData()->enable_on_ack);
    ultrasonic_can_->LINK_VAR(ultrasonic_can_->GetData()->ultrasonic_data);
    ultrasonic_can_->LINK_VAR(ultrasonic_can_->GetData()->ultrasonic_data_intensity);
    ultrasonic_can_->LINK_VAR(ultrasonic_can_->GetData()->ultrasonic_data_clock);

  } else if (name == "enable_off_ack") {
    INFO(" got %s callback", name.c_str());
    opened_ = false;
    started_ = false;
  } else {
    // INFO(" got %s callback", name.c_str());
    started_ = true;
    struct timespec time_stu;
    clock_gettime(CLOCK_REALTIME, &time_stu);
    ultrasonic_payload->header.frame_id = std::string("ultrasonic");
    ultrasonic_payload->header.stamp.nanosec = time_stu.tv_nsec;
    ultrasonic_payload->header.stamp.sec = time_stu.tv_sec;
    ultrasonic_payload->radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    ultrasonic_payload->min_range = 0.1f;
    ultrasonic_payload->max_range = 1.0f;
    ultrasonic_payload->field_of_view = 15.0f;
    ultrasonic_payload->range = ultrasonic_data_->ultrasonic_data * 0.001f;
    if (payload_callback_ != nullptr) {
      payload_callback_(ultrasonic_payload);
      INFO_ONCE(" publish ultrasonic payload succeed");
    } else {
      ERROR_ONCE(" publish ultrasonic payload failed");
    }
  }
}

void cyberdog::sensor::UltrasonicCarpo::UpdateSimulationData()
{
  while (true) {
    if (!rclcpp::ok()) {
      WARN(" !rclcpp::ok()");
      break;
    }
    // INFO(" publish ultrasonic payload succeed");
    std::this_thread::sleep_for(std::chrono::microseconds(200000));

    struct timespec time_stu;
    clock_gettime(CLOCK_REALTIME, &time_stu);
    ultrasonic_payload->header.frame_id = std::string("ultrasonic");
    ultrasonic_payload->header.stamp.nanosec = time_stu.tv_nsec;
    ultrasonic_payload->header.stamp.sec = time_stu.tv_sec;
    ultrasonic_payload->radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    ultrasonic_payload->min_range = 0.1f;
    ultrasonic_payload->max_range = 1.0f;
    ultrasonic_payload->field_of_view = 15.0f;
    ultrasonic_payload->range = 0.001f;
    payload_callback_(ultrasonic_payload);
    // INFO(" publish ultrasonic payload succeed");
  }
}

PLUGINLIB_EXPORT_CLASS(cyberdog::sensor::UltrasonicCarpo, cyberdog::sensor::UltrasonicBase)
