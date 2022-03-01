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
#include "ultrasonic_plugin/ultrasonic_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"


bool cyberdog::sensor::UltrasonicCarpo::Open()
{
  opened_ = false;
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto path = local_share_dir + std::string("/toml_config/sensors/ultrasonic.toml");
  if (access(path.c_str(), F_OK) != 0) {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_ultrasonic"), "%s do not exist!", path.c_str());
    RCLCPP_INFO(
      rclcpp::get_logger(
        "cyberdog_ultrasonic"), "fail to start ultrasonic,started =   %d ",
      static_cast<int>(opened_));
    return opened_;
  }
  ultrasonic_can_ = std::make_shared<EVM::Protocol<ultrasonic_can>>(path, false);
  ultrasonic_can_->SetDataCallback(
    std::bind(
      &cyberdog::sensor::UltrasonicCarpo::recv_callback,
      this, std::placeholders::_1, std::placeholders::_2));
  ultrasonic_can_->Operate("enable_on", std::vector<uint8_t>{0x00});
  ultrasonic_can_->LINK_VAR(ultrasonic_can_->GetData()->enable_on_ack);

  time_t time_opened_delay = time(nullptr);
  while (opened_ == false && difftime(time(nullptr), time_opened_delay) < 2.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
    RCLCPP_INFO(
      rclcpp::get_logger("cyberdog_ultrasonic"), "difftime = %2f ",
      difftime(time(nullptr), time_opened_delay));
  }

  RCLCPP_INFO(rclcpp::get_logger("cyberdog_ultrasonic"), "ultrasonic started successfully started");
  return opened_;
}

bool cyberdog::sensor::UltrasonicCarpo::Start()
{
  RCLCPP_INFO(rclcpp::get_logger("cyberdog_ultrasonic"), "ultrasonic started successfully opened");
  return opened_;
}

bool cyberdog::sensor::UltrasonicCarpo::Stop()
{
  std::this_thread::sleep_for(std::chrono::microseconds(3000000));
  if (opened_ == false) {
    RCLCPP_INFO(
      rclcpp::get_logger(
        "cyberdog_ultrasonic"), "The ultrasonic sensor is not on, so the close fails");
    return false;
  }
  ultrasonic_can_->Operate(
    "enable_off", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00});
  ultrasonic_can_->BREAK_VAR(ultrasonic_can_->GetData()->ultrasonic_data);
  ultrasonic_can_->BREAK_VAR(ultrasonic_can_->GetData()->ultrasonic_data_intensity);
  ultrasonic_can_->BREAK_VAR(ultrasonic_can_->GetData()->ultrasonic_data_clock);
  ultrasonic_can_->LINK_VAR(ultrasonic_can_->GetData()->enable_off_ack);
  time_t time_closed_delay = time(nullptr);
  while (opened_ == true && difftime(time(nullptr), time_closed_delay) < 2.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
    RCLCPP_INFO(
      rclcpp::get_logger("cyberdog_ultrasonic"), "difftime = %2f ",
      difftime(time(nullptr), time_closed_delay));
  }
  RCLCPP_INFO(rclcpp::get_logger("cyberdog_ultrasonic"), "ultrasonic Closed successfully");

  return !opened_;
}

bool cyberdog::sensor::UltrasonicCarpo::Close()
{
  if (opened_ == false) {
    RCLCPP_INFO(
      rclcpp::get_logger(
        "cyberdog_ultrasonic"), "The ultrasonic sensor is not on, so the stop fails");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("cyberdog_ultrasonic"), "ultrasonic Stoped successfully");
  return !opened_;
}

void cyberdog::sensor::UltrasonicCarpo::recv_callback(
  std::string & name,
  std::shared_ptr<cyberdog::sensor::ultrasonic_can> data)
{
  ultrasonic_data_ = data;
  if (name == "enable_on_ack") {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_ultrasonic"), "got %s callback", name.c_str());
    started_ = true;
    ultrasonic_can_->BREAK_VAR(ultrasonic_can_->GetData()->enable_on_ack);
    ultrasonic_can_->LINK_VAR(ultrasonic_can_->GetData()->ultrasonic_data);
    ultrasonic_can_->LINK_VAR(ultrasonic_can_->GetData()->ultrasonic_data_intensity);
    ultrasonic_can_->LINK_VAR(ultrasonic_can_->GetData()->ultrasonic_data_clock);

  } else if (name == "ultrasonic_data") {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_ultrasonic"), "got %s callback", name.c_str());
    opened_ = true;
    std::string out_put("");
    std::cout << "ultrasonic_data   " << ultrasonic_data_->ultrasonic_data << std::endl;
  } else if (name == "ultrasonic_data_clock") {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_ultrasonic"), "got %s callback", name.c_str());
    opened_ = true;
    std::string out_put("");
    /*
    std::cout<<"ultrasonic_data= "<<ultrasonic_data_->ultrasonic_data<<std::endl;
    std::cout<<"ultrasonic_data_clock= "<<ultrasonic_data_->ultrasonic_data_clock<<std::endl;
    std::cout<<"ultrasonic_data_intensity= "<<ultrasonic_data_->ultrasonic_data_intensity<<std::endl;
    */
  } else if (name == "ultrasonic_data_intensity") {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_ultrasonic"), "got %s callback", name.c_str());
    opened_ = true;
    std::string out_put("");
    std::cout << "ultrasonic_data_intensity= " << ultrasonic_data_->ultrasonic_data_intensity <<
      std::endl;
  } else if (name == "enable_off_ack") {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_ultrasonic"), "got %s callback", name.c_str());
    opened_ = false;
  }
  auto ultrasonic_payload = std::make_shared<sensor_msgs::msg::Range>();
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
  } else {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_ultrasonic"), "do not get payload_callback_");
  }
}

PLUGINLIB_EXPORT_CLASS(cyberdog::sensor::UltrasonicCarpo, cyberdog::sensor::UltrasonicBase)
