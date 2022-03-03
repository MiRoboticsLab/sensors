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


bool cyberdog::sensor::TofCarpo::Open()
{
  multiple_tof_payload = std::make_shared<protocol::msg::MultipleTof>();

  if (SingleOpen(protocol::msg::Tof::LEFT_FRONT)) {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof_opened_left_front opened successfully");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof_opened_left_front opened failed");
  }
/*
  if (SingleOpen(protocol::msg::Tof::LEFT_BACK)) {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof_opened_left_back opened successfully");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof_opened_left_back opened failed");
  }

  if (SingleOpen(protocol::msg::Tof::RIGHT_FRONT)) {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof_opened_right_front opened successfully");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof_opened_right_front opened failed");
  }

  if (SingleOpen(protocol::msg::Tof::RIGHT_BACK)) {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof_opened_right_back opened successfully");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof_opened_right_back opened failed");
  }
*/
  tof_pub_thread = std::thread(std::bind(&cyberdog::sensor::TofCarpo::tof_pub_callback, this));
  opened_ = tof_opened_left_back && tof_opened_left_back &&
    tof_opened_right_front && tof_opened_right_back;
  RCLCPP_INFO(
    rclcpp::get_logger("cyberdog_tof"), "tofs opened status = %d ",
    static_cast<int>(opened_));
  return opened_;
}


bool cyberdog::sensor::TofCarpo::Start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("cyberdog_tof"), "tof started status = %d ",
    static_cast<int>(opened_));
  return opened_;
}

bool cyberdog::sensor::TofCarpo::Stop()
{
  if (SingleStop(protocol::msg::Tof::LEFT_FRONT)) {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof left front stoped successfully");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof left front stoped failed");
  }
  if (SingleStop(protocol::msg::Tof::LEFT_BACK)) {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof left back stoped successfully");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof left back stoped failed");
  }
  if (SingleStop(protocol::msg::Tof::RIGHT_FRONT)) {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof right front stoped successfully");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof right front stoped failed");
  }
  if (SingleStop(protocol::msg::Tof::RIGHT_BACK)) {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof right back stoped successfully");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof right back stoped failed");
  }
  closed_ = (tof_started_left_front || tof_started_left_back ||
    tof_started_right_front || tof_started_right_back);
  return closed_;
}

bool cyberdog::sensor::TofCarpo::Close()
{
  if (closed_ == true) {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof closed successfully");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "tof closed failed");
  }
  return closed_;
}

bool cyberdog::sensor::TofCarpo::SingleStop(uint8_t serial_number)
{
  switch (serial_number) {
    // left_front
    case protocol::msg::Tof::LEFT_FRONT: {
        tof_can_left_front->Operate(
          "enable_off", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00});
        tof_can_left_front->BREAK_VAR(tof_can_left_front->GetData()->tof_data);
        tof_can_left_front->BREAK_VAR(tof_can_left_front->GetData()->tof_data_clock);
        tof_can_left_front->LINK_VAR(tof_can_left_front->GetData()->enable_off_ack);

        time_t time_left_front = time(nullptr);
        while (tof_started_left_front == true && difftime(time(nullptr), time_left_front) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "difftime = %2f "
          // ,difftime(time(nullptr), time_left_front));
        }

        RCLCPP_INFO(
          rclcpp::get_logger(
            "cyberdog_tof"), "tof stoped successfully !tof_started_left_front= %d ",
          static_cast<int>(!tof_started_left_front));
        return !tof_started_left_front;
      }
    // left_back
    case protocol::msg::Tof::LEFT_BACK: {
        tof_can_left_back->Operate(
          "enable_off", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00});
        tof_can_left_back->BREAK_VAR(tof_can_left_back->GetData()->tof_data);
        tof_can_left_back->BREAK_VAR(tof_can_left_back->GetData()->tof_data_clock);
        tof_can_left_back->LINK_VAR(tof_can_left_back->GetData()->enable_off_ack);

        time_t time_left_back = time(nullptr);
        while (tof_started_left_back == true && difftime(time(nullptr), time_left_back) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"),
          // "difftime = %2f ",difftime(time(nullptr), time_left_back));
        }

        RCLCPP_INFO(
          rclcpp::get_logger(
            "cyberdog_tof"), "tof stoped successfully !tof_started_left_back= %d ",
          static_cast<int>(!tof_started_left_back));
        return !tof_started_left_back;
      }
    // right_front
    case protocol::msg::Tof::RIGHT_FRONT: {
        tof_can_right_front->Operate(
          "enable_off", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00});
        tof_can_right_front->BREAK_VAR(tof_can_right_front->GetData()->tof_data);
        tof_can_right_front->BREAK_VAR(tof_can_right_front->GetData()->tof_data_clock);
        tof_can_right_front->LINK_VAR(tof_can_right_front->GetData()->enable_off_ack);

        time_t time_right_front = time(nullptr);
        while (tof_started_right_front == true &&
          difftime(time(nullptr), time_right_front) < 2.0f)
        {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "difftime = %2f "
          // ,difftime(time(nullptr), time_right_front));
        }
        RCLCPP_INFO(
          rclcpp::get_logger(
            "cyberdog_tof"), "tof stoped successfully !tof_started_right_front= %d ",
          static_cast<int>(!tof_started_right_front));
        return !tof_started_right_front;
      }
    // right_back
    case protocol::msg::Tof::RIGHT_BACK: {
        tof_can_right_back->Operate(
          "enable_off", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00});
        tof_can_right_back->BREAK_VAR(tof_can_right_back->GetData()->tof_data);
        tof_can_right_back->BREAK_VAR(tof_can_right_back->GetData()->tof_data_clock);
        tof_can_right_back->LINK_VAR(tof_can_right_back->GetData()->enable_off_ack);

        time_t time_right_back = time(nullptr);
        while (tof_started_right_back == true && difftime(time(nullptr), time_right_back) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"),
          // "difftime = %2f ",difftime(time(nullptr), time_right_back));
        }
        RCLCPP_INFO(
          rclcpp::get_logger(
            "cyberdog_tof"), "tof stoped successfully !tof_started_right_back= %d ",
          static_cast<int>(!tof_started_right_back));
        return !tof_started_right_back;
      }
    default: {
        return false;
      }
  }
}

void cyberdog::sensor::TofCarpo::tof_pub_callback()
{
  bool publish_ok = (tof_started_left_front || tof_started_left_back ||
    tof_started_right_front || tof_started_right_back);
  while (publish_ok) {
    // RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "payload_callback ");
    std::this_thread::sleep_for(std::chrono::microseconds(100000));

    if (payload_callback_ != nullptr && multiple_tof_payload != nullptr) {
      payload_callback_(multiple_tof_payload);
      RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "payload_callback is ok");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "payload_callback_failed");
    }
  }
}

bool cyberdog::sensor::TofCarpo::SingleOpen(uint8_t serial_number)
{
  switch (serial_number) {
    // left_front
    case protocol::msg::Tof::LEFT_FRONT: {
        tof_opened_left_front = false;
        auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
        auto path = local_share_dir + std::string("/toml_config/sensors/tof.toml");
        if (access(path.c_str(), F_OK) != 0) {
          RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "%s do not exist!", path.c_str());
          RCLCPP_INFO(
            rclcpp::get_logger(
              "cyberdog_tof"), "fail to open tof,tof_opened_left_front=   %d ",
            static_cast<int>(tof_opened_left_front));
          return tof_opened_left_front;
        }
        tof_can_left_front = std::make_shared<EVM::Protocol<tof_can>>(path, false);
        tof_can_left_front->SetDataCallback(
          std::bind(
            &cyberdog::sensor::TofCarpo::
            left_front_callback, this, std::placeholders::_1, std::placeholders::_2));
        tof_can_left_front->Operate(
          "enable_on", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00});
        tof_can_left_front->LINK_VAR(tof_can_left_front->GetData()->enable_on_ack);

        time_t time_left_front = time(nullptr);
        while (tof_opened_left_front == false && difftime(time(nullptr), time_left_front) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"),
          // "difftime = %2f ",difftime(time(nullptr), time_left_front));
        }

        RCLCPP_INFO(
          rclcpp::get_logger(
            "cyberdog_tof"), "tof opened successfully tof_opened_left_front= %d ",
          static_cast<int>(tof_opened_left_front));
        return tof_opened_left_front;
      }
    // left_back
    case protocol::msg::Tof::LEFT_BACK: {
        tof_opened_left_back = false;
        auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
        auto path = local_share_dir + std::string("/toml_config/sensors/tof.toml");
        if (access(path.c_str(), F_OK) != 0) {
          RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "%s do not exist!", path.c_str());
          RCLCPP_INFO(
            rclcpp::get_logger(
              "cyberdog_tof"), "fail to open tof,tof_opened_left_back=   %d ",
            static_cast<int>(tof_opened_left_back));
          return tof_opened_left_back;
        }
        tof_can_left_back = std::make_shared<EVM::Protocol<tof_can>>(path, false);
        tof_can_left_back->SetDataCallback(
          std::bind(
            &cyberdog::sensor::TofCarpo::left_back_callback,
            this, std::placeholders::_1, std::placeholders::_2));
        tof_can_left_back->Operate(
          "enable_on", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00});
        tof_can_left_back->LINK_VAR(tof_can_left_back->GetData()->enable_on_ack);
        // tof_can_left_back->LINK_VAR(tof_can_left_back->GetData()->tof_data_array);
        // tof_can_left_back->LINK_VAR(tof_can_left_back->GetData()->tof_data_clock);
        time_t time_left_back = time(nullptr);
        while (tof_opened_left_back == false && difftime(time(nullptr), time_left_back) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"),
          // "difftime = %2f ",difftime(time(nullptr), time_left_back));
        }
        RCLCPP_INFO(
          rclcpp::get_logger(
            "cyberdog_tof"), "tof opened successfully tof_opened_left_back= %d ",
          static_cast<int>(tof_opened_left_back));
        return tof_opened_left_back;
      }
    // right_front
    case protocol::msg::Tof::RIGHT_FRONT: {
        tof_opened_right_front = false;
        auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
        auto path = local_share_dir + std::string("/toml_config/sensors/tof.toml");
        if (access(path.c_str(), F_OK) != 0) {
          RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "%s do not exist!", path.c_str());
          RCLCPP_INFO(
            rclcpp::get_logger(
              "cyberdog_tof"), "fail to open tof,tof_opened_right_front=   %d ",
            static_cast<int>(tof_opened_right_front));
          return tof_opened_right_front;
        }
        tof_can_right_front = std::make_shared<EVM::Protocol<tof_can>>(path, false);
        tof_can_right_front->SetDataCallback(
          std::bind(
            &cyberdog::sensor::TofCarpo::
            right_front_callback, this, std::placeholders::_1, std::placeholders::_2));
        tof_can_right_front->Operate(
          "enable_on", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00});
        tof_can_right_front->LINK_VAR(tof_can_right_front->GetData()->enable_on_ack);
        // tof_can_right_front->LINK_VAR(tof_can_right_front->GetData()->tof_data_array);
        // tof_can_right_front->LINK_VAR(tof_can_right_front->GetData()->tof_data_clock);
        time_t time_right_front = time(nullptr);
        while (tof_opened_right_front == false &&
          difftime(time(nullptr), time_right_front) < 2.0f)
        {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"),
          // "difftime = %2f ",difftime(time(nullptr), time_right_front));
        }
        RCLCPP_INFO(
          rclcpp::get_logger(
            "cyberdog_tof"), "tof opened successfully tof_opened_right_front= %d ",
          static_cast<int>(tof_opened_right_front));
        return tof_opened_right_front;
      }
    // right_back
    case protocol::msg::Tof::RIGHT_BACK: {
        tof_opened_right_back = false;
        auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
        auto path = local_share_dir + std::string("/toml_config/sensors/tof.toml");
        if (access(path.c_str(), F_OK) != 0) {
          RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"), "%s do not exist!", path.c_str());
          RCLCPP_INFO(
            rclcpp::get_logger(
              "cyberdog_tof"), "fail to open tof,tof_opened_right_back=   %d ",
            static_cast<int>(tof_opened_right_back));
          return tof_opened_right_back;
        }
        tof_can_right_back = std::make_shared<EVM::Protocol<tof_can>>(path, false);
        tof_can_right_back->SetDataCallback(
          std::bind(
            &cyberdog::sensor::TofCarpo::
            right_back_callback, this, std::placeholders::_1, std::placeholders::_2));
        tof_can_right_back->Operate(
          "enable_on", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00});
        tof_can_right_back->LINK_VAR(tof_can_right_back->GetData()->enable_on_ack);
        // tof_can_right_back->LINK_VAR(tof_can_right_back->GetData()->tof_data_array);
        // tof_can_right_back->LINK_VAR(tof_can_right_back->GetData()->tof_data_clock);
        time_t time_right_back = time(nullptr);
        while (tof_opened_right_back == false && difftime(time(nullptr), time_right_back) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"),
          // "difftime = %2f ",difftime(time(nullptr), time_right_back));
        }
        RCLCPP_INFO(
          rclcpp::get_logger(
            "cyberdog_tof"), "tof opened successfully tof_opened_right_back= %d ",
          static_cast<int>(tof_opened_right_back));
        return tof_opened_right_back;
      }
    default: {
        return false;
      }
  }
}

void cyberdog::sensor::TofCarpo::left_front_callback(
  std::string & name,
  std::shared_ptr<cyberdog::sensor::tof_can> data)
{
  if (name == "enable_on_ack") {
    printf("I heard name:%s, data: '%02x'\n", name.c_str(), data->enable_on_ack);
    tof_opened_left_front = true;
    tof_can_left_front->BREAK_VAR(tof_can_left_front->GetData()->enable_on_ack);
    tof_can_left_front->LINK_VAR(tof_can_left_front->GetData()->tof_data_array);
    tof_can_left_front->LINK_VAR(tof_can_left_front->GetData()->tof_data_clock);

  } else if (name == "tof_data_array") {
    tof_started_left_front = true;
    std::string out_put("");
    std::cout << "tof_data_array   " << int(data->tof_data_array[0]) << std::endl;
  } else if (name == "tof_data_clock") {
    tof_started_left_front = true;
    std::string out_put("");
    std::cout << "tof_data_array " << int(data->tof_data_array[0]) << std::endl;
    std::cout << "left_front tof_data_clock= " << data->tof_data_clock << std::endl;
  } else if (name == "enable_off_ack") {
    printf("I heard name:%s, data: '%02x'\n", name.c_str(), data->enable_on_ack);
    tof_started_left_front = false;
  }

  const int datanum = protocol::msg::Tof::TOF_DATA_NUM;
  std::vector<float> obj;
  for (size_t i = 0; i < datanum; i++) {
    obj.push_back((data->tof_data_array[i] * 2.0f + 150) * protocol::msg::Tof::SCALE_FACTOR);
  }
  auto tof_payload = std::make_shared<protocol::msg::Tof>();

  struct timespec time_stu;
  clock_gettime(CLOCK_REALTIME, &time_stu);
  tof_payload->header.frame_id = std::string("tof_left_front");
  tof_payload->header.stamp.nanosec = time_stu.tv_nsec;
  tof_payload->header.stamp.sec = time_stu.tv_sec;
  tof_payload->tof_postion = protocol::msg::Tof::LEFT_FRONT;
  tof_payload->data = obj;
  tof_payload->data_available = tof_opened_left_front;

  std::cout << "tof_payload->header.stamp.nanosec= " << tof_payload->header.stamp.nanosec <<
    std::endl;
  std::cout << "tof_payload->header.stamp.sec= " << tof_payload->header.stamp.sec << std::endl;
  multiple_tof_payload->left_front = *tof_payload;
}

void cyberdog::sensor::TofCarpo::left_back_callback(
  std::string & name,
  std::shared_ptr<cyberdog::sensor::tof_can> data)
{
  if (name == "enable_on_ack") {
    printf("I heard name:%s, data: '%02x'\n", name.c_str(), data->enable_on_ack);
    tof_opened_left_back = true;
    tof_can_left_back->BREAK_VAR(tof_can_left_back->GetData()->enable_on_ack);
    tof_can_left_back->LINK_VAR(tof_can_left_back->GetData()->tof_data_array);
    tof_can_left_back->LINK_VAR(tof_can_left_back->GetData()->tof_data_clock);
  } else if (name == "tof_data_array") {
    tof_started_left_back = true;
    std::string out_put("");
    std::cout << "tof_data_array   " << int(data->tof_data_array[0]) << std::endl;
  } else if (name == "tof_data_clock") {
    tof_started_left_back = true;
    std::string out_put("");
    std::cout << "tof_data_array " << int(data->tof_data_array[0]) << std::endl;
    std::cout << "left_back tof_data_clock= " << data->tof_data_clock << std::endl;
  } else if (name == "enable_off_ack") {
    printf("I heard name:%s, data: '%02x'\n", name.c_str(), data->enable_on_ack);
    tof_started_left_back = false;
  }

  const int datanum = protocol::msg::Tof::TOF_DATA_NUM;
  std::vector<float> obj;
  for (size_t i = 0; i < datanum; i++) {
    obj.push_back(data->tof_data_array[i] * protocol::msg::Tof::SCALE_FACTOR);
  }
  auto tof_payload = std::make_shared<protocol::msg::Tof>();
  struct timespec time_stu;
  clock_gettime(CLOCK_REALTIME, &time_stu);
  tof_payload->header.frame_id = std::string("tof_left_back");
  tof_payload->header.stamp.nanosec = time_stu.tv_nsec;
  tof_payload->header.stamp.sec = time_stu.tv_sec;
  tof_payload->tof_postion = protocol::msg::Tof::LEFT_BACK;
  tof_payload->data = obj;
  tof_payload->data_available = tof_opened_left_back;

  std::cout << "tof_payload->header.stamp.nanosec= " << tof_payload->header.stamp.nanosec <<
    std::endl;
  std::cout << "tof_payload->header.stamp.sec= " << tof_payload->header.stamp.sec << std::endl;

  multiple_tof_payload->left_back = *tof_payload;
}

void cyberdog::sensor::TofCarpo::right_front_callback(
  std::string & name,
  std::shared_ptr<cyberdog::sensor::tof_can> data)
{
  if (name == "enable_on_ack") {
    printf("I heard name:%s, data: '%02x'\n", name.c_str(), data->enable_on_ack);
    tof_opened_right_front = true;
    tof_can_right_front->BREAK_VAR(tof_can_right_front->GetData()->enable_on_ack);
    tof_can_right_front->LINK_VAR(tof_can_right_front->GetData()->tof_data_array);
    tof_can_right_front->LINK_VAR(tof_can_right_front->GetData()->tof_data_clock);
  } else if (name == "tof_data_array") {
    tof_started_right_front = true;
    std::string out_put("");
    std::cout << "tof_data_array   " << int(data->tof_data_array[0]) << std::endl;
  } else if (name == "tof_data_clock") {
    tof_started_right_front = true;
    std::string out_put("");
    std::cout << "tof_data_array " << int(data->tof_data_array[0]) << std::endl;
    std::cout << "right_front tof_data_clock= " << data->tof_data_clock << std::endl;
  } else if (name == "enable_off_ack") {
    printf("I heard name:%s, data: '%02x'\n", name.c_str(), data->enable_on_ack);
    tof_started_right_front = false;
  }

  const int datanum = protocol::msg::Tof::TOF_DATA_NUM;
  std::vector<float> obj;
  for (size_t i = 0; i < datanum; i++) {
    obj.push_back(data->tof_data_array[i] * protocol::msg::Tof::SCALE_FACTOR);
  }
  auto tof_payload = std::make_shared<protocol::msg::Tof>();
  struct timespec time_stu;
  clock_gettime(CLOCK_REALTIME, &time_stu);
  tof_payload->header.frame_id = std::string("tof_right_front");
  tof_payload->header.stamp.nanosec = time_stu.tv_nsec;
  tof_payload->header.stamp.sec = time_stu.tv_sec;
  tof_payload->tof_postion = protocol::msg::Tof::RIGHT_FRONT;
  tof_payload->data = obj;
  tof_payload->data_available = tof_opened_right_front;

  std::cout << "tof_payload->header.stamp.nanosec= " << tof_payload->header.stamp.nanosec <<
    std::endl;
  std::cout << "tof_payload->header.stamp.sec= " << tof_payload->header.stamp.sec << std::endl;

  multiple_tof_payload->right_front = *tof_payload;
}

void cyberdog::sensor::TofCarpo::right_back_callback(
  std::string & name,
  std::shared_ptr<cyberdog::sensor::tof_can> data)
{
  if (name == "enable_on_ack") {
    printf("I heard name:%s, data: '%02x'\n", name.c_str(), data->enable_on_ack);
    tof_opened_right_back = true;
    tof_can_right_front->BREAK_VAR(tof_can_right_front->GetData()->enable_on_ack);
    tof_can_right_front->LINK_VAR(tof_can_right_front->GetData()->tof_data_array);
    tof_can_right_front->LINK_VAR(tof_can_right_front->GetData()->tof_data_clock);
  } else if (name == "tof_data_array") {
    tof_started_right_back = true;
    std::string out_put("");
    std::cout << "tof_data_array   " << int(data->tof_data_array[0]) << std::endl;
  } else if (name == "tof_data_clock") {
    tof_started_right_back = true;
    std::string out_put("");
    std::cout << "tof_data_array " << int(data->tof_data_array[0]) << std::endl;
    std::cout << "right_back tof_data_clock= " << data->tof_data_clock << std::endl;
  } else if (name == "enable_off_ack") {
    printf("I heard name:%s, data: '%02x'\n", name.c_str(), data->enable_on_ack);
    tof_started_right_back = false;
  }
  const int datanum = protocol::msg::Tof::TOF_DATA_NUM;
  std::vector<float> obj;
  for (size_t i = 0; i < datanum; i++) {
    obj.push_back(data->tof_data_array[i] * protocol::msg::Tof::SCALE_FACTOR);
  }
  auto tof_payload = std::make_shared<protocol::msg::Tof>();
  struct timespec time_stu;
  clock_gettime(CLOCK_REALTIME, &time_stu);
  tof_payload->header.frame_id = std::string("tof_right_back");
  tof_payload->header.stamp.nanosec = time_stu.tv_nsec;
  tof_payload->header.stamp.sec = time_stu.tv_sec;
  tof_payload->tof_postion = protocol::msg::Tof::RIGHT_BACK;
  tof_payload->data = obj;
  tof_payload->data_available = tof_opened_right_back;

  std::cout << "tof_payload->header.stamp.nanosec= " << tof_payload->header.stamp.nanosec <<
    std::endl;
  std::cout << "tof_payload->header.stamp.sec= " << tof_payload->header.stamp.sec << std::endl;

  multiple_tof_payload->right_back = *tof_payload;
}

PLUGINLIB_EXPORT_CLASS(cyberdog::sensor::TofCarpo, cyberdog::sensor::TofBase)
