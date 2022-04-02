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


bool cyberdog::sensor::TofCarpo::Open()
{
  multiple_tof_payload = std::make_shared<protocol::msg::MultipleTofPayload>();

  if (SingleOpen(protocol::msg::SingleTofPayload::LEFT_FRONT)) {
    INFO("tof_opened_left_front opened successfully");

  } else {
    FATAL("tof_opened_left_front opened failed");
  }
/*
  if (SingleOpen(protocol::msg::SingleTofPayload::LEFT_BACK)) {
    INFO("tof_opened_left_back opened successfully");
  } else {
    FATAL("tof_opened_left_back opened failed");
  }

  if (SingleOpen(protocol::msg::SingleTofPayload::RIGHT_FRONT)) {
    INFO("tof_opened_right_front opened successfully");
  } else {
    FATAL("tof_opened_right_front opened failed");
  }

  if (SingleOpen(protocol::msg::SingleTofPayload::RIGHT_BACK)) {
    INFO("tof_opened_right_back opened successfully");
  } else {
    FATAL("tof_opened_right_back opened failed");
  }
*/
  tof_pub_thread = std::thread(std::bind(&cyberdog::sensor::TofCarpo::tof_pub_callback, this));
  opened_ = tof_opened_left_back && tof_opened_left_back &&
    tof_opened_right_front && tof_opened_right_back;
  INFO(
    "all tofs opened status = %d ",
    static_cast<int>(opened_));
  return opened_;
}


bool cyberdog::sensor::TofCarpo::Start()
{
  if (SingleStart(protocol::msg::SingleTofPayload::LEFT_FRONT)) {
    INFO("tof left front started successfully");
  } else {
    FATAL("tof left front started failed");
  }
  /*
  if (SingleStart(protocol::msg::SingleTofPayload::LEFT_BACK)) {
    INFO("tof left back started successfully");
  } else {
    FATAL("tof left back started failed");
  }
  if (SingleStart(protocol::msg::SingleTofPayload::RIGHT_FRONT)) {
    INFO("tof right front started successfully");
  } else {
    FATAL("tof right front started failed");
  }
  if (SingleStart(protocol::msg::SingleTofPayload::RIGHT_BACK)) {
    INFO("tof right back started successfully");
  } else {
    FATAL("tof right back started failed");
  }
  */
  started_ = (tof_started_left_front || tof_started_left_back ||
    tof_started_right_front || tof_started_right_back);


  return started_;
}

bool cyberdog::sensor::TofCarpo::Stop()
{
  if (SingleStop(protocol::msg::SingleTofPayload::LEFT_FRONT)) {
    INFO("tof left front stoped successfully");
  } else {
    FATAL("tof left front stoped failed");
  }
  /*
  if (SingleStop(protocol::msg::SingleTofPayload::LEFT_BACK)) {
    INFO("tof left back stoped successfully");
  } else {
    FATAL("tof left back stoped failed");
  }
  if (SingleStop(protocol::msg::SingleTofPayload::RIGHT_FRONT)) {
    INFO("tof right front stoped successfully");
  } else {
    FATAL("tof right front stoped failed");
  }
  if (SingleStop(protocol::msg::SingleTofPayload::RIGHT_BACK)) {
    INFO("tof right back stoped successfully");
  } else {
    FATAL("tof right back stoped failed");
  }
  */
  stopped_ = (tof_started_left_front || tof_started_left_back ||
    tof_started_right_front || tof_started_right_back);
  return !stopped_;
}

bool cyberdog::sensor::TofCarpo::Close()
{
  closed_ = stopped_;
  if (closed_ == true) {
    INFO("tof closed successfully");
  } else {
    FATAL("tof closed failed");
  }
  return closed_;
}


bool cyberdog::sensor::TofCarpo::SingleStart(uint8_t serial_number)
{
  switch (serial_number) {
    // left_front
    case protocol::msg::SingleTofPayload::LEFT_FRONT: {
        time_t time_started_delay = time(nullptr);
        while (tof_started_left_front == false &&
          difftime(time(nullptr), time_started_delay) < 2.0f)
        {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          INFO(
            "difftime = %2f ",
            difftime(time(nullptr), time_started_delay));
        }
        if (tof_started_left_front == false) {
          FATAL("left_front tof  Start failed ");
        } else {
          INFO("left_front tof  Start successfully ");
        }
        return tof_started_left_front;
      }
    // left_back
    case protocol::msg::SingleTofPayload::LEFT_BACK: {
        time_t time_started_delay = time(nullptr);
        while (tof_started_left_back == false &&
          difftime(time(nullptr), time_started_delay) < 2.0f)
        {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          INFO(
            "difftime = %2f ",
            difftime(time(nullptr), time_started_delay));
        }
        if (tof_started_left_back == false) {
          FATAL("left_back tof  Start failed ");
        } else {
          INFO("left_back tof  Start successfully ");
        }
        return tof_started_left_back;
      }
    // right_front
    case protocol::msg::SingleTofPayload::RIGHT_FRONT: {
        time_t time_started_delay = time(nullptr);
        while (tof_started_right_front == false &&
          difftime(time(nullptr), time_started_delay) < 2.0f)
        {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          INFO(
            "difftime = %2f ",
            difftime(time(nullptr), time_started_delay));
        }
        if (tof_started_right_front == false) {
          FATAL("right_front tof  Start failed ");
        } else {
          INFO("right_front tof  Start successfully ");
        }
        return tof_started_right_front;
      }
    // right_back
    case protocol::msg::SingleTofPayload::RIGHT_BACK: {
        time_t time_started_delay = time(nullptr);
        while (tof_started_right_back == false &&
          difftime(time(nullptr), time_started_delay) < 2.0f)
        {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          INFO(
            "difftime = %2f ",
            difftime(time(nullptr), time_started_delay));
        }
        if (tof_started_right_back == false) {
          FATAL("right_back tof  Start failed ");
        } else {
          INFO("right_back tof  Start successfully ");
        }
        return tof_started_right_back;
      }
    default: {
        return false;
      }
  }
}


bool cyberdog::sensor::TofCarpo::SingleStop(uint8_t serial_number)
{
  std::this_thread::sleep_for(std::chrono::microseconds(10000000));
  switch (serial_number) {
    // left_front
    case protocol::msg::SingleTofPayload::LEFT_FRONT: {
        tof_can_left_front->Operate(
          "enable_off", std::vector<uint8_t>{});
        tof_can_left_front->BREAK_VAR(tof_can_left_front->GetData()->tof_data);
        tof_can_left_front->BREAK_VAR(tof_can_left_front->GetData()->tof_data_clock);
        tof_can_left_front->LINK_VAR(tof_can_left_front->GetData()->enable_off_ack);

        time_t time_left_front = time(nullptr);
        while (tof_started_left_front == true && difftime(time(nullptr), time_left_front) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // INFO("difftime = %2f "
          // ,difftime(time(nullptr), time_left_front));
        }

        INFO(
          "tof stoped successfully !tof_started_left_front= %d ",
          static_cast<int>(!tof_started_left_front));
        return !tof_started_left_front;
      }
    // left_back
    case protocol::msg::SingleTofPayload::LEFT_BACK: {
        tof_can_left_back->Operate(
          "enable_off", std::vector<uint8_t>{});
        tof_can_left_back->BREAK_VAR(tof_can_left_back->GetData()->tof_data);
        tof_can_left_back->BREAK_VAR(tof_can_left_back->GetData()->tof_data_clock);
        tof_can_left_back->LINK_VAR(tof_can_left_back->GetData()->enable_off_ack);

        time_t time_left_back = time(nullptr);
        while (tof_started_left_back == true && difftime(time(nullptr), time_left_back) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"),
          // "difftime = %2f ",difftime(time(nullptr), time_left_back));
        }

        INFO(
          "tof stoped successfully !tof_started_left_back= %d ",
          static_cast<int>(!tof_started_left_back));
        return !tof_started_left_back;
      }
    // right_front
    case protocol::msg::SingleTofPayload::RIGHT_FRONT: {
        tof_can_right_front->Operate(
          "enable_off", std::vector<uint8_t>{});
        tof_can_right_front->BREAK_VAR(tof_can_right_front->GetData()->tof_data);
        tof_can_right_front->BREAK_VAR(tof_can_right_front->GetData()->tof_data_clock);
        tof_can_right_front->LINK_VAR(tof_can_right_front->GetData()->enable_off_ack);

        time_t time_right_front = time(nullptr);
        while (tof_started_right_front == true &&
          difftime(time(nullptr), time_right_front) < 2.0f)
        {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // INFO("difftime = %2f "
          // ,difftime(time(nullptr), time_right_front));
        }
        INFO(
          "tof stoped successfully !tof_started_right_front= %d ",
          static_cast<int>(!tof_started_right_front));
        return !tof_started_right_front;
      }
    // right_back
    case protocol::msg::SingleTofPayload::RIGHT_BACK: {
        tof_can_right_back->Operate(
          "enable_off", std::vector<uint8_t>{});
        tof_can_right_back->BREAK_VAR(tof_can_right_back->GetData()->tof_data);
        tof_can_right_back->BREAK_VAR(tof_can_right_back->GetData()->tof_data_clock);
        tof_can_right_back->LINK_VAR(tof_can_right_back->GetData()->enable_off_ack);

        time_t time_right_back = time(nullptr);
        while (tof_started_right_back == true && difftime(time(nullptr), time_right_back) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"),
          // "difftime = %2f ",difftime(time(nullptr), time_right_back));
        }
        INFO(
          "tof stoped successfully !tof_started_right_back= %d ",
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
    INFO("payload_callback ");
    publish_ok = (tof_started_left_front || tof_started_left_back ||
      tof_started_right_front || tof_started_right_back);
    std::this_thread::sleep_for(std::chrono::microseconds(100000));

    if (payload_callback_ != nullptr && multiple_tof_payload != nullptr && publish_ok) {
      payload_callback_(multiple_tof_payload);
      INFO("payload_callback is ok");
      // tof_started_left_front = false;
      // tof_started_left_back = false;
      // tof_started_right_front = false;
      // tof_started_right_back = false;
    } else {
      ERROR("payload_callback_failed");
    }
  }
}

bool cyberdog::sensor::TofCarpo::SingleOpen(uint8_t serial_number)
{
  switch (serial_number) {
    // left_front
    case protocol::msg::SingleTofPayload::LEFT_FRONT: {
        tof_opened_left_front = false;
        auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
        auto path = local_share_dir + std::string("/toml_config/sensors/tof.toml");
        if (access(path.c_str(), F_OK) != 0) {
          ERROR("%s do not exist!", path.c_str());
          ERROR(
            "fail to open tof,tof_opened_left_front=   %d ",
            static_cast<int>(tof_opened_left_front));
          return tof_opened_left_front;
        }
        tof_can_left_front = std::make_shared<EVM::Protocol<tof_can>>(path, false);
        tof_can_left_front->SetDataCallback(
          std::bind(
            &cyberdog::sensor::TofCarpo::
            left_front_callback, this, std::placeholders::_1, std::placeholders::_2));
        tof_can_left_front->Operate(
          "enable_on", std::vector<uint8_t>{});
        tof_can_left_front->LINK_VAR(tof_can_left_front->GetData()->enable_on_ack);

        time_t time_left_front = time(nullptr);
        while (tof_opened_left_front == false && difftime(time(nullptr), time_left_front) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"),
          // "difftime = %2f ",difftime(time(nullptr), time_left_front));
        }

        INFO(
          "tof opened successfully tof_opened_left_front= %d ",
          static_cast<int>(tof_opened_left_front));
        return tof_opened_left_front;
      }
    // left_back
    case protocol::msg::SingleTofPayload::LEFT_BACK: {
        tof_opened_left_back = false;
        auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
        auto path = local_share_dir + std::string("/toml_config/sensors/tof.toml");
        if (access(path.c_str(), F_OK) != 0) {
          ERROR("%s do not exist!", path.c_str());
          ERROR(
            "fail to open tof,tof_opened_left_back=   %d ",
            static_cast<int>(tof_opened_left_back));
          return tof_opened_left_back;
        }
        tof_can_left_back = std::make_shared<EVM::Protocol<tof_can>>(path, false);
        tof_can_left_back->SetDataCallback(
          std::bind(
            &cyberdog::sensor::TofCarpo::left_back_callback,
            this, std::placeholders::_1, std::placeholders::_2));
        tof_can_left_back->Operate(
          "enable_on", std::vector<uint8_t>{});
        tof_can_left_back->LINK_VAR(tof_can_left_back->GetData()->enable_on_ack);
        // tof_can_left_back->LINK_VAR(tof_can_left_back->GetData()->tof_data_array);
        // tof_can_left_back->LINK_VAR(tof_can_left_back->GetData()->tof_data_clock);
        time_t time_left_back = time(nullptr);
        while (tof_opened_left_back == false && difftime(time(nullptr), time_left_back) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"),
          // "difftime = %2f ",difftime(time(nullptr), time_left_back));
        }
        INFO(
          "tof opened successfully tof_opened_left_back= %d ",
          static_cast<int>(tof_opened_left_back));
        return tof_opened_left_back;
      }
    // right_front
    case protocol::msg::SingleTofPayload::RIGHT_FRONT: {
        tof_opened_right_front = false;
        auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
        auto path = local_share_dir + std::string("/toml_config/sensors/tof.toml");
        if (access(path.c_str(), F_OK) != 0) {
          ERROR("%s do not exist!", path.c_str());
          ERROR(
            "fail to open tof,tof_opened_right_front=   %d ",
            static_cast<int>(tof_opened_right_front));
          return tof_opened_right_front;
        }
        tof_can_right_front = std::make_shared<EVM::Protocol<tof_can>>(path, false);
        tof_can_right_front->SetDataCallback(
          std::bind(
            &cyberdog::sensor::TofCarpo::
            right_front_callback, this, std::placeholders::_1, std::placeholders::_2));
        tof_can_right_front->Operate(
          "enable_on", std::vector<uint8_t>{});
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
        INFO(
          "tof opened successfully tof_opened_right_front= %d ",
          static_cast<int>(tof_opened_right_front));
        return tof_opened_right_front;
      }
    // right_back
    case protocol::msg::SingleTofPayload::RIGHT_BACK: {
        tof_opened_right_back = false;
        auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
        auto path = local_share_dir + std::string("/toml_config/sensors/tof.toml");
        if (access(path.c_str(), F_OK) != 0) {
          ERROR("%s do not exist!", path.c_str());
          ERROR(
            "fail to open tof,tof_opened_right_back=   %d ",
            static_cast<int>(tof_opened_right_back));
          return tof_opened_right_back;
        }
        tof_can_right_back = std::make_shared<EVM::Protocol<tof_can>>(path, false);
        tof_can_right_back->SetDataCallback(
          std::bind(
            &cyberdog::sensor::TofCarpo::
            right_back_callback, this, std::placeholders::_1, std::placeholders::_2));
        tof_can_right_back->Operate(
          "enable_on", std::vector<uint8_t>{});
        tof_can_right_back->LINK_VAR(tof_can_right_back->GetData()->enable_on_ack);
        // tof_can_right_back->LINK_VAR(tof_can_right_back->GetData()->tof_data_array);
        // tof_can_right_back->LINK_VAR(tof_can_right_back->GetData()->tof_data_clock);
        time_t time_right_back = time(nullptr);
        while (tof_opened_right_back == false && difftime(time(nullptr), time_right_back) < 2.0f) {
          std::this_thread::sleep_for(std::chrono::microseconds(30000));
          // RCLCPP_INFO(rclcpp::get_logger("cyberdog_tof"),
          // "difftime = %2f ",difftime(time(nullptr), time_right_back));
        }
        INFO(
          "tof opened successfully tof_opened_right_back= %d ",
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
    INFO_STREAM("I heard name left_front " << name);
    tof_opened_left_front = true;
    tof_can_left_front->BREAK_VAR(tof_can_left_front->GetData()->enable_on_ack);
    tof_can_left_front->LINK_VAR(tof_can_left_front->GetData()->tof_data_array);
    tof_can_left_front->LINK_VAR(tof_can_left_front->GetData()->tof_data_clock);

  } else if (name == "tof_data_array") {
    tof_started_left_front = true;
    DEBUG_STREAM("tof_data_array   " << data->tof_data_array[0]);
  } else if (name == "tof_data_clock") {
    tof_started_left_front = true;
    DEBUG_STREAM("tof_data_array " << data->tof_data_array[0]);
    DEBUG_STREAM("left_front tof_data_clock= " << data->tof_data_clock);
  } else if (name == "enable_off_ack") {
    INFO_STREAM("I heard name left_front " << name);
    tof_started_left_front = false;
    tof_opened_left_front = false;
  }

  const int datanum = protocol::msg::SingleTofPayload::TOF_DATA_NUM;
  std::vector<float> obj;
  for (size_t i = 0; i < datanum; i++) {
    obj.push_back(
      (data->tof_data_array[i] * 2.0f + 150) * protocol::msg::SingleTofPayload::SCALE_FACTOR);
  }
  auto tof_payload = std::make_shared<protocol::msg::SingleTofPayload>();

  struct timespec time_stu;
  clock_gettime(CLOCK_REALTIME, &time_stu);
  tof_payload->header.frame_id = std::string("tof_left_front");
  tof_payload->header.stamp.nanosec = time_stu.tv_nsec;
  tof_payload->header.stamp.sec = time_stu.tv_sec;
  tof_payload->tof_position = protocol::msg::SingleTofPayload::LEFT_FRONT;
  tof_payload->data = obj;
  tof_payload->data_available = tof_opened_left_front;
  multiple_tof_payload->left_front = *tof_payload;
}

void cyberdog::sensor::TofCarpo::left_back_callback(
  std::string & name,
  std::shared_ptr<cyberdog::sensor::tof_can> data)
{
  if (name == "enable_on_ack") {
    INFO_STREAM("I heard name left_back " << name);
    tof_opened_left_back = true;
    tof_can_left_back->BREAK_VAR(tof_can_left_back->GetData()->enable_on_ack);
    tof_can_left_back->LINK_VAR(tof_can_left_back->GetData()->tof_data_array);
    tof_can_left_back->LINK_VAR(tof_can_left_back->GetData()->tof_data_clock);
  } else if (name == "tof_data_array") {
    tof_started_left_back = true;
  } else if (name == "tof_data_clock") {
    tof_started_left_back = true;
  } else if (name == "enable_off_ack") {
    INFO_STREAM("I heard name left_back " << name);
    tof_started_left_back = false;
    tof_opened_left_back = false;
  }

  const int datanum = protocol::msg::SingleTofPayload::TOF_DATA_NUM;
  std::vector<float> obj;
  for (size_t i = 0; i < datanum; i++) {
    obj.push_back(data->tof_data_array[i] * protocol::msg::SingleTofPayload::SCALE_FACTOR);
  }
  auto tof_payload = std::make_shared<protocol::msg::SingleTofPayload>();
  struct timespec time_stu;
  clock_gettime(CLOCK_REALTIME, &time_stu);
  tof_payload->header.frame_id = std::string("tof_left_back");
  tof_payload->header.stamp.nanosec = time_stu.tv_nsec;
  tof_payload->header.stamp.sec = time_stu.tv_sec;
  tof_payload->tof_position = protocol::msg::SingleTofPayload::LEFT_BACK;
  tof_payload->data = obj;
  tof_payload->data_available = tof_opened_left_back;
  multiple_tof_payload->left_back = *tof_payload;
}

void cyberdog::sensor::TofCarpo::right_front_callback(
  std::string & name,
  std::shared_ptr<cyberdog::sensor::tof_can> data)
{
  if (name == "enable_on_ack") {
    INFO_STREAM("I heard name right_front " << name);
    tof_opened_right_front = true;
    tof_can_right_front->BREAK_VAR(tof_can_right_front->GetData()->enable_on_ack);
    tof_can_right_front->LINK_VAR(tof_can_right_front->GetData()->tof_data_array);
    tof_can_right_front->LINK_VAR(tof_can_right_front->GetData()->tof_data_clock);
  } else if (name == "tof_data_array") {
    tof_started_right_front = true;
  } else if (name == "tof_data_clock") {
    tof_started_right_front = true;
  } else if (name == "enable_off_ack") {
    INFO_STREAM("I heard name right_front " << name);
    tof_started_right_front = false;
    tof_opened_right_front = false;
  }

  const int datanum = protocol::msg::SingleTofPayload::TOF_DATA_NUM;
  std::vector<float> obj;
  for (size_t i = 0; i < datanum; i++) {
    obj.push_back(data->tof_data_array[i] * protocol::msg::SingleTofPayload::SCALE_FACTOR);
  }
  auto tof_payload = std::make_shared<protocol::msg::SingleTofPayload>();
  struct timespec time_stu;
  clock_gettime(CLOCK_REALTIME, &time_stu);
  tof_payload->header.frame_id = std::string("tof_right_front");
  tof_payload->header.stamp.nanosec = time_stu.tv_nsec;
  tof_payload->header.stamp.sec = time_stu.tv_sec;
  tof_payload->tof_position = protocol::msg::SingleTofPayload::RIGHT_FRONT;
  tof_payload->data = obj;
  tof_payload->data_available = tof_opened_right_front;
  multiple_tof_payload->right_front = *tof_payload;
}

void cyberdog::sensor::TofCarpo::right_back_callback(
  std::string & name,
  std::shared_ptr<cyberdog::sensor::tof_can> data)
{
  if (name == "enable_on_ack") {
    INFO_STREAM("I heard name right_back " << name);
    tof_opened_right_back = true;
    tof_can_right_front->BREAK_VAR(tof_can_right_front->GetData()->enable_on_ack);
    tof_can_right_front->LINK_VAR(tof_can_right_front->GetData()->tof_data_array);
    tof_can_right_front->LINK_VAR(tof_can_right_front->GetData()->tof_data_clock);
  } else if (name == "tof_data_array") {
    tof_started_right_back = true;
  } else if (name == "tof_data_clock") {
    tof_started_right_back = true;
  } else if (name == "enable_off_ack") {
    INFO_STREAM("I heard name right_back " << name);
    tof_started_right_back = false;
    tof_opened_right_back = false;
  }
  const int datanum = protocol::msg::SingleTofPayload::TOF_DATA_NUM;
  std::vector<float> obj;
  for (size_t i = 0; i < datanum; i++) {
    obj.push_back(data->tof_data_array[i] * protocol::msg::SingleTofPayload::SCALE_FACTOR);
  }
  auto tof_payload = std::make_shared<protocol::msg::SingleTofPayload>();
  struct timespec time_stu;
  clock_gettime(CLOCK_REALTIME, &time_stu);
  tof_payload->header.frame_id = std::string("tof_right_back");
  tof_payload->header.stamp.nanosec = time_stu.tv_nsec;
  tof_payload->header.stamp.sec = time_stu.tv_sec;
  tof_payload->tof_position = protocol::msg::SingleTofPayload::RIGHT_BACK;
  tof_payload->data = obj;
  tof_payload->data_available = tof_opened_right_back;
  multiple_tof_payload->right_back = *tof_payload;
}

PLUGINLIB_EXPORT_CLASS(cyberdog::sensor::TofCarpo, cyberdog::sensor::TofBase)
