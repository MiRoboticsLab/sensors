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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "lidar_plugin/ydlidar_plugin.hpp"

bool cyberdog::sensor::YdlidarCarpo::Open()
{
  INFO("Open ydlidar ...");
  this->lidar_ptr_ = std::make_shared<CYdLidar>();
  this->scan_ptr_ = std::make_shared<ScanMsg>();

  std::string lidar_config_dir = ament_index_cpp::get_package_share_directory("params") +
    "/toml_config/sensors/lidar.toml";
  INFO("Params config file dir:%s", lidar_config_dir.c_str());
  if (access(lidar_config_dir.c_str(), F_OK)) {
    ERROR("Params config file does not exist");
    return false;
  }

  if (access(lidar_config_dir.c_str(), R_OK)) {
    ERROR("Params config file does not have read permissions");
    return false;
  }

  if (access(lidar_config_dir.c_str(), W_OK)) {
    ERROR("Params config file does not have write permissions");
    return false;
  }

  if (!cyberdog::common::CyberdogToml::ParseFile(
      lidar_config_dir.c_str(), this->params_toml_))
  {
    ERROR("Params config file is not in toml format");
    return false;
  }

  this->scan_ptr_->header.frame_id = "laser_frame";
  this->scan_ptr_->header.frame_id = toml::find<std::string>(
    this->params_toml_, "dylidar", "frame_id");

  std::string str_optvalue;
  str_optvalue = "/dev/ydlidar";
  str_optvalue = toml::find<std::string>(this->params_toml_, "dylidar", "port");
  this->lidar_ptr_->setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

  str_optvalue = "";
  str_optvalue = toml::find<std::string>(this->params_toml_, "dylidar", "ignore_array");
  this->lidar_ptr_->setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

  int int_optvalue;
  int_optvalue = 230400;
  int_optvalue = toml::find<int>(this->params_toml_, "dylidar", "baudrate");
  this->lidar_ptr_->setlidaropt(LidarPropSerialBaudrate, &int_optvalue, sizeof(int));

  int_optvalue = TYPE_TRIANGLE;
  int_optvalue = toml::find<int>(this->params_toml_, "dylidar", "lidar_type");
  this->lidar_ptr_->setlidaropt(LidarPropLidarType, &int_optvalue, sizeof(int));

  int_optvalue = YDLIDAR_TYPE_SERIAL;
  int_optvalue = toml::find<int>(this->params_toml_, "dylidar", "device_type");
  this->lidar_ptr_->setlidaropt(LidarPropDeviceType, &int_optvalue, sizeof(int));

  int_optvalue = 9;
  int_optvalue = toml::find<int>(this->params_toml_, "dylidar", "sample_rate");
  this->lidar_ptr_->setlidaropt(LidarPropSampleRate, &int_optvalue, sizeof(int));

  int_optvalue = 4;
  int_optvalue = toml::find<int>(this->params_toml_, "dylidar", "abnormal_check_count");
  this->lidar_ptr_->setlidaropt(LidarPropAbnormalCheckCount, &int_optvalue, sizeof(int));

  bool bool_optvalue;
  bool_optvalue = false;
  bool_optvalue = toml::find<bool>(this->params_toml_, "dylidar", "resolution_fixed");
  this->lidar_ptr_->setlidaropt(LidarPropFixedResolution, &bool_optvalue, sizeof(bool));

  bool_optvalue = true;
  bool_optvalue = toml::find<bool>(this->params_toml_, "dylidar", "reversion");
  this->lidar_ptr_->setlidaropt(LidarPropReversion, &bool_optvalue, sizeof(bool));

  bool_optvalue = true;
  bool_optvalue = toml::find<bool>(this->params_toml_, "dylidar", "inverted");
  this->lidar_ptr_->setlidaropt(LidarPropInverted, &bool_optvalue, sizeof(bool));

  bool_optvalue = true;
  bool_optvalue = toml::find<bool>(this->params_toml_, "dylidar", "auto_reconnect");
  this->lidar_ptr_->setlidaropt(LidarPropAutoReconnect, &bool_optvalue, sizeof(bool));

  bool_optvalue = false;
  bool_optvalue = toml::find<bool>(this->params_toml_, "dylidar", "isSingleChannel");
  this->lidar_ptr_->setlidaropt(LidarPropSingleChannel, &bool_optvalue, sizeof(bool));

  bool_optvalue = false;
  bool_optvalue = toml::find<bool>(this->params_toml_, "dylidar", "intensity");
  this->lidar_ptr_->setlidaropt(LidarPropIntenstiy, &bool_optvalue, sizeof(bool));

  bool_optvalue = false;
  bool_optvalue = toml::find<bool>(this->params_toml_, "dylidar", "support_motor_dtr");
  this->lidar_ptr_->setlidaropt(LidarPropSupportMotorDtrCtrl, &bool_optvalue, sizeof(bool));

  float float_optvalue;
  float_optvalue = 180.0f;
  float_optvalue = toml::find<float>(this->params_toml_, "dylidar", "angle_max");
  this->lidar_ptr_->setlidaropt(LidarPropMaxAngle, &float_optvalue, sizeof(float));

  float_optvalue = -180.0f;
  float_optvalue = toml::find<float>(this->params_toml_, "dylidar", "angle_min");
  this->lidar_ptr_->setlidaropt(LidarPropMinAngle, &float_optvalue, sizeof(float));

  float_optvalue = 64.f;
  float_optvalue = toml::find<float>(this->params_toml_, "dylidar", "range_max");
  this->lidar_ptr_->setlidaropt(LidarPropMaxRange, &float_optvalue, sizeof(float));

  float_optvalue = 0.1f;
  float_optvalue = toml::find<float>(this->params_toml_, "dylidar", "range_min");
  this->lidar_ptr_->setlidaropt(LidarPropMinRange, &float_optvalue, sizeof(float));

  this->frequency_ = 10.f;
  this->frequency_ = toml::find<float>(this->params_toml_, "dylidar", "frequency");
  this->lidar_ptr_->setlidaropt(LidarPropScanFrequency, &this->frequency_, sizeof(float));

  if (!this->lidar_ptr_->initialize()) {
    ERROR("ydlidar initialize failed:%s", this->lidar_ptr_->DescribeError());
    return false;
  }
  this->sensor_state_ = SwitchState::open;
  INFO("ydlidar initialize ok");
  return true;
}

bool cyberdog::sensor::YdlidarCarpo::Start()
{
  INFO("Start ydlidar ...");

  if (this->sensor_state_ == SwitchState::close) {
    ERROR("ydlidar is close, unable to start");
    return false;
  }

  if (this->lidar_ptr_->turnOn()) {
    ERROR("ydlidar turnOn failed:%s", this->lidar_ptr_->DescribeError());
    return false;
  }

  if (this->sensor_state_ == SwitchState::open) {
    this->scan_ptr_ = std::make_shared<ScanMsg>();
    this->update_data_thread_ptr_ = std::make_shared<std::thread>(
      std::bind(&cyberdog::sensor::YdlidarCarpo::UpdateData, this));
  }

  this->sensor_state_ = SwitchState::start;
  INFO("ydlidar Start ok");
  return true;
}

bool cyberdog::sensor::YdlidarCarpo::Stop()
{
  INFO("Stop");
  this->sensor_state_ = SwitchState::stop;
  this->lidar_ptr_->turnOff();
  return true;
}

bool cyberdog::sensor::YdlidarCarpo::Close()
{
  INFO("Close");
  this->sensor_state_ = SwitchState::close;
  this->lidar_ptr_->turnOff();
  this->lidar_ptr_->disconnecting();
  if (this->update_data_thread_ptr_->joinable()) {
    this->update_data_thread_ptr_->join();
  }
  return true;
}

void cyberdog::sensor::YdlidarCarpo::UpdateData()
{
  int sleep_time = static_cast<int>(1000 / this->frequency_);
  while (true) {
    if (this->lidar_ptr_->doProcessSimple(this->scan_sdk)) {
      this->scan_ptr_->header.stamp.sec = RCL_NS_TO_S(this->scan_sdk.stamp);
      this->scan_ptr_->header.stamp.nanosec = this->scan_sdk.stamp -
        RCL_S_TO_NS(this->scan_ptr_->header.stamp.sec);
      this->scan_ptr_->angle_min = this->scan_sdk.config.min_angle;
      this->scan_ptr_->angle_max = this->scan_sdk.config.max_angle;
      this->scan_ptr_->angle_increment = this->scan_sdk.config.angle_increment;
      this->scan_ptr_->scan_time = this->scan_sdk.config.scan_time;
      this->scan_ptr_->time_increment = this->scan_sdk.config.time_increment;
      this->scan_ptr_->range_min = this->scan_sdk.config.min_range;
      this->scan_ptr_->range_max = this->scan_sdk.config.max_range;
      int size = (this->scan_sdk.config.max_angle - this->scan_sdk.config.min_angle) /
        this->scan_sdk.config.angle_increment + 1;
      this->scan_ptr_->ranges.resize(size);
      this->scan_ptr_->intensities.resize(size);
      for (size_t i = 0; i < this->scan_sdk.points.size(); i++) {
        int index = std::ceil(
          (this->scan_sdk.points[i].angle - this->scan_sdk.config.min_angle) /
          this->scan_sdk.config.angle_increment);
        if (index >= 0 && index < size) {
          this->scan_ptr_->ranges[index] = this->scan_sdk.points[i].range;
          this->scan_ptr_->intensities[index] = this->scan_sdk.points[i].intensity;
        }
      }
      this->payload_callback_(this->scan_ptr_);
    }

    if (!rclcpp::ok()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
  }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cyberdog::sensor::YdlidarCarpo, cyberdog::sensor::LidarBase)
