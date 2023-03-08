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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "lidar_plugin/ydlidar_plugin.hpp"

int32_t cyberdog::sensor::YdlidarCarpo::Init(bool simulator)
{
  this->state_msg_.insert({SwitchState::open, "Open"});
  this->state_msg_.insert({SwitchState::start, "Start"});
  this->state_msg_.insert({SwitchState::stop, "Stop"});
  this->state_msg_.insert({SwitchState::close, "Close"});
  const SYS::ModuleCode kModuleCode = SYS::ModuleCode::kLidar;
  code_ = std::make_shared<SYS::CyberdogCode<YdlidarCode>>(kModuleCode);

  std::string lidar_config_dir = ament_index_cpp::get_package_share_directory("params") +
    "/toml_config/sensors/lidar.toml";
  INFO("Params config file dir:%s", lidar_config_dir.c_str());

  if (access(lidar_config_dir.c_str(), F_OK)) {
    ERROR("Params config file does not exist");
    return code_->GetKeyCode(SYS::KeyCode::kFailed);
  }

  if (access(lidar_config_dir.c_str(), R_OK)) {
    ERROR("Params config file does not have read permissions");
    return code_->GetKeyCode(SYS::KeyCode::kFailed);
  }

  if (access(lidar_config_dir.c_str(), W_OK)) {
    ERROR("Params config file does not have write permissions");
    return code_->GetKeyCode(SYS::KeyCode::kFailed);
  }

  if (!cyberdog::common::CyberdogToml::ParseFile(
      lidar_config_dir.c_str(), this->params_toml_))
  {
    ERROR("Params config file is not in toml format");
    return code_->GetKeyCode(SYS::KeyCode::kFailed);
  }
  this->filter_ = toml::find_or(
    this->params_toml_, "dylidar", "filter", false);
  this->filter_ptr_ = std::make_shared<Filter>("sensor_msgs::msg::LaserScan");
  this->raw_scan_.header.frame_id = toml::find_or(
    this->params_toml_, "dylidar", "frame_id", "laser_frame");
  INFO("this->raw_scan_.header.frame_id = %s", this->raw_scan_.header.frame_id.c_str());

  if (!simulator) {
    this->Open = std::bind(&cyberdog::sensor::YdlidarCarpo::Open_, this);
    this->Start = std::bind(&cyberdog::sensor::YdlidarCarpo::Start_, this);
    this->Stop = std::bind(&cyberdog::sensor::YdlidarCarpo::Stop_, this);
    this->Close = std::bind(&cyberdog::sensor::YdlidarCarpo::Close_, this);
  } else {
    auto Simulator = [this](SwitchState now_state) -> bool {
        INFO("%s ydlidar ...", this->state_msg_[now_state].c_str());
        switch (now_state) {
          case SwitchState::open:
          case SwitchState::stop:
          case SwitchState::close:
            break;
          case SwitchState::start:
            if (this->update_data_thread_ptr_ == nullptr) {
              this->update_data_thread_ptr_ = std::make_shared<std::thread>(
                std::bind(&cyberdog::sensor::YdlidarCarpo::UpdateSimulationData, this));
              this->update_data_thread_ptr_->detach();
            }
            break;
          default:
            WARN("Ydlidar not recognized state");
            break;
        }

        this->sensor_state_ = now_state;
        INFO("ydlidar %s ok", this->state_msg_[now_state].c_str());
        return code_->GetKeyCode(SYS::KeyCode::kOK);
      };
    this->Open = std::bind(Simulator, SwitchState::open);
    this->Start = std::bind(Simulator, SwitchState::start);
    this->Stop = std::bind(Simulator, SwitchState::stop);
    this->Close = std::bind(Simulator, SwitchState::close);
  }
  return code_->GetKeyCode(SYS::KeyCode::kOK);
}

int32_t cyberdog::sensor::YdlidarCarpo::Open_()
{
  INFO("%s ydlidar ...", this->state_msg_[SwitchState::open].c_str());
  if (this->lidar_ptr_ == nullptr) {
    this->lidar_ptr_ = std::make_shared<CYdLidar>();
  }

  std::string str_optvalue;
  str_optvalue = toml::find_or(this->params_toml_, "dylidar", "port", "/dev/ydlidar");
  this->lidar_ptr_->setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());
  DEBUG("[Open] dylidar->port = %s", str_optvalue.c_str());

  str_optvalue = toml::find_or(this->params_toml_, "dylidar", "ignore_array", "");
  this->lidar_ptr_->setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());
  DEBUG("[Open] dylidar->ignore_array = %s", str_optvalue.c_str());

  int int_optvalue;
  int_optvalue = toml::find_or(this->params_toml_, "dylidar", "baudrate", 512000);
  this->lidar_ptr_->setlidaropt(LidarPropSerialBaudrate, &int_optvalue, sizeof(int));
  DEBUG("[Open] dylidar->baudrate = %d", int_optvalue);

  int_optvalue =
    toml::find_or(
    this->params_toml_, "dylidar", "lidar_type", static_cast<int>(LidarTypeID::TYPE_TOF));
  this->lidar_ptr_->setlidaropt(LidarPropLidarType, &int_optvalue, sizeof(int));
  DEBUG("[Open] dylidar->lidar_type = %d", int_optvalue);

  int_optvalue =
    toml::find_or(
    this->params_toml_, "dylidar", "device_type",
    static_cast<int>(DeviceTypeID::YDLIDAR_TYPE_SERIAL));
  this->lidar_ptr_->setlidaropt(LidarPropDeviceType, &int_optvalue, sizeof(int));
  DEBUG("[Open] dylidar->device_type = %d", int_optvalue);

  int_optvalue = toml::find_or(this->params_toml_, "dylidar", "sample_rate", 9);
  this->lidar_ptr_->setlidaropt(LidarPropSampleRate, &int_optvalue, sizeof(int));
  DEBUG("[Open] dylidar->sample_rate = %d", int_optvalue);

  int_optvalue = toml::find_or(this->params_toml_, "dylidar", "abnormal_check_count", 4);
  this->lidar_ptr_->setlidaropt(LidarPropAbnormalCheckCount, &int_optvalue, sizeof(int));
  DEBUG("[Open] dylidar->sample_rate = %d", int_optvalue);

  bool bool_optvalue;
  bool_optvalue = toml::find_or(this->params_toml_, "dylidar", "resolution_fixed", false);
  this->lidar_ptr_->setlidaropt(LidarPropFixedResolution, &bool_optvalue, sizeof(bool));
  DEBUG("[Open] dylidar->resolution_fixed = %s", bool_optvalue ? "True" : "False");

  bool_optvalue = toml::find_or(this->params_toml_, "dylidar", "reversion", true);
  this->lidar_ptr_->setlidaropt(LidarPropReversion, &bool_optvalue, sizeof(bool));
  DEBUG("[Open] dylidar->reversion = %s", bool_optvalue ? "True" : "False");

  bool_optvalue = toml::find_or(this->params_toml_, "dylidar", "inverted", true);
  this->lidar_ptr_->setlidaropt(LidarPropInverted, &bool_optvalue, sizeof(bool));
  DEBUG("[Open] dylidar->inverted = %s", bool_optvalue ? "True" : "False");

  bool_optvalue = toml::find_or(this->params_toml_, "dylidar", "auto_reconnect", true);
  this->lidar_ptr_->setlidaropt(LidarPropAutoReconnect, &bool_optvalue, sizeof(bool));
  DEBUG("[Open] dylidar->auto_reconnect = %s", bool_optvalue ? "True" : "False");

  bool_optvalue = toml::find_or(this->params_toml_, "dylidar", "isSingleChannel", false);
  this->lidar_ptr_->setlidaropt(LidarPropSingleChannel, &bool_optvalue, sizeof(bool));
  DEBUG("[Open] dylidar->isSingleChannel = %s", bool_optvalue ? "True" : "False");

  bool_optvalue = toml::find_or(this->params_toml_, "dylidar", "intensity", false);
  this->lidar_ptr_->setlidaropt(LidarPropIntenstiy, &bool_optvalue, sizeof(bool));
  DEBUG("[Open] dylidar->intensity = %s", bool_optvalue ? "True" : "False");

  bool_optvalue = toml::find_or(this->params_toml_, "dylidar", "support_motor_dtr", false);
  this->lidar_ptr_->setlidaropt(LidarPropSupportMotorDtrCtrl, &bool_optvalue, sizeof(bool));
  DEBUG("[Open] dylidar->support_motor_dtr = %s", bool_optvalue ? "True" : "False");

  float float_optvalue;
  float_optvalue =
    toml::find_or(this->params_toml_, "dylidar", "angle_max", static_cast<float>(180.0f));
  this->lidar_ptr_->setlidaropt(LidarPropMaxAngle, &float_optvalue, sizeof(float));
  DEBUG("[Open] dylidar->angle_max = %f", float_optvalue);

  float_optvalue =
    toml::find_or(this->params_toml_, "dylidar", "angle_min", static_cast<float>(-180.0f));
  this->lidar_ptr_->setlidaropt(LidarPropMinAngle, &float_optvalue, sizeof(float));
  DEBUG("[Open] dylidar->angle_min = %f", float_optvalue);

  float_optvalue =
    toml::find_or(this->params_toml_, "dylidar", "range_max", static_cast<float>(64.f));
  this->lidar_ptr_->setlidaropt(LidarPropMaxRange, &float_optvalue, sizeof(float));
  DEBUG("[Open] dylidar->range_max = %f", float_optvalue);

  float_optvalue =
    toml::find_or(this->params_toml_, "dylidar", "range_min", static_cast<float>(0.1f));
  this->lidar_ptr_->setlidaropt(LidarPropMinRange, &float_optvalue, sizeof(float));
  DEBUG("[Open] dylidar->range_min = %f", float_optvalue);

  this->frequency_ =
    toml::find_or(this->params_toml_, "dylidar", "frequency", static_cast<float>(10.f));
  this->lidar_ptr_->setlidaropt(LidarPropScanFrequency, &this->frequency_, sizeof(float));
  DEBUG("[Open] dylidar->frequency = %f", this->frequency_);

  if (!this->lidar_ptr_->initialize()) {
    ERROR(
      "Ydlidar %s failed:%s",
      this->state_msg_[SwitchState::open].c_str(), this->lidar_ptr_->DescribeError());
    this->Close_();
    return code_->GetKeyCode(SYS::KeyCode::kFailed);
  }

  this->sensor_state_ = SwitchState::open;
  INFO("Ydlidar %s ok", this->state_msg_[this->sensor_state_].c_str());
  return code_->GetKeyCode(SYS::KeyCode::kOK);
}

int32_t cyberdog::sensor::YdlidarCarpo::Start_()
{
  INFO("%s ydlidar ...", this->state_msg_[SwitchState::start].c_str());
  if (this->lidar_ptr_ == nullptr) {
    ERROR("Now Ydlidar is not yet opened.");
    return code_->GetKeyCode(SYS::KeyCode::kFailed);
  }
  if (this->sensor_state_ == SwitchState::start) {
    return code_->GetKeyCode(SYS::KeyCode::kOK);
  }
  if (this->sensor_state_ != SwitchState::open) {
    this->lidar_ptr_->disconnecting();
    if (!this->Open_()) {
      ERROR("Now is stop, open Ydlidar failed:%s", this->lidar_ptr_->DescribeError());
      this->Close_();
      return code_->GetKeyCode(SYS::KeyCode::kFailed);
    }
  }

  if (!this->lidar_ptr_->turnOn()) {
    ERROR("Ydlidar turnOn failed:%s", this->lidar_ptr_->DescribeError());
    this->Close_();
    return code_->GetKeyCode(SYS::KeyCode::kFailed);
  }

  if (this->update_data_thread_ptr_ == nullptr) {
    this->update_data_thread_ptr_ = std::make_shared<std::thread>(
      std::bind(&cyberdog::sensor::YdlidarCarpo::UpdateData, this));
    this->update_data_thread_ptr_->detach();
  }

  this->sensor_state_ = SwitchState::start;
  INFO("Ydlidar %s ok", this->state_msg_[this->sensor_state_].c_str());
  return code_->GetKeyCode(SYS::KeyCode::kOK);
}

int32_t cyberdog::sensor::YdlidarCarpo::Stop_()
{
  INFO("%s ydlidar ...", this->state_msg_[SwitchState::stop].c_str());
  if (this->lidar_ptr_ == nullptr) {
    ERROR("Now Ydlidar is not yet opened.");
    return code_->GetKeyCode(SYS::KeyCode::kFailed);
  }
  this->lidar_ptr_->turnOff();

  this->sensor_state_ = SwitchState::stop;
  INFO("Ydlidar %s ok", this->state_msg_[this->sensor_state_].c_str());
  return code_->GetKeyCode(SYS::KeyCode::kOK);
}

int32_t cyberdog::sensor::YdlidarCarpo::Close_()
{
  INFO("%s ydlidar ...", this->state_msg_[SwitchState::close].c_str());
  if (this->lidar_ptr_ == nullptr) {
    ERROR("Now Ydlidar is not yet opened.");
    return code_->GetKeyCode(SYS::KeyCode::kFailed);
  }
  this->lidar_ptr_->turnOff();
  this->lidar_ptr_->disconnecting();
  this->sensor_state_ = SwitchState::close;
  INFO("Ydlidar %s ok", this->state_msg_[this->sensor_state_].c_str());
  return code_->GetKeyCode(SYS::KeyCode::kOK);
}

void cyberdog::sensor::YdlidarCarpo::UpdateData()
{
  int sleep_time = static_cast<int>(1000 / this->frequency_);
  while (true) {
    if (!rclcpp::ok()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
    if (this->sensor_state_ != SwitchState::start) {
      continue;
    }
    if (this->lidar_ptr_->doProcessSimple(this->scan_sdk)) {
      this->raw_scan_.header.stamp.sec = RCL_NS_TO_S(this->scan_sdk.stamp);
      this->raw_scan_.header.stamp.nanosec = this->scan_sdk.stamp -
        RCL_S_TO_NS(this->raw_scan_.header.stamp.sec);
      this->raw_scan_.angle_min = this->scan_sdk.config.min_angle;
      this->raw_scan_.angle_max = this->scan_sdk.config.max_angle;
      this->raw_scan_.angle_increment = this->scan_sdk.config.angle_increment;
      this->raw_scan_.scan_time = this->scan_sdk.config.scan_time;
      this->raw_scan_.time_increment = this->scan_sdk.config.time_increment;
      this->raw_scan_.range_min = this->scan_sdk.config.min_range;
      this->raw_scan_.range_max = this->scan_sdk.config.max_range;
      int size = (this->scan_sdk.config.max_angle - this->scan_sdk.config.min_angle) /
        this->scan_sdk.config.angle_increment + 1;
      this->raw_scan_.ranges.resize(size);
      this->raw_scan_.intensities.resize(size);
      std::vector<bool> scan_updata;
      scan_updata.resize(size);
      for (size_t i = 0; i < this->scan_sdk.points.size(); i++) {
        int index = std::ceil(
          (this->scan_sdk.points[i].angle - this->scan_sdk.config.min_angle) /
          this->scan_sdk.config.angle_increment);
        if ((index >= 0) && (index < size) && (!scan_updata[index])) {
          scan_updata[index] = true;
          this->raw_scan_.ranges[index] = this->scan_sdk.points[i].range;
          this->raw_scan_.intensities[index] = this->scan_sdk.points[i].intensity;
        }
      }
      if (this->filter_ && this->filter_ptr_->update(this->raw_scan_, this->filter_scan_)) {
        this->payload_callback_(std::make_shared<ScanMsg>(this->filter_scan_));
      } else {
        this->payload_callback_(std::make_shared<ScanMsg>(this->raw_scan_));
      }
    }
  }
}

void cyberdog::sensor::YdlidarCarpo::UpdateSimulationData()
{
  int sleep_time = static_cast<int>(1000 / this->frequency_);
  std::shared_ptr<ScanMsg> sim_scan_ptr = std::make_shared<ScanMsg>(this->raw_scan_);
  while (true) {
    if (!rclcpp::ok()) {
      WARN("[cyberdog_lidar]: !rclcpp::ok()");
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
    if (this->sensor_state_ != SwitchState::start) {
      continue;
    }
    this->raw_scan_.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    this->payload_callback_(sim_scan_ptr);
  }
}

int32_t cyberdog::sensor::YdlidarCarpo::SelfCheck()
{
  bool ret = false;
  switch (this->sensor_state_) {
    case SwitchState::open:
      ret = true;
      break;
    case SwitchState::start:
      ret = true;
      break;
    case SwitchState::stop:
      ret = this->Start() && this->Stop();
      break;
    case SwitchState::close:
      ret = this->Open() && this->Start() && this->Stop() && this->Close();
      break;
    default:
      ret = false;
      break;
  }
  if (ret) {
    return code_->GetKeyCode(SYS::KeyCode::kOK);
  } else {
    return code_->GetKeyCode(SYS::KeyCode::kFailed);
  }
}

int32_t cyberdog::sensor::YdlidarCarpo::LowPowerOn()
{
  int32_t return_code = code_->GetKeyCode(SYS::KeyCode::kOK);
  if (Stop() != return_code) {
    return_code = code_->GetKeyCode(SYS::KeyCode::kFailed);
  }
  return return_code;
}

int32_t cyberdog::sensor::YdlidarCarpo::LowPowerOff()
{
  int32_t return_code = code_->GetKeyCode(SYS::KeyCode::kOK);
  if (Start() != return_code) {
    return_code = code_->GetKeyCode(SYS::KeyCode::kFailed);
  }
  return return_code;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cyberdog::sensor::YdlidarCarpo, cyberdog::sensor::LidarBase)
