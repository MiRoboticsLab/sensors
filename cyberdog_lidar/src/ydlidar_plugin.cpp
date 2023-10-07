// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
  INFO("Open ydlidar ...");
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

  // if (access(lidar_config_dir.c_str(), W_OK)) {
  //   ERROR("Params config file does not have write permissions");
  //   return code_->GetKeyCode(SYS::KeyCode::kFailed);
  // }

  toml::value params_toml;
  if (!cyberdog::common::CyberdogToml::ParseFile(
      lidar_config_dir.c_str(), params_toml))
  {
    ERROR("Params config file is not in toml format");
    return code_->GetKeyCode(SYS::KeyCode::kFailed);
  }

  this->filter_ = toml::find_or(
    params_toml, "dylidar", "filter", false);
  this->filter_ptr_ = std::make_shared<Filter>("sensor_msgs::msg::LaserScan");
  this->raw_scan_.header.frame_id = toml::find_or(
    params_toml, "dylidar", "frame_id", "laser_frame");

  INFO("this->raw_scan_.header.frame_id = %s", this->raw_scan_.header.frame_id.c_str());

  if (this->lidar_ptr_ == nullptr) {
    this->lidar_ptr_ = std::make_shared<CYdLidar>();
  }

  std::string str_optvalue;
  str_optvalue = toml::find_or(params_toml, "dylidar", "port", "/dev/ydlidar");
  this->lidar_ptr_->setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());
  INFO("[Open] dylidar->port = %s", str_optvalue.c_str());

  str_optvalue = toml::find_or(params_toml, "dylidar", "ignore_array", "");
  this->lidar_ptr_->setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());
  INFO("[Open] dylidar->ignore_array = %s", str_optvalue.c_str());

  int int_optvalue;
  int_optvalue = toml::find_or(params_toml, "dylidar", "baudrate", 512000);
  this->lidar_ptr_->setlidaropt(LidarPropSerialBaudrate, &int_optvalue, sizeof(int));
  INFO("[Open] dylidar->baudrate = %d", int_optvalue);

  int_optvalue =
    toml::find_or(
    params_toml, "dylidar", "lidar_type", static_cast<int>(LidarTypeID::TYPE_TOF));
  this->lidar_ptr_->setlidaropt(LidarPropLidarType, &int_optvalue, sizeof(int));
  INFO("[Open] dylidar->lidar_type = %d", int_optvalue);

  int_optvalue =
    toml::find_or(
    params_toml, "dylidar", "device_type",
    static_cast<int>(DeviceTypeID::YDLIDAR_TYPE_SERIAL));
  this->lidar_ptr_->setlidaropt(LidarPropDeviceType, &int_optvalue, sizeof(int));
  INFO("[Open] dylidar->device_type = %d", int_optvalue);

  int_optvalue = toml::find_or(params_toml, "dylidar", "sample_rate", 9);
  this->lidar_ptr_->setlidaropt(LidarPropSampleRate, &int_optvalue, sizeof(int));
  INFO("[Open] dylidar->sample_rate = %d", int_optvalue);

  int_optvalue = toml::find_or(params_toml, "dylidar", "abnormal_check_count", 4);
  this->lidar_ptr_->setlidaropt(LidarPropAbnormalCheckCount, &int_optvalue, sizeof(int));
  INFO("[Open] dylidar->sample_rate = %d", int_optvalue);

  bool bool_optvalue;
  bool_optvalue = toml::find_or(params_toml, "dylidar", "resolution_fixed", false);
  this->lidar_ptr_->setlidaropt(LidarPropFixedResolution, &bool_optvalue, sizeof(bool));
  INFO("[Open] dylidar->resolution_fixed = %s", bool_optvalue ? "True" : "False");

  bool_optvalue = toml::find_or(params_toml, "dylidar", "reversion", true);
  this->lidar_ptr_->setlidaropt(LidarPropReversion, &bool_optvalue, sizeof(bool));
  INFO("[Open] dylidar->reversion = %s", bool_optvalue ? "True" : "False");

  bool_optvalue = toml::find_or(params_toml, "dylidar", "inverted", true);
  this->lidar_ptr_->setlidaropt(LidarPropInverted, &bool_optvalue, sizeof(bool));
  INFO("[Open] dylidar->inverted = %s", bool_optvalue ? "True" : "False");

  bool_optvalue = toml::find_or(params_toml, "dylidar", "auto_reconnect", true);
  this->lidar_ptr_->setlidaropt(LidarPropAutoReconnect, &bool_optvalue, sizeof(bool));
  INFO("[Open] dylidar->auto_reconnect = %s", bool_optvalue ? "True" : "False");

  bool_optvalue = toml::find_or(params_toml, "dylidar", "isSingleChannel", false);
  this->lidar_ptr_->setlidaropt(LidarPropSingleChannel, &bool_optvalue, sizeof(bool));
  INFO("[Open] dylidar->isSingleChannel = %s", bool_optvalue ? "True" : "False");

  bool_optvalue = toml::find_or(params_toml, "dylidar", "intensity", false);
  this->lidar_ptr_->setlidaropt(LidarPropIntenstiy, &bool_optvalue, sizeof(bool));
  INFO("[Open] dylidar->intensity = %s", bool_optvalue ? "True" : "False");

  bool_optvalue = toml::find_or(params_toml, "dylidar", "support_motor_dtr", false);
  this->lidar_ptr_->setlidaropt(LidarPropSupportMotorDtrCtrl, &bool_optvalue, sizeof(bool));
  INFO("[Open] dylidar->support_motor_dtr = %s", bool_optvalue ? "True" : "False");

  float float_optvalue;
  float_optvalue =
    toml::find_or(params_toml, "dylidar", "angle_max", static_cast<float>(180.0f));
  this->lidar_ptr_->setlidaropt(LidarPropMaxAngle, &float_optvalue, sizeof(float));
  INFO("[Open] dylidar->angle_max = %f", float_optvalue);

  float_optvalue =
    toml::find_or(params_toml, "dylidar", "angle_min", static_cast<float>(-180.0f));
  this->lidar_ptr_->setlidaropt(LidarPropMinAngle, &float_optvalue, sizeof(float));
  INFO("[Open] dylidar->angle_min = %f", float_optvalue);

  float_optvalue =
    toml::find_or(params_toml, "dylidar", "range_max", static_cast<float>(64.f));
  this->lidar_ptr_->setlidaropt(LidarPropMaxRange, &float_optvalue, sizeof(float));
  INFO("[Open] dylidar->range_max = %f", float_optvalue);

  float_optvalue =
    toml::find_or(params_toml, "dylidar", "range_min", static_cast<float>(0.1f));
  this->lidar_ptr_->setlidaropt(LidarPropMinRange, &float_optvalue, sizeof(float));
  INFO("[Open] dylidar->range_min = %f", float_optvalue);

  this->frequency_ =
    toml::find_or(params_toml, "dylidar", "frequency", static_cast<float>(10.f));
  this->lidar_ptr_->setlidaropt(LidarPropScanFrequency, &this->frequency_, sizeof(float));
  INFO("[Open] dylidar->frequency = %f", this->frequency_);

  this->sensor_state_ = SwitchState::open;
  INFO("Ydlidar %s ok", this->state_msg_[this->sensor_state_].c_str());
  return code_->GetKeyCode(SYS::KeyCode::kOK);
}

int32_t cyberdog::sensor::YdlidarCarpo::Start_()
{
  INFO("Start ydlidar ...");
  if (this->lidar_ptr_ == nullptr) {
    ERROR("Start ydlidar failed (Now Ydlidar is not yet opened).");
    return code_->GetKeyCode(SYS::KeyCode::kFailed);
  }
  if (this->sensor_state_ == SwitchState::start) {
    INFO("Start ydlidar ok (consistent with the current state)");
    return code_->GetKeyCode(SYS::KeyCode::kOK);
  }
  if (!this->lidar_ptr_->initialize()) {
    ERROR("Start ydlidar failed (%s).", this->lidar_ptr_->DescribeError());
    this->lidar_ptr_->disconnecting();
    return code_->GetKeyCode(SYS::KeyCode::kFailed);
  }
  if (!this->lidar_ptr_->turnOn()) {
    ERROR("Start ydlidar (turnOn) failed (%s)", this->lidar_ptr_->DescribeError());
    this->lidar_ptr_->disconnecting();
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
  INFO("Stop ydlidar ...");
  if (this->lidar_ptr_ == nullptr) {
    ERROR("Stop ydlidar failed (Now Ydlidar is not yet opened).");
    return code_->GetKeyCode(SYS::KeyCode::kFailed);
  }
  if (this->sensor_state_ == SwitchState::stop) {
    INFO("Stop ydlidar ok (consistent with the current state)");
    return code_->GetKeyCode(SYS::KeyCode::kOK);
  }
  if (!this->lidar_ptr_->turnOff()) {
    ERROR("Stop ydlidar (turnOff) failed (%s)", this->lidar_ptr_->DescribeError());
    return code_->GetKeyCode(SYS::KeyCode::kFailed);
  }
  this->lidar_ptr_->disconnecting();
  this->sensor_state_ = SwitchState::stop;
  INFO("Ydlidar %s ok", this->state_msg_[this->sensor_state_].c_str());
  return code_->GetKeyCode(SYS::KeyCode::kOK);
}

int32_t cyberdog::sensor::YdlidarCarpo::Close_()
{
  INFO("Close ydlidar ...");
  if (this->sensor_state_ == SwitchState::close) {
    INFO("Close ydlidar ok (consistent with the current state)");
    return code_->GetKeyCode(SYS::KeyCode::kOK);
  }
  if (this->lidar_ptr_ != nullptr) {
    this->Stop_();
  }
  this->lidar_ptr_ = nullptr;
  this->sensor_state_ = SwitchState::close;
  INFO("Ydlidar %s ok", this->state_msg_[this->sensor_state_].c_str());
  return code_->GetKeyCode(SYS::KeyCode::kOK);
}

void cyberdog::sensor::YdlidarCarpo::UpdateData()
{
  INFO("UpdateData ydlidar ...");
  int sleep_time = static_cast<int>(1000 / this->frequency_);
  bool warn_is_print = false;
  while (true) {
    if (!rclcpp::ok()) {
      ERROR("Ydlidar update data failed (!rclcpp::ok())");
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
    if (this->sensor_state_ != SwitchState::start) {
      if (!warn_is_print) {
        warn_is_print = true;
        WARN("Ydlidar update data continue (!start)");
      }
      continue;
    }
    if (this->lidar_ptr_ == nullptr) {
      if (!warn_is_print) {
        warn_is_print = true;
        WARN("Ydlidar update data continue (lidar_ptr == nullptr)");
      }
      continue;
    }
    if (warn_is_print) {
      warn_is_print = false;
      INFO("Ydlidar update data ...");
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
  INFO("Ydlidar update data ok");
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
    if ((this->sensor_state_ != SwitchState::start) || (this->lidar_ptr_ == nullptr)) {
      continue;
    }
    this->raw_scan_.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    this->payload_callback_(sim_scan_ptr);
  }
}

int32_t cyberdog::sensor::YdlidarCarpo::SelfCheck()
{
  INFO("SelfCheck ydlidar ...");
  // auto self_check = [&]() -> int32_t {
  //   switch (this->sensor_state_)
  //   {
  //   case SwitchState::open:
  //     return this->Start() + this->Stop() + this->Close() +
  //     this->Open() + this->Start() + this->Stop() + this->Start() + this->Stop() +
  //     this->Close() + this->Open();
  //   case SwitchState::start:
  //     return this->Stop() + this->Close() +
  //     this->Open() + this->Start() + this->Stop() + this->Start() + this->Stop() +
  //     this->Close() + this->Open() + this->Start();

  //   case SwitchState::stop:
  //     return this->Close() +
  //     this->Open() + this->Start() + this->Stop() + this->Start() + this->Stop() +
  //     this->Close() + this->Open() + this->Start() + this->Stop();

  //   case SwitchState::close:
  //     return this->Open() + this->Start() + this->Stop() + this->Start() + this->Stop() +
  //     this->Close() + this->Open() + this->Start() + this->Stop() + this->Close();

  //   default:
  //     return code_->GetKeyCode(SYS::KeyCode::kOK);
  //     break;
  //   }
  // };
  auto fast_self_check = [&]() -> int32_t {
      switch (this->sensor_state_) {
        case SwitchState::open:
          return this->Start() + this->Stop() + this->Close() + this->Open();
        case SwitchState::start:
          return this->Stop() + this->Start() + this->Stop() + this->Start();
        case SwitchState::stop:
          return this->Close() + this->Open() + this->Start() + this->Stop();
        case SwitchState::close:
          return this->Open() + this->Start() + this->Stop() + this->Close();
        default:
          return code_->GetKeyCode(SYS::KeyCode::kOK);
          break;
      }
    };
  if (fast_self_check()) {
    ERROR("SelfCheck ydlidar failed.");
    return code_->GetKeyCode(SYS::KeyCode::kSelfCheckFailed);
  }
  INFO("Ydlidar self check ok");
  return code_->GetKeyCode(SYS::KeyCode::kOK);
}

int32_t cyberdog::sensor::YdlidarCarpo::LowPowerOn()
{
  INFO("LowPowerOn ydlidar ...");
  if (this->Stop()) {
    ERROR("LowPowerOn ydlidar failed.");
    return code_->GetKeyCode(SYS::KeyCode::kFailed);
  }
  INFO("Ydlidar low power on ok");
  return code_->GetKeyCode(SYS::KeyCode::kOK);
}

int32_t cyberdog::sensor::YdlidarCarpo::LowPowerOff()
{
  INFO("LowPowerOff ydlidar ...");
  if (this->Start()) {
    ERROR("LowPowerOff ydlidar failed.");
    return code_->GetKeyCode(SYS::KeyCode::kFailed);
  }
  INFO("Ydlidar low power off ok");
  return code_->GetKeyCode(SYS::KeyCode::kOK);
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cyberdog::sensor::YdlidarCarpo, cyberdog::sensor::LidarBase)
