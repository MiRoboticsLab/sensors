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
#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "bcm_gps/bcm_gps.hpp"
#include "bream_vendor/bream_callbacks.hpp"
#include "bream_vendor/patch_downloader.h"
#include "bream_vendor/bream_helper.h"
#include "cyberdog_common/cyberdog_log.hpp"


bcm_gps::GPS::GPS(PAYLOAD_callback PAYLOAD_cb, NMEA_callback NMEA_cb)
{
  if (all_num_ == 0) {
    NMEA_callbacks_ = std::map<int, NMEA_callback>();
    PAYLOAD_callbacks_ = std::map<int, PAYLOAD_callback>();
  }
  id_ = all_num_++;
  Open();
  SetCallback(PAYLOAD_cb);
  SetCallback(NMEA_cb);
}

bcm_gps::GPS::~GPS()
{
  // Close();
  INFO("[cyberdog_gps] destroy GPS");
}

bool bcm_gps::GPS::Open()
{
  if (opened_ || (ready_ == false && Init() == false)) {
    return opened_;
  }
  INFO("[cyberdog_gps]: GPS%d Opened", id_);
  opened_ = true;
  init_num_++;
  return true;
}

void bcm_gps::GPS::Start()
{
  if (ready_ == false) {
    ERROR("[cyberdog_gps]: GPS module not ready yet");
    return;
  }
  if (start_) {return;}
  if (start_num_++ == 0) {
    BreamHelper::GetInstance().GnssStart();
    start_ = true;
    INFO("[cyberdog_gps]: GnssStart");
  }
}

void bcm_gps::GPS::Stop()
{
  if (ready_ == false) {
    ERROR("[cyberdog_gps][Stop]: GPS module not ready yet");
    return;
  }
  if (start_ == false) {return;}
  start_ = false;
  if (--start_num_ == 0) {
    INFO("[cyberdog_gps]: GnssStop");
    BreamHelper::GetInstance().GnssStop();
  }
}

void bcm_gps::GPS::Close()
{
  Stop();
  if (opened_ == false) {return;}
  if (NMEA_callbacks_.count(id_) != 0) {NMEA_callbacks_.erase(id_);}
  if (PAYLOAD_callbacks_.count(id_) != 0) {PAYLOAD_callbacks_.erase(id_);}
  if (ready_ && opened_ && --init_num_ == 0) {
    INFO("[cyberdog_gps]: Last usage close, exit main thread");
    main_running_ = false;
    main_T_.join();
  }
  opened_ = false;
}

bool bcm_gps::GPS::Ready()
{
  return ready_;
}

bool bcm_gps::GPS::IsOpened()
{
  return opened_;
}

bool bcm_gps::GPS::IsStarted()
{
  return start_;
}


void bcm_gps::GPS::SetCallback(NMEA_callback NMEA_cb)
{
  if (NMEA_cb == nullptr) {return;}
  if (NMEA_callbacks_.count(id_) != 0) {NMEA_callbacks_.at(id_) = NMEA_cb;} else {
    NMEA_callbacks_.insert(std::map<int, NMEA_callback>::value_type(id_, NMEA_cb));
  }
  INFO("[cyberdog_gps]: GPS%dSuccess SetCallback NMEA", id_);
}

void bcm_gps::GPS::SetCallback(PAYLOAD_callback PAYLOAD_cb)
{
  if (PAYLOAD_cb == nullptr) {return;}
  if (PAYLOAD_callbacks_.count(id_) != 0) {PAYLOAD_callbacks_.at(id_) = PAYLOAD_cb;} else {
    PAYLOAD_callbacks_.insert(std::map<int, PAYLOAD_callback>::value_type(id_, PAYLOAD_cb));
  }
  INFO("[cyberdog_gps]: GPS%d Success SetCallback PAYLOAD", id_);
}

void bcm_gps::GPS::SetL5Bias(uint32_t biasCm)
{
  BreamHelper::GetInstance().SetL5Bias(biasCm);
}

void bcm_gps::GPS::SetLteFilterEn(bool enable)
{
  BreamHelper::GetInstance().SetLteFilterEn(enable);
}

void change(char * ptr, const char * str)
{
  while (*str != '\0') {
    *ptr = *str;
    ptr++;
    str++;
  }
}

// make "skip_download = true" in bcmgps_config.toml
void SetSkip(std::string path)
{
  try {
    FILE * fp1;
    if ((fp1 = fopen(path.c_str(), "rw+")) != NULL) {
      char tmp[40];
      fread(tmp, sizeof(char), 40, fp1);
      for (int a = 0; a < 40; a++) {
        if (tmp[a] == 'f') {
          change(&tmp[a], "true ");
          break;
        }
      }
      fseek(fp1, 0, 0);
      fwrite(tmp, sizeof(char), 40, fp1);
      fclose(fp1);
      INFO("[cyberdog_gps]: :Write file: %s", path.c_str());
      return;
    }
  } catch (...) {
  }
  ERROR("[cyberdog_gps]: Cant write file");
}

bool bcm_gps::GPS::Init()
{
  INFO("[cyberdog_gps]: BCM_GPS Init");
  // change toml11vondor to cyberdog_common
  toml::value value;
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto local_config_dir = local_share_dir + std::string("/toml_config/sensors/bcmgps_config.toml");
  if (access(local_config_dir.c_str(), F_OK) != 0) {
    ERROR("[cyberdog_gps]: %s do not exist!", local_config_dir.c_str());
    ERROR("[cyberdog_gps]: init failed");
    return false;
  }
  if (!cyberdog::common::CyberdogToml::ParseFile(
      std::string(local_share_dir) +
      "/toml_config/sensors/bcmgps_config.toml", value))
  {
    ERROR("[cyberdog_gps]: fail to read data from toml");
  }
  bool skip_download;
  if (!cyberdog::common::CyberdogToml::Get(value, "skip_download", skip_download)) {
    ERROR("[cyberdog_gps]: fail to read key skip_download from toml");
  }
  bool onlyfirst_download;
  if (!cyberdog::common::CyberdogToml::Get(value, "onlyfirst_download", onlyfirst_download)) {
    ERROR("[cyberdog_gps]: fail to read key onlyfirst_download from toml");
  }
  std::string spi_str;
  if (!cyberdog::common::CyberdogToml::Get(value, "spi", spi_str)) {
    ERROR("[cyberdog_gps]: fail to read key spi from toml");
  }
  std::string patch_path;
  if (!cyberdog::common::CyberdogToml::Get(value, "patch_path", patch_path)) {
    ERROR("[cyberdog_gps]: fail to read key patch_path from toml");
  }

  std::vector<uint8_t> infMsgMask_vec;

  if (!cyberdog::common::CyberdogToml::Get(value, "infMsgMask", infMsgMask_vec)) {
    ERROR("[cyberdog_gps]: fail to read key infMsgMask from toml");
  }
  uint8_t PowerModePreset;
  if (!cyberdog::common::CyberdogToml::Get(value, "PowerModePreset", PowerModePreset)) {
    ERROR("[cyberdog_gps]: fail to read key PowerModePreset from toml");
  }
  std::vector<uint8_t> MsgRate_vec;
  if (!cyberdog::common::CyberdogToml::Get(value, "MsgRate", MsgRate_vec)) {
    ERROR("[cyberdog_gps]: fail to read key MsgRate from toml");
  }
  bool AckAiding;
  if (!cyberdog::common::CyberdogToml::Get(value, "AckAiding", AckAiding)) {
    ERROR("[cyberdog_gps]: fail to read key AckAiding from toml");
  }

  if (access(patch_path.c_str(), F_OK) != 0) {
    ERROR("[cyberdog_gps]: %s do not exist!", patch_path.c_str());
    ERROR("[cyberdog_gps]: init failed");
    return false;
  }

  // reset entire chip
  LD2OS_initGpio();
  int portNumber = -1;
  const int baudrate = 3000000;
  const char * tty = spi_str.c_str();
  LoDi2SerialConnection connType = LODI2_SERIAL_SPI;

  if (!skip_download) {
    INFO("[cyberdog_gps]: Start load patch: %s", patch_path.c_str());
    // Open serial port
    INFO("[cyberdog_gps]: tty: %s", tty);

    LD2OS_open(connType, portNumber, baudrate, tty);
    // Download patch
    INFO("[cyberdog_gps]: down load patch: %s", patch_path.c_str());
    if (false == Bream_LoadPatch(patch_path.c_str())) {
      INFO("[cyberdog_gps]: Cant load patch: %s", patch_path.c_str());
      ready_ = false;
      return false;
    }
    INFO("[cyberdog_gps]: Success load patch: %s", patch_path.c_str());

  } else {
    INFO("[cyberdog_gps]: Skip load patch: %s", patch_path.c_str());
  }
  if (onlyfirst_download == true && skip_download == false) {SetSkip(local_config_dir);}

  if (connType == LODI2_SERIAL_UART) {
    // Reopen and send command to switch baudrate from default 115200
    LD2OS_open(connType, portNumber, 115200, tty);

    BreamHelper::GetInstance().SetBaudrate(baudrate);
    // delay needed because MCU needs some time in receiving CFG-PRT and handling it.
    // This delay can be replaced by checking ACK for CFG-PRT
    LD2OS_delay(100);
    // Reopen in new baudrate
    LD2OS_open(connType, portNumber, baudrate, tty);
  }
  // Register callbacks and start listener loop
  main_T_ = std::thread(std::bind(&GPS::MainThread, this));
  // If SPI connection, run read thread first and then send first packet to mcu
  if (connType == LODI2_SERIAL_SPI) {
    LD2OS_delay(200);
    BreamHelper::GetInstance().SetBaudrate(baudrate, 4);
  }

  // Config GNSS
  INFO("[cyberdog_gps]: ConfigGNSS");
  uint8_t infMsgMask[6];
  for (int a = 0; a < 6 && a < static_cast<int>(infMsgMask_vec.size()); a++) {
    infMsgMask[a] = infMsgMask_vec[a];
  }
  BreamHelper::GetInstance().SetLogging(infMsgMask);
  BreamHelper::GetInstance().EnableBlindGalSearch();

  // 1(L1 Best) 2(L1 Auto) 3(L1 ULP), 4(L1L5 Best) 5(L1L5 Auto)
  switch (PowerModePreset) {
    case 1: BreamHelper::GetInstance().SetPowerModePreset(0, 0); break;
    case 2: BreamHelper::GetInstance().SetPowerModePreset(0, 1); break;
    case 3: BreamHelper::GetInstance().SetPowerModePreset(0, 3); break;
    default:
    case 4: BreamHelper::GetInstance().SetPowerModePreset(1, 0); break;
    case 5: BreamHelper::GetInstance().SetPowerModePreset(1, 1); break;
  }

  for (int a = static_cast<int>(MsgRate_vec.size()); a < 18; a++) {
    MsgRate_vec.push_back(0);
  }
  int index = 0;
  BreamHelper::GetInstance().SetMsgRate(0xF0, 0x00, MsgRate_vec[index++]);  // Report GPGGA
  BreamHelper::GetInstance().SetMsgRate(0xF0, 0x04, MsgRate_vec[index++]);  // Report GPRMC
  BreamHelper::GetInstance().SetMsgRate(0xF0, 0x03, MsgRate_vec[index++]);  // Report GPGSV
  BreamHelper::GetInstance().SetMsgRate(0x01, 0x61, MsgRate_vec[index++]);  // Report NAVEOE
  BreamHelper::GetInstance().SetMsgRate(0x01, 0x07, MsgRate_vec[index++]);  // Report NAVPVT
  BreamHelper::GetInstance().SetMsgRate(0xF1, 0x00, MsgRate_vec[index++]);  // Report PGLOR SPEED
  BreamHelper::GetInstance().SetMsgRate(0xF1, 0x01, MsgRate_vec[index++]);  // Report PGLOR FIX
  BreamHelper::GetInstance().SetMsgRate(0xF1, 0x02, MsgRate_vec[index++]);  // Report PGLOR SAT
  BreamHelper::GetInstance().SetMsgRate(0xF1, 0x03, MsgRate_vec[index++]);  // Report PGLOR LSQ
  BreamHelper::GetInstance().SetMsgRate(0xF1, 0x04, MsgRate_vec[index++]);  // Report PGLOR PWR
  BreamHelper::GetInstance().SetMsgRate(0xF1, 0x05, MsgRate_vec[index++]);  // Report PGLOR STA
  BreamHelper::GetInstance().SetMsgRate(0x01, 0x09, MsgRate_vec[index++]);  // Report NAVODO
  BreamHelper::GetInstance().SetMsgRate(0x01, 0x35, MsgRate_vec[index++]);  // Report NAVSAT
  BreamHelper::GetInstance().SetMsgRate(0x01, 0x04, MsgRate_vec[index++]);  // Report NAVDOP
  BreamHelper::GetInstance().SetMsgRate(0x01, 0x60, MsgRate_vec[index++]);  // Report CBEE status
  BreamHelper::GetInstance().SetMsgRate(0x02, 0x13, MsgRate_vec[index++]);  // Report ASC SUBFRAMES
  BreamHelper::GetInstance().SetMsgRate(0x02, 0x15, MsgRate_vec[index++]);  // Report ASC MEAS
  BreamHelper::GetInstance().SetMsgRate(0x02, 0x80, MsgRate_vec[index++]);  // Report ASC AGC
  BreamHelper::GetInstance().SetAckAiding(AckAiding);
  BreamHelper::GetInstance().GetVer();
  INFO("[cyberdog_gps]: Finish Config");
  ready_ = true;
  return true;
}

void bcm_gps::GPS::MainThread()
{
  INFO("[cyberdog_gps]: MainThread Start");
  RegisterBreamCallbacks();
  main_running_ = true;

  uint8_t rxBuff[1024];
  uint32_t rxLen = 0;
  while (main_running_) {
    bool ret = LD2OS_readFromSerial(rxBuff, &rxLen, sizeof(rxBuff), -1);
    if (!ret) {break;}
    if (rxLen) {
      BreamHandler::GetInstance().AsicData(rxBuff, rxLen);
    }
  }
  ready_ = false;
  LD2OS_freeGpio();
  LD2OS_close();
  LD2OS_closeLog();
  INFO("[cyberdog_gps]: MainThread Exit");
}
