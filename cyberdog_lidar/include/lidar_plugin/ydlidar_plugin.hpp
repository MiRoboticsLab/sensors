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

#ifndef LIDAR_PLUGIN__YDLIDAR_PLUGIN_HPP_
#define LIDAR_PLUGIN__YDLIDAR_PLUGIN_HPP_

#include <sensor_msgs/msg/laser_scan.hpp>
#include <filters/filter_chain.hpp>
#include <cyberdog_common/cyberdog_log.hpp>
#include <cyberdog_common/cyberdog_toml.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <src/CYdLidar.h>

#include <memory>
#include <thread>
#include <string>
#include <map>

#include "lidar_base/lidar_base.hpp"
#include "cyberdog_system/robot_code.hpp"

namespace cyberdog
{
namespace sensor
{
namespace SYS = cyberdog::system;

/*! \file       ydlidar_plugin.hpp
    \brief      雷达插件模块。
    \details    创建及初始化雷达插件模块。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class YdlidarCarpo : public cyberdog::sensor::LidarBase
{
  using SwitchState = enum {open = 0, start, stop, close, };          /*!< [类型]切换状态 */
  using ScanMsg = sensor_msgs::msg::LaserScan;                        /*!< [topic 类型]激光数据 */
  using Filter = filters::FilterChain<ScanMsg>;                       /*!< 过滤激光数据 */

public:
  int32_t Init(bool simulator = false) override;
  enum class YdlidarCode : int32_t
  {
    kDemoError1 = 21
  };

private:
  float frequency_;                                                   /*!< 频率 */
  LaserScan scan_sdk;                                                 /*!< 激光数据 */
  std::map<SwitchState, std::string> state_msg_;                      /*!< 状态消息 */
  std::atomic<SwitchState> sensor_state_ {SwitchState::close};        /*!< node 状态 */
  std::shared_ptr<std::thread> update_data_thread_ptr_ {nullptr};     /*!< 更新数据线程 */
  std::shared_ptr<CYdLidar> lidar_ptr_ {nullptr};                     /*!< SDK雷达对象 */
  ScanMsg raw_scan_;                                                  /*!< 原始激光数据 */
  ScanMsg filter_scan_;                                               /*!< 过滤激光数据 */
  bool filter_ {false};                                               /*!< 是否过滤激光数据 */
  std::shared_ptr<Filter> filter_ptr_ {nullptr};                      /*!< 激光过滤器 */
  std::shared_ptr<SYS::CyberdogCode<YdlidarCode>> code_{nullptr};

private:
  int32_t Open_() override;
  int32_t Start_() override;
  int32_t Stop_() override;
  int32_t Close_() override;
  int32_t SelfCheck() override;
  int32_t LowPowerOn() override;
  int32_t LowPowerOff() override;
  void UpdateData();                                                  /*!< 更新数据 */
  void UpdateSimulationData();                                        /*!< 更新模拟数据 */

  LOGGER_MINOR_INSTANCE("Ydlidar");
};  // class LidarCarpo
}  // namespace sensor
}  // namespace cyberdog

#endif  // LIDAR_PLUGIN__YDLIDAR_PLUGIN_HPP_
