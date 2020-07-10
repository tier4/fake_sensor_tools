/*
 * Copyright 2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file rqt_fake_imu.h
 * @brief RQt plugin class
 */

#ifndef FAKE_IMU_INCLUDE_RQT_FAKE_IMU_RQT_FAKE_IMU_H_
#define FAKE_IMU_INCLUDE_RQT_FAKE_IMU_RQT_FAKE_IMU_H_

#include <rqt_fake_imu/rqt_fake_imu_widget.h>
#include <rqt_gui_cpp/plugin.h>

namespace rqt_fake_imu
{
class FakeImu : public rqt_gui_cpp::Plugin
{
public:
  /**
   * @brief Constructor.
   */
  FakeImu();

  /**
   * Instantiate the plugin.
   * @param [in] context The plugin context
   */
  void initPlugin(qt_gui_cpp::PluginContext & context) override;

  /**
   * Shutdown and clean up the plugin before unloading.
   */
  void shutdownPlugin() override;

  /**
   * Save the intrinsic state of the plugin to the plugin-specific or instance-specific settings.
   * @param [in] plugin_settings The plugin-specific settings
   * @param [in] instance_settings The instance-specific settings
   */
  void saveSettings(
    qt_gui_cpp::Settings & plugin_settings,
    qt_gui_cpp::Settings & instance_settings) const override;

  /**
   * Restore the intrinsic state of the plugin from the plugin-specific or instance-specific settings.
   * @param [in] plugin_settings The plugin-specific settings
   * @param [in] instance_settings The instance-specific settings
   */
  void restoreSettings(
    const qt_gui_cpp::Settings & plugin_settings,
    const qt_gui_cpp::Settings & instance_settings) override;

private:
  FakeImuWidget * widget = nullptr;  //!< @brief RQt plugin widget class
};

}  // namespace rqt_fake_imu

#endif  // FAKE_IMU_INCLUDE_RQT_FAKE_IMU_RQT_FAKE_IMU_H_
