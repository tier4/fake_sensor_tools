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
 * @file rqt_fake_imu.cpp
 * @brief RQt plugin class
 */

#include <rqt_fake_imu/rqt_fake_imu.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

namespace rqt_fake_imu
{
FakeImu::FakeImu() : rqt_gui_cpp::Plugin() { setObjectName("FakeImu"); }

void FakeImu::initPlugin(qt_gui_cpp::PluginContext & context)
{
  widget = new FakeImuWidget();
  context.addWidget(widget);
}

void FakeImu::shutdownPlugin() {}

void FakeImu::saveSettings(
  qt_gui_cpp::Settings & plugin_settings, qt_gui_cpp::Settings & instance_settings) const
{
  instance_settings.setValue("device_name", widget->getDeviceName());
}

void FakeImu::restoreSettings(
  const qt_gui_cpp::Settings & plugin_settings, const qt_gui_cpp::Settings & instance_settings)
{
  widget->setDeviceName(instance_settings.value("device_name", "/dev/fake-imu").toString());
}

}  // namespace rqt_fake_imu

PLUGINLIB_EXPORT_CLASS(rqt_fake_imu::FakeImu, rqt_gui_cpp::Plugin)
