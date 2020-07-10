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
 * @file rqt_fake_gnss.cpp
 * @brief RQt plugin class
 */

#include <rqt_fake_gnss/rqt_fake_gnss.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

namespace rqt_fake_gnss
{
FakeGnss::FakeGnss() : rqt_gui_cpp::Plugin() { setObjectName("FakeGnss"); }

void FakeGnss::initPlugin(qt_gui_cpp::PluginContext & context)
{
  widget = new FakeGnssWidget();
  context.addWidget(widget);
}

void FakeGnss::shutdownPlugin() {}

void FakeGnss::saveSettings(
  qt_gui_cpp::Settings & plugin_settings, qt_gui_cpp::Settings & instance_settings) const
{
  instance_settings.setValue("device_name", widget->getDeviceName());
}

void FakeGnss::restoreSettings(
  const qt_gui_cpp::Settings & plugin_settings, const qt_gui_cpp::Settings & instance_settings)
{
  widget->setDeviceName(instance_settings.value("device_name", "/dev/fake-gnss").toString());
}

}  // end namespace rqt_fake_gnss

PLUGINLIB_EXPORT_CLASS(rqt_fake_gnss::FakeGnss, rqt_gui_cpp::Plugin)
