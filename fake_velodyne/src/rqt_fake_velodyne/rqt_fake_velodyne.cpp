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
 * @file rqt_fake_velodyne.cpp
 * @brief RQt plugin class
 */

#include <rqt_fake_velodyne/rqt_fake_velodyne.h>

#include <pluginlib/class_list_macros.hpp>

namespace rqt_fake_velodyne
{
FakeVelodyne::FakeVelodyne() : rqt_gui_cpp::Plugin() { setObjectName("FakeVelodyne"); }

void FakeVelodyne::initPlugin(qt_gui_cpp::PluginContext & context)
{
  widget = new FakeVelodyneWidget();
  context.addWidget(widget);
}

void FakeVelodyne::shutdownPlugin() {}

void FakeVelodyne::saveSettings(
  qt_gui_cpp::Settings & plugin_settings, qt_gui_cpp::Settings & instance_settings) const
{
  (void)plugin_settings;
  instance_settings.setValue("address", widget->getAddress());
}

void FakeVelodyne::restoreSettings(
  const qt_gui_cpp::Settings & plugin_settings, const qt_gui_cpp::Settings & instance_settings)
{
  (void)plugin_settings;
  widget->setAddress(instance_settings.value("address", "http://localhost:8000").toString());
}

}  // end namespace rqt_fake_velodyne

PLUGINLIB_EXPORT_CLASS(rqt_fake_velodyne::FakeVelodyne, rqt_gui_cpp::Plugin)
