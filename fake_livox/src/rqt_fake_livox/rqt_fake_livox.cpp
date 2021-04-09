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
 * @file rqt_fake_livox.cpp
 * @brief RQt plugin class
 */

#include <rqt_fake_livox/rqt_fake_livox.h>

#include <pluginlib/class_list_macros.hpp>

namespace rqt_fake_livox
{
FakeLivox::FakeLivox() : rqt_gui_cpp::Plugin() { setObjectName("FakeLivox"); }

void FakeLivox::initPlugin(qt_gui_cpp::PluginContext & context)
{
  widget = new FakeLivoxWidget();
  context.addWidget(widget);
}

void FakeLivox::shutdownPlugin() {}

void FakeLivox::saveSettings(qt_gui_cpp::Settings & plugin_settings, qt_gui_cpp::Settings & instance_settings) const
{
  (void)plugin_settings;
  instance_settings.setValue("name", widget->getName());
  instance_settings.setValue("broadcast_code", widget->getBroadcastCode());
  instance_settings.setValue("network_interface", widget->getNetworkInterface());
  instance_settings.setValue("pcap_path", widget->getPcapPath());
  instance_settings.setValue("pcap_loop", widget->getPcapLoop());
  instance_settings.setValue("pcap_address", widget->getPcapSourceAddress());
  instance_settings.setValue("pcap_list", widget->getPcapFromList());
}

void FakeLivox::restoreSettings(
  const qt_gui_cpp::Settings & plugin_settings, const qt_gui_cpp::Settings & instance_settings)
{
  (void)plugin_settings;
  widget->setName(instance_settings.value("name", "Horizon").toString());
  widget->setBroadcastCode(instance_settings.value("broadcast_code", "100000000000000").toString());
  widget->setNetworkInterface(instance_settings.value("network_interface", "").toString());
  widget->setPcapPath(instance_settings.value("pcap_path", "").toString());
  widget->setPcapLoop(instance_settings.value("pcap_loop", true).toBool());
  widget->setPcapSourceAddress(instance_settings.value("pcap_address", "").toString());
  widget->setPcapFromList(instance_settings.value("pcap_list", false).toBool());
}

}  // end namespace rqt_fake_livox

PLUGINLIB_EXPORT_CLASS(rqt_fake_livox::FakeLivox, rqt_gui_cpp::Plugin)
