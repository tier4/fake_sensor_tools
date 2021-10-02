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
 * @file rqt_fake_livox_widget.h
 * @brief RQt plugin widget class
 */

#pragma once

#include <pcap/pcap.h>
#include <map>
#include <string>
#include <vector>

#include <QButtonGroup>
#include <QWidget>

#include <fake_point_cloud.h>
#include <lidar_status_code.h>
#include <sdk_protocol.h>
#include <udp_list_model.h>

struct NetworkInterface
{
  std::string ifname;
  asip::address_v4 address;

  NetworkInterface(const std::string & name, const asip::address_v4 & addr) : ifname(name), address(addr) {}
};

namespace Ui
{
class FakeLivoxWidget;
}

class FakeLivoxWidget : public QWidget
{
  Q_OBJECT

public:
  /**
   * @brief Constructs a widget which is a child of parent.
   * @param [in] parent The new widget becomes a window
   */
  explicit FakeLivoxWidget(QWidget * parent = nullptr);

  /**
   * @brief Destructor.
   */
  ~FakeLivoxWidget();

  /**
   * @brief Set name.
   * @param [in] name name
   */
  void setName(const QString & name);

  /**
   * @brief Get name.
   * @return name
   */
  QString getName() const;

  /**
   * @brief Set broadcast code.
   * @param [in] broadcast_code Broadcast code
   */
  void setBroadcastCode(const QString & broadcast_code);

  /**
   * @brief Get broadcast code.
   * @return Broadcast code
   */
  QString getBroadcastCode() const;

  /**
   * @brief Set network interface.
   * @param[in] interface network interface
   */
  void setNetworkInterface(const QString & interface);

  /**
   * @brief Get network interface.
   * @return network interface
   */
  QString getNetworkInterface() const;

  /**
   * @brief Set path of pcap file.
   * @param [in] pcap_path path of pcap file
   */
  void setPcapPath(const QString & pcap_path);

  /**
   * @brief Get path of pcap file.
   * @return path of pcap file
   */
  QString getPcapPath() const;

  /**
   * @brief Set loop playback of pcap.
   * @param [in] pcap_path loop playback
   */
  void setPcapLoop(bool pcap_loop);

  /**
   * @brief Get loop playback of pcap.
   * @return loop playback
   */
  bool getPcapLoop() const;

  /**
   * @brief Set source address to filter.
   * @param [in] source_address source address to filter
   */
  void setPcapSourceAddress(const QString & source_address);

  /**
   * @brief Get source address to filter.
   * @return source address
   */
  QString getPcapSourceAddress() const;

  /**
   * @brief Set flag to filter by selecting from list.
   * @param [in] pcap_from_list flag to filter by selecting from list
   */
  void setPcapFromList(bool pcap_from_list);

  /**
   * @brief Get flag to filter by selecting from list.
   * @return flag to filter by selecting from list
   */
  bool getPcapFromList() const;

private slots:
  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_pushButton_comm_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_pushButton_checksum_error_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_pushButton_debug_output_toggled(bool checked);

  /**
   * @brief This signal is emitted when the button is activated
   * @param[in] id id
   */
  void onButtonGroupReturnCodeClicked(int id);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_pushButton_rain_fog_suppression_switch_toggled(bool checked);

  /**
   * @brief This signal is emitted when the value shown in the progress bar changes. value is the new value shown by the progress bar.
   * @param[in] value progress bar's current value
   */
  void on_slider_initialization_progress_valueChanged(double value);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_comboBox_lidar_state_currentIndexChanged(int index);

  /**
   * @brief This signal is emitted when the button is activated
   * @param[in] id id
   */
  void onButtonGroupTempStatusClicked(int id);

  /**
   * @brief This signal is emitted when the button is activated
   * @param[in] id id
   */
  void onButtonGroupVoltStatusClicked(int id);

  /**
   * @brief This signal is emitted when the button is activated
   * @param[in] id id
   */
  void onButtonGroupMotorStatusClicked(int id);

  /**
   * @brief This signal is emitted when the button is activated
   * @param[in] id id
   */
  void onButtonGroupDirtyWarnClicked(int id);

  /**
   * @brief This signal is emitted when the button is activated
   * @param[in] id id
   */
  void onButtonGroupFirmwareStatusClicked(int id);

  /**
   * @brief This signal is emitted when the button is activated
   * @param[in] id id
   */
  void onButtonGroupPpsStatusClicked(int id);

  /**
   * @brief This signal is emitted when the button is activated
   * @param[in] id id
   */
  void onButtonGroupDeviceStatusClicked(int id);

  /**
   * @brief This signal is emitted when the button is activated
   * @param[in] id id
   */
  void onButtonGroupFanStatusClicked(int id);

  /**
   * @brief This signal is emitted when the button is activated
   * @param[in] id id
   */
  void onButtonGroupSelfHeatingClicked(int id);

  /**
   * @brief This signal is emitted when the button is activated
   * @param[in] id id
   */
  void onButtonGroupPtpStatusClicked(int id);

  /**
   * @brief This signal is emitted when the button is activated
   * @param[in] id id
   */
  void onButtonGroupTimeSyncStatusClicked(int id);

  /**
   * @brief This signal is emitted when the button is activated
   * @param[in] id id
   */
  void onButtonGroupSystemStatusClicked(int id);

  /**
   * @brief This signal is emitted when the button is activated
   */
  void on_pushButton_pcap_path_clicked();

  /**
   * @brief This signal is emitted when the button is activated
   */
  void on_pushButton_pcap_read_clicked();

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_pushButton_pcap_loop_toggled(bool checked);

  /**
   * @brief This signal is emitted when a mouse button is double-clicked.
   * @param [in] index specified index
   */
  void on_tableView_pcap_packets_doubleClicked(const QModelIndex & index);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_checkBox_select_from_list_toggled(bool checked);

private:
  /**
   * @brief Initialize LiDAR Status Code.
   */
  void initLidarStatusCode();

  /**
   * @brief Get LiDAR Status Code.
   * @param[in] name name of LiDAR status
   * @param[in] value value of LiDAR status
   */
  uint32_t getLidarStatusCode(const LidarStatusCode & name, int value);

  /**
   * @brief Set LiDAR Status Code to other class.
   * @param[in] name name of LiDAR Status Code
   * @param[in] value value of LiDAR Status Code
   */
  void notifyLidarStatusCode(const LidarStatusCode & name, uint32_t value);

  /**
   * @brief Callback for Start/Stop Sampling.
   * @param[in] request Start/Stop Sampling
   * @param[in] livox_ip IP address of Livox
   * @param[in] user_ip Host IPAddress
   * @param[in] data_port Host Point Cloud Data UDP Destination Port
   */
  void onControlSampleRequest(
    const ControlSampleRequest * request, const asip::address_v4 & livox_ip, const asip::address_v4 & user_ip,
    uint16_t data_port);

  /**
   * @brief Add radio button to group.
   */
  void addButtonGroup();

  /**
   * @brief Enumerate network interface.
   */
  void getNetworkInterfaces();

  /**
   * @brief Read pcap.
   * @param[in] fileName the name of the file to open
   */
  void readPcap(const QString & fileName);

  /**
   * @brief Get packet count from pcap.
   * @param[in] fname the name of the file to open
   * @return packet count on success, -1 on failure.
   */
  int getPcapPacketCount(const char * fname);

  /**
   * @brief Get packet data from pcap.
   * @param[in] fname the name of the file to open
   * @return packet count on success, -1 on failure.
   */
  int getPcapPacketData(const char * fname);

  /**
   * @brief Add UDP information.
   * @param[in] ip a pointer to header of ip
   * @param[in] udp a pointer to header of udp
   */
  void addUDPInfo(const struct iphdr * ip, const struct udphdr * udp);

  /**
   * Add source addresses to combo box.
   */
  void addSourceAddresses();

  /**
   * Create filter expression
   */
  std::string createFilter();

  Ui::FakeLivoxWidget * ui;                                      //!< @brief UI
  QButtonGroup * buttonGroup_system_status_;                     //!< @brief QButtonGroup
  std::map<std::string, int> status_codes_;                      //!< @brief LiDAR status_codes
  pcap_t * pcap_;                                                //!< @brief Descriptor of an open capture instance
  int packet_count_;                                             //!< @brief packet count in pcap
  UDPListModel * model_;                                         //!< @brief list model for UDP information
  SDKProtocol sdk_protocol_;                                     //!< @brief Handling SDK protocol class
  FakePointCloud point_cloud_;                                   //!< @brief Fake point cloud sampling class
  bool loop_;                                                    //!< @brief loop playback
  std::vector<NetworkInterface> network_list_;                   //!< @brief list of network interface
  std::map<LidarStatusCode, LidarStatusCodeValue> status_code_;  //!< @brief LiDAR Status Code
};
