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

#ifndef FAKE_LIVOX_INCLUDE_RQT_FAKE_LIVOX_RQT_FAKE_LIVOX_WIDGET_H_
#define FAKE_LIVOX_INCLUDE_RQT_FAKE_LIVOX_RQT_FAKE_LIVOX_WIDGET_H_

#include <stdint.h>
#include <boost/asio.hpp>
#include <map>
#include <string>
#include <vector>

#include <FastCRC.h>
#include <livox.h>
#include <QWidget>

/**
 * @brief CMD Set & CMD ID.
 */
struct CMD_SET_ID
{
  uint8_t cmd_set_;
  uint8_t cmd_id_;

  CMD_SET_ID() : cmd_set_(0), cmd_id_(0) {}

  CMD_SET_ID(uint8_t cmd_set, uint8_t cmd_id) : cmd_set_(cmd_set), cmd_id_(cmd_id) {}

  bool operator<(const CMD_SET_ID & value) const
  {
    if (cmd_set_ == value.cmd_set_) {
      return cmd_id_ < value.cmd_id_;
    }
    return cmd_set_ < value.cmd_set_;
  }
};

namespace as = boost::asio;
namespace ip = boost::asio::ip;

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
   * @brief Set broadcast code.
   * @param [in] broadcast_code Broadcast code
   */
  void setBroadcastCode(const QString & broadcast_code);

  /**
   * @brief Get broadcast code.
   * @return Broadcast code
   */
  QString getBroadcastCode(void);

  /**
   * @brief Start UDP communication.
   * @return 0 on success, otherwise error
   */
  int start(void);

  /**
   * @brief Stop UDP communication.
   */
  void stop(void);

private slots:
  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_pushButton_comm_toggled(bool checked);

  /**
   * @brief Get broadcast code thread-safely.
   * @return Broadcast code
   */
  QString get_broadcast_code(void);

  /**
   * @brief Get Checksum error thread-safely.
   * @return Checksum error
   */
  bool get_checksum_error(void);

  /**
   * @brief Get Debug output thread-safely.
   * @return Debug output
   */
  bool get_debug_output(void);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_comboBox_lidar_state_currentIndexChanged(int index);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_radioButton_temp_status_0_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_radioButton_temp_status_1_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_radioButton_temp_status_2_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_radioButton_volt_status_0_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_radioButton_volt_status_1_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_radioButton_volt_status_2_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_radioButton_motor_status_0_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_radioButton_motor_status_1_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_radioButton_motor_status_2_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_radioButton_dirty_warn_0_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_radioButton_dirty_warn_1_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_radioButton_device_status_0_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_radioButton_device_status_1_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_radioButton_fan_status_0_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_radioButton_fan_status_1_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_radioButton_firmware_status_0_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_radioButton_firmware_status_1_toggled(bool checked);

  /**
   * @brief Get Return Code thread-safely.
   * @return Return Code
   */
  int get_return_code(void);

  /**
   * @brief Get LiDAR State thread-safely.
   * @return LiDAR State
   */
  int get_lidar_state(void);

  /**
   * @brief Get Rain/Fog Suppression Switch thread-safely.
   * @return Rain/Fog Suppression Switch
   */
  int get_rain_fog_suppression_switch(void);

  /**
   * @brief Get Initialization Progress thread-safely.
   * @return Initialization Progress
   */
  int get_initialization_progress(void);

  /**
   * @brief Get temp_status thread-safely.
   * @return temp_status
   */
  int get_temp_status(void);

  /**
   * @brief Get volt_status thread-safely.
   * @return volt_status
   */
  int get_volt_status(void);

  /**
   * @brief Get motor_status thread-safely.
   * @return volt_status
   */
  int get_motor_status(void);

  /**
   * @brief Get dirty_warn thread-safely.
   * @return dirty_warn
   */
  int get_dirty_warn(void);

  /**
   * @brief Get firmware_status thread-safely.
   * @return firmware_status
   */
  int get_firmware_status(void);

  /**
   * @brief Get pps_status thread-safely.
   * @return pps_status
   */
  int get_pps_status(void);

  /**
   * @brief Get device_status thread-safely.
   * @return device_status
   */
  int get_device_status(void);

  /**
   * @brief Get fan_status thread-safely.
   * @return fan_status
   */
  int get_fan_status(void);

  /**
   * @brief Get self_heating thread-safely.
   * @return self_heating
   */
  int get_self_heating(void);

  /**
   * @brief Get ptp_status thread-safely.
   * @return ptp_status
   */
  int get_ptp_status(void);

  /**
   * @brief Get time_sync_status thread-safely.
   * @return time_sync_status
   */
  int get_time_sync_status(void);

  /**
   * @brief Get get_system_status thread-safely.
   * @return get_system_status
   */
  int get_system_status(void);

Q_SIGNALS:
  /**
   * @brief Get broadcast code thread-safely.
   * @return Broadcast code
   */
  QString signal_get_broadcast_code(void);

  /**
   * @brief Get Checksum error thread-safely.
   * @return Checksum error
   */
  bool signal_get_checksum_error(void);

  /**
   * @brief Get Debug output thread-safely.
   * @return Debug output
   */
  bool signal_get_debug_output(void);

  /**
   * @brief Get Return Code thread-safely.
   * @return Return Code
   */
  int signal_get_return_code(void);

  /**
   * @brief Get LiDAR State thread-safely.
   * @return LiDAR State
   */
  int signal_get_lidar_state(void);

  /**
   * @brief Get Rain/Fog Suppression Switch thread-safely.
   * @return Rain/Fog Suppression Switch
   */
  int signal_get_rain_fog_suppression_switch(void);

  /**
   * @brief Get Initialization Progress thread-safely.
   * @return Initialization Progress
   */
  int signal_get_initialization_progress(void);

  /**
   * @brief Get temp_status thread-safely.
   * @return temp_status
   */
  int signal_get_temp_status(void);

  /**
   * @brief Get volt_status thread-safely.
   * @return volt_status
   */
  int signal_get_volt_status(void);

  /**
   * @brief Get motor_status thread-safely.
   * @return volt_status
   */
  int signal_get_motor_status(void);

  /**
   * @brief Get dirty_warn thread-safely.
   * @return dirty_warn
   */
  int signal_get_dirty_warn(void);

  /**
   * @brief Get firmware_status thread-safely.
   * @return firmware_status
   */
  int signal_get_firmware_status(void);

  /**
   * @brief Get pps_status thread-safely.
   * @return pps_status
   */
  int signal_get_pps_status(void);

  /**
   * @brief Get device_status thread-safely.
   * @return device_status
   */
  int signal_get_device_status(void);

  /**
   * @brief Get fan_status thread-safely.
   * @return fan_status
   */
  int signal_get_fan_status(void);

  /**
   * @brief Get self_heating thread-safely.
   * @return self_heating
   */
  int signal_get_self_heating(void);

  /**
   * @brief Get ptp_status thread-safely.
   * @return ptp_status
   */
  int signal_get_ptp_status(void);

  /**
   * @brief Get time_sync_status thread-safely.
   * @return time_sync_status
   */
  int signal_get_time_sync_status(void);

  /**
   * @brief Get get_system_status thread-safely.
   * @return get_system_status
   */
  int signal_get_system_status(void);

private:
  /**
   * @brief I/O direction.
   */
  enum Direction {
    Read = 0,
    Write,
  };

  typedef void (FakeLivoxWidget::*HANDLE_FUNC)(SdkPacket * packet);  //!< @brief message handler

  /**
   * @brief Thread helper funcion.
   * @param [in] arg Argument
   */
  static void * threadHelper(void * arg) { return reinterpret_cast<FakeLivoxWidget *>(arg)->thread(); }

  /**
   * @brief Thread loop.
   * @return nullptr
   */
  void * thread(void);

  /**
   * @brief Dump sent/received Data.
   * @param [in] dir I/O direction
   * @param [in] data Pointer to data
   * @param [in] size Size of data
   */
  void dump(Direction dir, const uint8_t * data, std::size_t size);

  /**
   * @brief Handler to be called when the read operation completes.
   * @param [in] error Error argument of a handler
   * @param [in] bytes_transfered Bytes transferred argument of a handler
   * @param [inout] data Received data
   */
  void onRead(const boost::system::error_code & error, std::size_t bytes_transfered, const uint8_t * data);

  /**
   * @brief Handler to be called when the write operation completes.
   * @param [in] error Error argument of a handler
   * @param [in] bytes_transfered Bytes transferred argument of a handler
   * @param [inout] data Sent data
   */
  void onWrite(
    const boost::system::error_code & error, std::size_t bytes_transfered, const std::vector<uint8_t> & data);

  /**
   * @brief Handle sdk packet.
   * @param[in] packet Received packet
   */
  void handleSdkPacket(SdkPacket * packet);

  /**
   * @brief Handle handshake command.
   * @param[in] packet Received packet
   */
  void handleGeneralHandshake(SdkPacket * packet);

  /**
   * @brief Handle query the information of device.
   * @param[in] packet Received packet
   */
  void handleUGeneralDeviceInfo(SdkPacket * packet);

  /**
   * @brief Handle heartbeat command.
   * @param[in] packet Received packet
   */
  void handleGeneralHeartbeat(SdkPacket * packet);

  /**
   * @brief Handle Start/Stop Sampling.
   * @param[in] packet Received packet
   */
  void handleGeneralControlSample(SdkPacket * packet);

  /**
   * @brief Handle coordinate of point cloud data.
   * @param[in] packet Received packet
   */
  void handleGeneralCoordinateSystem(SdkPacket * packet);

  /**
   * @brief Handle set point cloud return mode of a LiDAR.
   * @param[in] packet Received packet
   */
  void handleLidarSetPointCloudReturnMode(SdkPacket * packet);

  /**
   * @brief Handle set IMU push frequency of a LiDAR.
   * @param[in] packet Received packet
   */
  void handleLidarSetImuPushFrequency(SdkPacket * packet);

  /**
   * @brief Send data.
   * @param ep Endpoint
   * @param packet Pointer to protocol frame
   * @param payload Pointer to payload
   * @param payload_size Size of payload
   */
  void send(ip::udp::endpoint ep, SdkPacket * packet, uint8_t * payload, uint16_t payload_size);

  /**
   * @brief Send broadcast command.
   */
  void sendGeneralBroadcast(void);

  /**
   * @brief Send Push Abnormal Status.
   */
  void sendPushAbnormalStatus(void);

  /**
   * @brief Update system_status.
   */
  void updateSystemStatus(void);

  Ui::FakeLivoxWidget * ui;                              //!< @brief UI
  as::io_service io_;                                    //!< @brief facilities of custom asynchronous services
  boost::shared_ptr<ip::udp::socket> socket_;            //!< @brief socket
  pthread_mutex_t mutex_stop_;                           //!< @brief mutex to protect access to stop_thread
  pthread_t th_;                                         //!< @brief thread handle
  pthread_t * th_ptr_;                                   //!< @brief pointer to thread handle
  bool stop_thread_;                                     //!< @brief flag to stop thread
  static std::map<CMD_SET_ID, HANDLE_FUNC> handle_map_;  //!< @brief message handler map
  FastCRC16 crc16_;                                      //!< @brief 16-BIT CRC
  FastCRC32 crc32_;                                      //!< @brief 32-BIT CRC
  ip::address_v4 user_ip_;                               //!< @brief Host IPAddress
  uint16_t data_port_;                                   // @brief Host Point Cloud Data UDP Destination Port
  uint16_t cmd_port_;                                    // @brief Host Control Command UDP Destination Port
  std::map<std::string, int> status_codes_;              // @brief LiDAR status_codes
};

#endif  // FAKE_LIVOX_INCLUDE_RQT_FAKE_LIVOX_RQT_FAKE_LIVOX_WIDGET_H_
