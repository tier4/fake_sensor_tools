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
 * @file sdk_protocol.h
 * @brief Handling SDK protocol class
 */

#pragma once

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <netinet/ip.h>
#include <netinet/udp.h>
#include <pcap/pcap.h>
#include <boost/asio.hpp>
#include <boost/function.hpp>

#include <FastCRC.h>
#include <fake_point_cloud.h>
#include <lidar_status.h>
#include <livox.h>

/**
 * @brief CMD Set & CMD ID.
 */
struct CmdSetId
{
  uint8_t cmd_set_;
  uint8_t cmd_id_;

  CmdSetId() : cmd_set_(0), cmd_id_(0) {}

  CmdSetId(uint8_t cmd_set, uint8_t cmd_id) : cmd_set_(cmd_set), cmd_id_(cmd_id) {}

  bool operator<(const CmdSetId & value) const
  {
    if (cmd_set_ == value.cmd_set_) {
      return cmd_id_ < value.cmd_id_;
    }
    return cmd_set_ < value.cmd_set_;
  }
};

namespace as = boost::asio;
namespace asip = boost::asio::ip;

class SDKProtocol
{
public:
  /**
   * @brief Constructor.
   */
  SDKProtocol();

  /**
   * @brief Destructor.
   */
  ~SDKProtocol();

  /**
   * @brief Set LiDAR status.
   * @param[in] name name of LiDAR status
   * @param[in] value value of LiDAR status
   */
  void setLidarStatus(const LidarStatus & name, int value);

  /**
   * @brief Set system_status in LiDAR Status Code.
   * @return system_status
   */
  int getSystemStatus();

  typedef boost::function<void(
    const ControlSampleRequest * request, const asip::address_v4 & user_ip, uint16_t data_port)>
    ControlSampleRequestCallback;

  /**
   * @brief Set callback for Start/Stop Sampling.
   * @param[in] callback callback
   */
  void setCallback(const ControlSampleRequestCallback & callback) { callback_ = callback; }

  /**
   * @brief Start UDP communication.
   * @param[in] livox_ip_ IP address of Livox
   * @param[in] broadcast_code Broadcast code
   * @return 0 on success, otherwise error
   */
  int start(const asip::address_v4 & livox_ip, const std::string & broadcast_code);

  /**
   * @brief Stop UDP communication.
   */
  void stop();

private:
  /**
   * @brief I/O direction.
   */
  enum Direction {
    Read = 0,
    Write,
  };

  typedef void (SDKProtocol::*HandleFunction)(SdkPacket * packet);  //!< @brief message handler

  /**
   * @brief Thread helper funcion.
   * @param [in] arg Argument
   */
  static void * threadHelper(void * arg) { return reinterpret_cast<SDKProtocol *>(arg)->thread(); }

  /**
   * @brief Thread loop.
   * @return nullptr
   */
  void * thread();

  /**
   * @brief Initialize LiDAR status.
   */
  void initLidarStatus();

  /**
   * @brief Get LiDAR status thread-safely.
   * @param[in] name name of LiDAR status
   * @return value of LiDAR status
   */
  int getLidarStatus(const LidarStatus & name);

  /**
   * @brief Set system_status in LiDAR Status Code.
   */
  void setSystemStatus();

  /**
   * @brief Dump sent/received Data.
   * @param [in] dir I/O direction
   * @param [in] data Pointer to data
   * @param [in] size Size of data
   */
  void dump(Direction dir, const uint8_t * data, std::size_t size);

  /**
   * @brief Handler to be called when the timer expires.
   * @param [in] error Error argument of a handler
   */
  void onTimer(const boost::system::error_code & error);

  /**
   * @brief Handler to be called when the read operation completes.
   * @param [in] error Error argument of a handler
   * @param [in] bytes_transferred Bytes transferred argument of a handler
   * @param [inout] data Received data
   */
  void onRead(const boost::system::error_code & error, std::size_t bytes_transferred, const uint8_t * data);

  /**
   * @brief Handler to be called when the write operation completes.
   * @param [in] error Error argument of a handler
   * @param [in] bytes_transferred Bytes transferred argument of a handler
   * @param [inout] data Sent data
   */
  void onWrite(
    const boost::system::error_code & error, std::size_t bytes_transferred, const std::vector<uint8_t> & data);

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
  void handleGeneralDeviceInfo(SdkPacket * packet);

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
  void send(asip::udp::endpoint ep, SdkPacket * packet, uint8_t * payload, uint16_t payload_size);

  /**
   * @brief Send broadcast command.
   */
  void sendGeneralBroadcast();

  as::io_service io_;                                     //!< @brief facilities of custom asynchronous services
  boost::shared_ptr<asip::udp::socket> socket_;           //!< @brief socket
  std::mutex mutex_stop_;                                 //!< @brief mutex to protect access to stop_thread
  pthread_t th_;                                          //!< @brief thread handle
  pthread_t * th_ptr_;                                    //!< @brief pointer to thread handle
  bool stop_thread_;                                      //!< @brief flag to stop thread
  boost::shared_ptr<as::deadline_timer> timer_;           //!< @brief periodic timer
  static std::map<CmdSetId, HandleFunction> handle_map_;  //!< @brief message handler map
  FastCRC16 crc16_;                                       //!< @brief 16-BIT CRC
  FastCRC32 crc32_;                                       //!< @brief 32-BIT CRC
  asip::address_v4 user_ip_;                              //!< @brief Host IPAddress
  uint16_t data_port_;                                    //!< @brief Host Point Cloud Data UDP Destination Port
  uint16_t cmd_port_;                                     //!< @brief Host Control Command UDP Destination Port
  FakePointCloud point_cloud_;                            //!< @brief Fake point cloud sampling class

  std::mutex mutex_config_;                   //!< @brief mutex to protect access to configuration
  std::string broadcast_code_;                //!< @brief Broadcast code
  std::map<LidarStatus, LidarValue> status_;  //!< @brief LiDAR status
  ControlSampleRequestCallback callback_;     //!< @brief callback
};
