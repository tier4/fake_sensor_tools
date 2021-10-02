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
 * @file fake_point_cloud.h
 * @brief Fake point cloud sampling class
 */

#pragma once

#include <mutex>
#include <string>

#include <netinet/ip.h>
#include <netinet/udp.h>
#include <pcap/pcap.h>
#include <boost/asio.hpp>

namespace as = boost::asio;
namespace asip = boost::asio::ip;

class FakePointCloud
{
public:
  /**
   * @brief Constructor.
   */
  FakePointCloud();

  /**
   * @brief Destructor.
   */
  ~FakePointCloud();

  /**
   * @brief Start point cloud sampling.
   * @param[in] pcap_path path of pcap file
   * @param[in] pcap_filter filter program for pcap
   * @param[in] livox_ip IP address of Livox
   * @param[in] user_ip Host IPAddress
   * @param[in] data_port Host Point Cloud Data UDP Destination Port
   * @param[in] loop loop playback
   * @return 0 on success, otherwise error
   */
  int start(
    const std::string & pcap_path, const std::string & pcap_filter, const asip::address_v4 & livox_ip,
    const asip::address_v4 & user_ip, uint16_t data_port, bool loop);

  /**
   * @brief Stop point cloud sampling.
   */
  void stop();

  /**
   * @brief Set LiDAR Status Code.
   * @param[in] status_code value of LiDAR Status Code
   */
  void setLidarStatusCode(uint32_t status_code);

private:
  /**
   * @brief Thread helper funcion.
   * @param [in] arg Argument
   */
  static void * threadHelper(void * arg) { return reinterpret_cast<FakePointCloud *>(arg)->thread(); }

  /**
   * @brief Thread loop.
   * @return nullptr
   */
  void * thread();

  /**
   * @brief Open pcap file.
   * @return 0 on success, otherwise error
   */
  int openPcap();

  /**
   * @brief Close pcap file.
   */
  void closePcap();

  /**
   * @brief Read pcap file and send packet.
   */
  void performPcap();

  /**
   * @brief Send packet.
   * @param packet Pointer to packet
   */
  void send(const u_char * packet);

  /**
   * @brief Wait packet time.
   * @param t time
   * @param reset flag to reset
   */
  void packetTimer(struct timeval t, int reset);

  as::io_service io_;                            //!< @brief facilities of custom asynchronous services
  boost::shared_ptr<asip::udp::socket> socket_;  //!< @brief socket
  std::mutex mutex_stop_;                        //!< @brief mutex to protect access to stop_thread
  pthread_t th_;                                 //!< @brief thread handle
  pthread_t * th_ptr_;                           //!< @brief pointer to thread handle
  bool stop_thread_;                             //!< @brief flag to stop thread
  asip::address_v4 user_ip_;                     //!< @brief Host IPAddress
  uint16_t data_port_;                           //!< @brief Host Point Cloud Data UDP Destination Port
  pcap_t * pcap_;                                //!< @brief Descriptor of an open capture instance
  std::string pcap_path_;                        //!< @brief path of pcap file
  std::string pcap_filter_;                      //!< @brief filter program for pcap
  bool loop_;                                    //!< @brief loop playback
  uint32_t status_code_;                         //!< @brief LiDAR Status Code
  std::mutex mutex_config_;                      //!< @brief mutex to protect access to configuration
};
