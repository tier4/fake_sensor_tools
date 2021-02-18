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
 * @file udp_info.h
 * @brief UDP info class
 */

#ifndef FAKE_LIVOX_INCLUDE_RQT_FAKE_LIVOX_UDP_INFO_H_
#define FAKE_LIVOX_INCLUDE_RQT_FAKE_LIVOX_UDP_INFO_H_

#include <QObject>
#include <QString>

class UDPInfo
{
public:
  /**
   * @brief Constructor.
   */
  UDPInfo() : packet_count_(0), transmit_(false) {}

  /**
   * @brief Constructor.
   * @param[in] source_address source address of packet
   * @param[in] source_port source port of packet
   * @param[in] dest_address destination address of packet
   * @param[in] dest_port destination port of packet
   * @param[in] packet_count number of packets
   * @param[in] parent QObject
   */
  UDPInfo(
    const QString & source_address, const int source_port, const QString & dest_address, const int dest_port,
    const int packet_count, QObject * parent = nullptr)
  : UDPInfo()
  {
    source_address_ = source_address;
    source_port_ = source_port;
    dest_address_ = dest_address;
    dest_port_ = dest_port;
    packet_count_ = packet_count;
  }

  /**
   * @brief Equal to operator.
   * @param[in] other other
   * @return true if this is equal to other, false otherwise
   */
  bool operator==(const UDPInfo & other) const
  {
    return (
      (source_address_ == other.source_address_) && (source_port_ == other.source_port_) &&
      (dest_address_ == other.dest_address_) && (dest_port_ == other.dest_port_));
  }

  /**
   * @brief Get source address of packet.
   * @return source address of packet
   */
  const QString & getSourceAddress() const { return source_address_; }

  /**
   * @brief Get source port of packet.
   * @return source address of packet
   */
  const int getSourcePort() const { return source_port_; }

  /**
   * @brief Get destination address of packet.
   * @return source address of packet
   */
  const QString & getDestAddress() const { return dest_address_; }

  /**
   * @brief Get destination port of packet.
   * @return source address of packet
   */
  const int getDestPort() const { return dest_port_; }

  /**
   * @brief Get number of packets.
   * @return number of packets
   */
  const int getPacketCount() const { return packet_count_; }

  /**
   * @brief Increment number of packets.
   */
  void incrementPacketCount() { ++packet_count_; }

  /**
   * @brief Get transmit or not.
   * @return transmit or not
   */
  const bool getTransmit() const { return transmit_; }

  /**
   * @brief Set transmit or not.
   * @param[in] transmit transmit or not
   */
  void setTransmit(bool transmit) { transmit_ = transmit; }

  /**
   * @brief Toggle transmit or not.
   */
  void toggleTransmit() { transmit_ = !transmit_; }

private:
  QString source_address_;  //!< @brief source address of packet
  int source_port_;         //!< @brief source port of packet
  QString dest_address_;    //!< @brief destination address of packet
  int dest_port_;           //!< @brief destination port of packet
  int packet_count_;        //!< @brief number of packets
  bool transmit_;           //!< @brief transmit or not
};

#endif  // FAKE_LIVOX_INCLUDE_RQT_FAKE_LIVOX_UDP_INFO_H_
