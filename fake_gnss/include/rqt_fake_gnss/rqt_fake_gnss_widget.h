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
 * @file rqt_fake_gnss_widget.h
 * @brief RQt plugin widget class
 */

#ifndef FAKE_GNSS_INCLUDE_RQT_FAKE_GNSS_RQT_FAKE_GNSS_WIDGET_H_
#define FAKE_GNSS_INCLUDE_RQT_FAKE_GNSS_RQT_FAKE_GNSS_WIDGET_H_

#include <QWidget>
#include <boost/asio.hpp>
#include <map>
#include <string>
#include <vector>

/**
 * @brief UBX Message Class and ID.
 */
struct UBX_ID
{
  uint8_t classId_;
  uint8_t messageId_;

  UBX_ID() : classId_(0), messageId_(0) {}

  UBX_ID(uint8_t classId, uint8_t messageId) : classId_(classId), messageId_(messageId) {}

  bool operator<(const UBX_ID & value) const
  {
    if (classId_ == value.classId_) {
      return messageId_ < value.messageId_;
    }
    return classId_ < value.classId_;
  }
};

/**
 * @brief Port block information of UBX-MON-COMMS.
 */
typedef struct
{
  bool port_enabled;
  int tx_usage;
} PortBlock;

namespace as = boost::asio;

namespace Ui
{
class FakeGnssWidget;
}

class FakeGnssWidget : public QWidget
{
  Q_OBJECT

public:
  /**
   * @brief Constructs a widget which is a child of parent.
   * @param [in] parent The new widget becomes a window
   */
  explicit FakeGnssWidget(QWidget * parent = nullptr);

  /**
   * @brief Destructor.
   */
  ~FakeGnssWidget();

  /**
   * @brief Set device name.
   * @param [in] device_name Device name
   */
  void setDeviceName(const QString & device_name);

  /**
   * @brief Get device name.
   * @return Device name
   */
  QString getDeviceName(void);

  /**
   * @brief Start serial port communication.
   * @return 0 on success, otherwise error
   */
  int start(void);

  /**
   * @brief Stop serial port communication.
   */
  void stop(void);

private slots:
  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_pushButton_serial_port_toggled(bool checked);

  /**
   * @brief This signal is sent whenever currentText changes.
   * @param [in] arg1 The new value is passed as text
   */
  void on_comboBox_port_id_currentIndexChanged(const QString & arg1);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_pushButton_enabled_toggled(bool checked);

  /**
   * @brief This signal is emitted when the slider value has changed.
   * @param [in] value The new slider value 
   */
  void on_slider_tx_usage_valueChanged(double value);

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
   * @brief Get aStatus thread-safely.
   * @return aStatus
   */
  int get_a_status(void);

  /**
   * @brief Get jammingState thread-safely.
   * @return jammingState
   */
  int get_jamming_state(void);

  /**
   * @brief Get spoofDetState thread-safely.
   * @return spoofDetState
   */
  int get_spoof_det_state(void);

Q_SIGNALS:
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
   * @brief Get aStatus thread-safely.
   * @return aStatus
   */
  int signal_get_a_status(void);

  /**
   * @brief Get jammingState thread-safely.
   * @return jammingState
   */
  int signal_get_jamming_state(void);

  /**
   * @brief Get spoofDetState thread-safely.
   * @return spoofDetState
   */
  int signal_get_spoof_det_state(void);

private:
  /**
   * @brief io direction.
   */
  enum Direction {
    Read = 0,
    Write,
  };

  typedef void (FakeGnssWidget::*HANDLE_FUNC)(const uint8_t * data);  //!< @brief message handler
  typedef void (FakeGnssWidget::*TRANSMIT_FUNC)();                    //!< @brief transmit function

  /**
   * @brief Periodic transmission.
   */
  typedef struct
  {
    TRANSMIT_FUNC func_;
    int rate_;
    int cnt_;
  } PERIODIC_TRANSMIT;

  /**
   * @brief Thread helper funcion.
   * @param[in] arg Argument
   */
  static void * threadHelper(void * arg)
  {
    return reinterpret_cast<FakeGnssWidget *>(arg)->thread();
  }

  /**
   * @brief Thread loop.
   * @return nullptr
   */
  void * thread(void);

  /**
   * @brief Handle periodic transmission.
   */
  void handlePeriodicTransmit(void);

  /**
   * @brief Dump sent/received Data.
   * @param[in] dir I/O direction
   * @param[in] data Pointer to data
   * @param[in] size Size of data
   */
  void dump(Direction dir, const uint8_t * data, std::size_t size);

  /**
   * @brief Handler to be called when the read operation completes.
   * @param[in] error Error argument of a handler
   * @param[in] bytes_transfered Bytes transferred argument of a handler
   * @param[inout] data Received data
   */
  void onRead(
    const boost::system::error_code & error, std::size_t bytes_transfered, const uint8_t * data);

  /**
   * @brief Handler to be called when the write operation completes.
   * @param[in] error Error argument of a handler
   * @param[in] bytes_transfered Bytes transferred argument of a handler
   * @param[inout] data Sent data
   */
  void onWrite(
    const boost::system::error_code & error, std::size_t bytes_transfered,
    const std::vector<uint8_t> & data);

  /**
   * @brief Handle UBX data.
   * @param[in] data Received data
   */
  void handleUbx(const uint8_t * data);

  /**
   * @brief Handle UBX-MON-VER.
   * @param[in] data Received data
   */
  void handleUbxMonVER(const uint8_t * data);

  /**
   * @brief Handle UBX-CFG-PRT.
   * @param[in] data Received data
   */
  void handleUbxCfgPRT(const uint8_t * data);

  /**
   * @brief Handle UBX-CFG-MSG.
   * @param[in] data Received data
   */
  void handleUbxCfgMSG(const uint8_t * data);

  /**
   * @brief Send UBX-ACK-ACK.
   * @param[in] ack true on UBX-ACK-ACK, false on UBX-ACK-NAK
   * @param[in] message_class Message class
   * @param[in] message_id Message id
   */
  void sendUbxAck(bool ack, uint8_t message_class, uint8_t message_id);

  /**
   * @brief Send UBX-NAV-STATUS.
   */
  void sendUbxNavSTATUS(void);

  /**
   * @brief Send UBX-NAV-PVT.
   */
  void sendUbxNavPVT(void);

  /**
   * @brief Send UBX-NAV-RELPOSNED.
   */
  void sendUbxNavRELPOSNED(void);

  /**
   * @brief Send UBX-MON-HW.
   */
  void sendUbxMonHW(void);

  /**
   * @brief Send UBX-MON-COMMS.
   */
  void sendUbxMonCOMMS(void);

  /**
   * @brief Send data.
   * @param data Start of range
   * @param size Size of range
   */
  void send(uint8_t * data, int size);

  /**
   * @brief Calculate checksum.
   * @param data Start of range
   * @param size Size of range
   * @param ck_a Checksum a
   * @param ck_b Checksum b
   */
  void calculateChecksum(const uint8_t * data, int size, uint8_t & ck_a, uint8_t & ck_b);

  Ui::FakeGnssWidget * ui;                   //!< @brief UI
  as::io_service io_;                        //!< @brief facilities of custom asynchronous services
  boost::shared_ptr<as::serial_port> port_;  //!< @brief wrapper over serial port functionality
  pthread_mutex_t mutex_stop_;               //!< @brief mutex to protect access to stop_thread
  pthread_mutex_t mutex_port_;               //!< @brief mutex to protect access to port_blocks
  pthread_t th_;                             //!< @brief thread handle
  pthread_t * th_ptr_;                       //!< @brief pointer to thread handle
  bool stop_thread_;                         //!< @brief flag to stop thread
  static std::map<UBX_ID, HANDLE_FUNC> handle_map_;          //!< @brief message handler map
  static std::map<UBX_ID, PERIODIC_TRANSMIT> periodic_map_;  //!< @brief Periodic transmission map
  static std::map<std::string, PortBlock> port_blocks_;      //!< @brief Port blocks
};

#endif  // FAKE_GNSS_INCLUDE_RQT_FAKE_GNSS_RQT_FAKE_GNSS_WIDGET_H_
