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
 * @file rqt_fake_imu_widget.h
 * @brief RQt plugin widget class
 */

#ifndef FAKE_IMU_INCLUDE_RQT_FAKE_IMU_RQT_FAKE_IMU_WIDGET_H_
#define FAKE_IMU_INCLUDE_RQT_FAKE_IMU_RQT_FAKE_IMU_WIDGET_H_

#include <QWidget>
#include <boost/asio.hpp>
#include <vector>

namespace as = boost::asio;

namespace Ui
{
class FakeImuWidget;
}

class FakeImuWidget : public QWidget
{
  Q_OBJECT

public:
  /**
   * @brief Constructs a widget which is a child of parent.
   * @param [in] parent The new widget becomes a window
   */
  explicit FakeImuWidget(QWidget * parent = nullptr);

  /**
   * @brief Destructor.
   */
  ~FakeImuWidget();

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
   * @brief Get Checksum error thread-safely.
   * @return Checksum error
   */
  bool get_checksum_error(void);

  /**
   * @brief Get Debug output thread-safely.
   * @return Debug output
   */
  bool get_debug_output(void);

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

private:
  /**
   * @brief I/O direction.
   */
  enum Direction {
    Read = 0,
    Write,
  };

  /**
   * @brief Thread helper funcion.
   * @param [in] arg Argument
   */
  static void * threadHelper(void * arg) { return reinterpret_cast<FakeImuWidget *>(arg)->thread(); }

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
  void dump(Direction dir, const std::string & data, std::size_t size);

  /**
   * @brief Dump received BIN Data.
   * @param [in] data Pointer to data
   */
  void dumpBIN(const uint8_t * data);

  /**
   * @brief Handler to be called when the read operation completes.
   * @param [in] error Error argument of a handler
   * @param [in] bytes_transfered Bytes transferred argument of a handler
   * @param [inout] data Received data
   */
  void onRead(const boost::system::error_code & error, std::size_t bytes_transfered);

  /**
   * @brief Handler to be called when the write operation completes.
   * @param [in] error Error argument of a handler
   * @param [in] bytes_transfered Bytes transferred argument of a handler
   * @param [inout] data Sent data
   */
  void onWrite(
    const boost::system::error_code & error, std::size_t bytes_transfered, const std::vector<uint8_t> & data);

  Ui::FakeImuWidget * ui;                    //!< @brief UI
  as::io_service io_;                        //!< @brief facilities of custom asynchronous services
  boost::shared_ptr<as::serial_port> port_;  //!< @brief wrapper over serial port functionality
  as::streambuf buffer_;                     //!< @brief Received data
  pthread_mutex_t mutex_stop_;               //!< @brief mutex to protect access to stop_thread
  pthread_t th_;                             //!< @brief thread handle
  pthread_t * th_ptr_;                       //!< @brief pointer to thread handle
  bool stop_thread_;                         //!< @brief flag to stop thread
  bool bin_req_;                             //!< @brief flag of BIN request received
};

#endif  // FAKE_IMU_INCLUDE_RQT_FAKE_IMU_RQT_FAKE_IMU_WIDGET_H_
