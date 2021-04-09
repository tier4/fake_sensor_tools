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
 * @file rqt_fake_velodyne_widget.h
 * @brief RQt plugin widget class
 */

#ifndef FAKE_VELODYNE_INCLUDE_RQT_FAKE_VELODYNE_RQT_FAKE_VELODYNE_WIDGET_H_
#define FAKE_VELODYNE_INCLUDE_RQT_FAKE_VELODYNE_RQT_FAKE_VELODYNE_WIDGET_H_

#include <QWidget>

#include <cpprest/http_listener.h>
#include <string>

namespace http = web::http;
namespace json = web::json;

namespace Ui
{
class FakeVelodyneWidget;
}

class FakeVelodyneWidget : public QWidget
{
  Q_OBJECT

public:
  /**
   * @brief Constructs a widget which is a child of parent.
   * @param [in] parent The new widget becomes a window
   */
  explicit FakeVelodyneWidget(QWidget * parent = nullptr);

  /**
   * @brief Destructor.
   */
  ~FakeVelodyneWidget();

  /**
   * @brief Set server address.
   * @param [in] address Server address
   */
  void setAddress(const QString & address);

  /**
   * @brief Get server address.
   * @return Server addess
   */
  QString getAddress();

  /**
   * @brief Start HTTP server.
   * @return 0 on success, otherwise error
   */
  int start();

  /**
   * @brief Stop HTTP server.
   */
  void stop();

private slots:
  /**
   * @brief This signal is sent whenever currentText changes.
   * @param [in] arg1 The new value is passed as text
   */
  void on_comboBox_type_currentIndexChanged(const QString & arg1);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_pushButton_server_toggled(bool checked);

  /**
   * @brief This signal is emitted whenever the text changes.
   * @param [in] arg1 The new text
   */
  void on_lineEdit_model_textChanged(const QString & arg1);

  /**
   * @brief This signal is emitted whenever the text changes.
   * @param [in] arg1 The new text
   */
  void on_lineEdit_serial_textChanged(const QString & arg1);

  /**
   * @brief Notify a change of value.
   * @param [in] value New value
   */
  void on_slider_top_hv_valueChanged(double value);

  /**
   * @brief Notify a change of value.
   * @param [in] value New value
   */
  void on_slider_top_temp_valueChanged(double value);

  /**
   * @brief Notify a change of value.
   * @param [in] value New value
   */
  void on_slider_top_5v_valueChanged(double value);

  /**
   * @brief Notify a change of value.
   * @param [in] value New value
   */
  void on_slider_top_2_5v_valueChanged(double value);

  /**
   * @brief Notify a change of value.
   * @param [in] value New value
   */
  void on_slider_top_3_3v_valueChanged(double value);

  /**
   * @brief Notify a change of value.
   * @param [in] value New value
   */
  void on_slider_top_5v_raw_valueChanged(double value);

  /**
   * @brief Notify a change of value.
   * @param [in] value New value
   */
  void on_slider_top_vcc_valueChanged(double value);

  /**
   * @brief Notify a change of value.
   * @param [in] value New value
   */
  void on_slider_bot_i_out_valueChanged(double value);

  /**
   * @brief Notify a change of value.
   * @param [in] value New value
   */
  void on_slider_bot_1_2v_valueChanged(double value);

  /**
   * @brief Notify a change of value.
   * @param [in] value New value
   */
  void on_slider_bot_temp_valueChanged(double value);

  /**
   * @brief Notify a change of value.
   * @param [in] value New value
   */
  void on_slider_bot_5v_valueChanged(double value);

  /**
   * @brief Notify a change of value.
   * @param [in] value New value
   */
  void on_slider_bot_2_5v_valueChanged(double value);

  /**
   * @brief Notify a change of value.
   * @param [in] value New value
   */
  void on_slider_bot_3_3v_valueChanged(double value);

  /**
   * @brief Notify a change of value.
   * @param [in] value New value
   */
  void on_slider_bot_v_in_valueChanged(double value);

  /**
   * @brief Notify a change of value.
   * @param [in] value New value
   */
  void on_slider_bot_1_25v_valueChanged(double value);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_pushButton_motor_state_toggled(bool checked);

  /**
   * @brief Notify a change of value.
   * @param [in] value New value
   */
  void on_slider_motor_rpm_valueChanged(double value);

  /**
   * @brief This signal is emitted whenever a checkable button changes its state.
   * @param [in] checked true if the button is checked, or false if the button is unchecked
   */
  void on_pushButton_laser_state_toggled(bool checked);

  /**
   * @brief Notify a change of value.
   * @param [in] value New value
   */
  void on_slider_motor_rpm_setting_valueChanged(double value);

  /**
   * @brief Get server address thread-safely.
   * @return Server address
   */
  QString get_server_address();

Q_SIGNALS:
  /**
   * @brief Get server address thread-safely.
   * @return Server address
   */
  QString signal_get_server_address();

private:
  /**
   * @brief Thread helper funcion.
   * @param[in] arg Argument
   */
  static void * threadHelper(void * arg)
  {
    return reinterpret_cast<FakeVelodyneWidget *>(arg)->thread();
  }

  /**
   * @brief Thread loop.
   * @return nullptr
   */
  void * thread();

  /**
   * @brief Handle 'GET' request.
   * @param [in] request HTTP request with the 'GET' method
   */
  void handleGet(http::http_request request);

  /**
   * @brief Load data from info.json.
   */
  void loadInfoJson();

  /**
   * @brief Load data from diag.json.
   */
  void loadDiagJson();

  /**
   * @brief Load data from status.json.
   */
  void loadStatusJson();

  /**
   * @brief Load data from settings.json.
   */
  void loadSettingsJson();

  /**
   * @brief Round off a number to specified place after decimal point.
   * @param [in] x Double value
   * @param [in] l Place after decimal point
   * @return Rounded number
   */
  double roundAt(double x, int k);

  Ui::FakeVelodyneWidget * ui;  //!< @brief UI
  pthread_mutex_t mutex_stop_;  //!< @brief mutex to protect access to stop_thread
  pthread_mutex_t mutex_json_;  //!< @brief mutex to protect access to json
  pthread_t th_;                //!< @brief thread handle
  pthread_t * th_ptr_;          //!< @brief pointer to thread handle
  bool stop_thread_;            //!< @brief flag to stop thread

  json::value info_json_;      //!< @brief values of info.json
  json::value diag_json_;      //!< @brief values of diag.json
  json::value status_json_;    //!< @brief values of diag.json
  json::value settings_json_;  //!< @brief values of settings.json
};

#endif  // FAKE_VELODYNE_INCLUDE_RQT_FAKE_VELODYNE_RQT_FAKE_VELODYNE_WIDGET_H_
