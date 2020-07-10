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
 * @file rqt_fake_velodyne_widget.cpp
 * @brief RQt plugin widget class
 */

#include <QCoreApplication>
#include <QFile>
#include <boost/process.hpp>
#include <boost/thread.hpp>

#include <rqt_fake_velodyne/rqt_fake_velodyne_widget.h>
#include <ui_rqt_fake_velodyne_widget.h>

FakeVelodyneWidget::FakeVelodyneWidget(QWidget * parent)
: QWidget(parent),
  ui(new Ui::FakeVelodyneWidget),
  mutex_stop_(),
  mutex_json_(),
  stop_thread_(false),
  th_ptr_(nullptr)
{
  ui->setupUi(this);

  ui->comboBox_type->addItem("VLP-16");
  ui->comboBox_type->addItem("VLP-32C");
  ui->comboBox_type->addItem("VLS-128-AP");

  connect(
    this, SIGNAL(signal_get_server_address()), this, SLOT(get_server_address()),
    Qt::BlockingQueuedConnection);

  // Load data from info.json
  loadInfoJson();
  // Load data from diag.json
  loadDiagJson();
  // Load data from status.json
  loadStatusJson();
  // Load data from settings.json
  loadSettingsJson();
}

FakeVelodyneWidget::~FakeVelodyneWidget() { delete ui; }

void FakeVelodyneWidget::on_comboBox_type_currentIndexChanged(const QString & arg1)
{
  QPixmap pix(":/images/" + arg1 + ".png");
  ui->label_type->setPixmap(pix);
  ui->lineEdit_model->setText(arg1);
}

void FakeVelodyneWidget::on_pushButton_server_toggled(bool checked)
{
  if (checked) {
    // Start HTTP server
    start();
  } else {
    // Stop HTTP server
    stop();
  }
}

void FakeVelodyneWidget::on_lineEdit_model_textChanged(const QString & arg1)
{
  pthread_mutex_lock(&mutex_json_);
  info_json_["model"] = json::value::string(arg1.toStdString());
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_lineEdit_serial_textChanged(const QString & arg1)
{
  pthread_mutex_lock(&mutex_json_);
  info_json_["serial"] = json::value::string(arg1.toStdString());
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_slider_top_hv_valueChanged(double value)
{
  int raw = 4096 * (value / 101.0 + 5.0) / 5.0;

  pthread_mutex_lock(&mutex_json_);
  diag_json_["volt_temp"]["top"]["hv"] = raw;
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_slider_top_temp_valueChanged(double value)
{
  double d = roundAt(1.8639 - (std::pow(value + 1481.96, 2.0) - 2.1962e6) * 3.88e-6, 4);
  int raw = roundAt(4096 * d / 5.0, 3);

  pthread_mutex_lock(&mutex_json_);
  diag_json_["volt_temp"]["top"]["lm20_temp"] = raw;
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_slider_top_5v_valueChanged(double value)
{
  int raw = 4096 * (value / 2.0) / 5.0;

  pthread_mutex_lock(&mutex_json_);
  diag_json_["volt_temp"]["top"]["pwr_5v"] = raw;
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_slider_top_2_5v_valueChanged(double value)
{
  int raw = 4096 * value / 5.0;

  pthread_mutex_lock(&mutex_json_);
  diag_json_["volt_temp"]["top"]["pwr_2_5v"] = raw;
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_slider_top_3_3v_valueChanged(double value)
{
  int raw = 4096 * value / 5.0;

  pthread_mutex_lock(&mutex_json_);
  diag_json_["volt_temp"]["top"]["pwr_3_3v"] = raw;
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_slider_top_5v_raw_valueChanged(double value)
{
  int raw = 4096 * (value / 2.0) / 5.0;

  pthread_mutex_lock(&mutex_json_);
  diag_json_["volt_temp"]["top"]["pwr_5v_raw"] = raw;
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_slider_top_vcc_valueChanged(double value)
{
  int raw = 4096 * value / 5.0;

  pthread_mutex_lock(&mutex_json_);
  diag_json_["volt_temp"]["top"]["pwr_vccint"] = raw;
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_slider_bot_i_out_valueChanged(double value)
{
  int raw = 4096 * (value / 10.0 + 2.5) / 5.0;

  pthread_mutex_lock(&mutex_json_);
  diag_json_["volt_temp"]["bot"]["i_out"] = raw;
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_slider_bot_1_2v_valueChanged(double value)
{
  int raw = 4096 * value / 5.0;

  pthread_mutex_lock(&mutex_json_);
  diag_json_["volt_temp"]["top"]["pwr_1_2v"] = raw;
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_slider_bot_temp_valueChanged(double value)
{
  double d = roundAt(1.8639 - (std::pow(value + 1481.96, 2.0) - 2.1962e6) * 3.88e-6, 4);
  int raw = roundAt(4096 * d / 5.0, 3);

  pthread_mutex_lock(&mutex_json_);
  diag_json_["volt_temp"]["bot"]["lm20_temp"] = raw;
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_slider_bot_5v_valueChanged(double value)
{
  int raw = 4096 * value / 5.0 / 2.0;

  pthread_mutex_lock(&mutex_json_);
  diag_json_["volt_temp"]["top"]["pwr_5v"] = raw;
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_slider_bot_2_5v_valueChanged(double value)
{
  int raw = 4096 * value / 5.0;

  pthread_mutex_lock(&mutex_json_);
  diag_json_["volt_temp"]["top"]["pwr_2_5v"] = raw;
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_slider_bot_3_3v_valueChanged(double value)
{
  int raw = 4096 * value / 5.0;

  pthread_mutex_lock(&mutex_json_);
  diag_json_["volt_temp"]["top"]["pwr_3_3v"] = raw;
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_slider_bot_v_in_valueChanged(double value)
{
  int raw = 4096 * value / 5.0 / 11.0;

  pthread_mutex_lock(&mutex_json_);
  diag_json_["volt_temp"]["bot"]["pwr_v_in"] = raw;
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_slider_bot_1_25v_valueChanged(double value)
{
  int raw = 4096 * value / 5.0;

  pthread_mutex_lock(&mutex_json_);
  diag_json_["volt_temp"]["top"]["pwr_1_25v"] = raw;
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_pushButton_motor_state_toggled(bool checked)
{
  pthread_mutex_lock(&mutex_json_);
  status_json_["motor"]["state"] = json::value::string(checked ? "On" : "Off");
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_slider_motor_rpm_valueChanged(double value)
{
  pthread_mutex_lock(&mutex_json_);
  status_json_["motor"]["rpm"] = static_cast<int>(value);
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_pushButton_laser_state_toggled(bool checked)
{
  pthread_mutex_lock(&mutex_json_);
  status_json_["laser"]["state"] = json::value::string(checked ? "On" : "Off");
  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::on_slider_motor_rpm_setting_valueChanged(double value)
{
  pthread_mutex_lock(&mutex_json_);
  settings_json_["rpm"] = static_cast<int>(value);
  pthread_mutex_unlock(&mutex_json_);
}

QString FakeVelodyneWidget::get_server_address(void) { return ui->lineEdit_address->text(); }

void FakeVelodyneWidget::setAddress(const QString & device_name)
{
  ui->lineEdit_address->setText(device_name);
}

QString FakeVelodyneWidget::getAddress(void) { return ui->lineEdit_address->text(); }

int FakeVelodyneWidget::start(void)
{
  int ret = 0;
  stop_thread_ = false;
  pthread_create(&th_, nullptr, &FakeVelodyneWidget::threadHelper, this);
  th_ptr_ = &th_;
  return ret;
}

void FakeVelodyneWidget::stop(void)
{
  pthread_mutex_lock(&mutex_stop_);
  stop_thread_ = true;
  pthread_mutex_unlock(&mutex_stop_);
  if (th_ptr_ != nullptr) pthread_join(th_, NULL);
  th_ptr_ = nullptr;
}

void * FakeVelodyneWidget::thread(void)
{
  web::http::experimental::listener::http_listener listener(
    emit signal_get_server_address().toStdString());

  // Add a general handler to support all requests
  listener.support(web::http::methods::GET, boost::bind(&FakeVelodyneWidget::handleGet, this, _1));
  // Asynchronously open the listener
  listener.open().wait();

  // Handle incoming requests.
  std::cout << "Starting up HTTP server." << std::endl;

  while (true) {
    bool b;
    pthread_mutex_lock(&mutex_stop_);
    b = stop_thread_;
    pthread_mutex_unlock(&mutex_stop_);
    if (b) break;

    usleep(1000);
  }

  listener.close();

  return nullptr;
}

void FakeVelodyneWidget::handleGet(http::http_request request)
{
  // Get the underling URI of the request message
  std::string path = request.request_uri().path();

  pthread_mutex_lock(&mutex_json_);

  if (path == "/cgi/info.json") {
    request.reply(http::status_codes::OK, info_json_);
  } else if (path == "/cgi/diag.json") {
    request.reply(http::status_codes::OK, diag_json_);
  } else if (path == "/cgi/status.json") {
    request.reply(http::status_codes::OK, status_json_);
  } else if (path == "/cgi/settings.json") {
    request.reply(http::status_codes::OK, settings_json_);
  } else {
    request.reply(http::status_codes::BadRequest);
  }

  pthread_mutex_unlock(&mutex_json_);
}

void FakeVelodyneWidget::loadInfoJson(void)
{
  QFile file(":/data/info.json");
  if (file.open(QIODevice::ReadOnly | QIODevice::Text))
    info_json_ = json::value::parse(file.readAll().toStdString());

  ui->lineEdit_model->setText(info_json_["model"].as_string().c_str());
  ui->lineEdit_serial->setText(info_json_["serial"].as_string().c_str());
}

void FakeVelodyneWidget::loadDiagJson(void)
{
  QFile file(":/data/diag.json");
  if (file.open(QIODevice::ReadOnly | QIODevice::Text))
    diag_json_ = json::value::parse(file.readAll().toStdString());

  double top_hv = diag_json_["volt_temp"]["top"]["hv"].as_double();
  top_hv = roundAt((top_hv * 5.0) / 4096, 3);
  top_hv = roundAt(101.0 * (top_hv - 5.0), 2);
  ui->slider_top_hv->setValue(top_hv);

  double top_temp = diag_json_["volt_temp"]["top"]["lm20_temp"].as_double();
  top_temp = roundAt((top_temp * 5.0) / 4096, 4);
  top_temp = roundAt(-1481.96 + std::sqrt(2.1962e6 + ((1.8639 - top_temp) / 3.88e-6)), 3);
  ui->slider_top_temp->setValue(top_temp);

  double top_5v = diag_json_["volt_temp"]["top"]["pwr_5v"].as_double();
  top_5v = roundAt((top_5v * 5.0) / 4096 * 2.0, 3);
  ui->slider_top_5v->setValue(top_5v);

  double top_2_5v = diag_json_["volt_temp"]["top"]["pwr_2_5v"].as_double();
  top_2_5v = roundAt((top_2_5v * 5.0) / 4096, 3);
  ui->slider_top_2_5v->setValue(top_2_5v);

  double top_3_3v = diag_json_["volt_temp"]["top"]["pwr_3_3v"].as_double();
  top_3_3v = roundAt((top_3_3v * 5.0) / 4096, 3);
  ui->slider_top_3_3v->setValue(top_3_3v);

  double top_5v_raw = diag_json_["volt_temp"]["top"]["pwr_5v_raw"].as_double();
  top_5v_raw = roundAt((top_5v_raw * 5.0) / 4096 * 2.0, 3);
  ui->slider_top_5v_raw->setValue(top_5v_raw);

  double top_vcc = diag_json_["volt_temp"]["top"]["pwr_vccint"].as_double();
  top_vcc = roundAt((top_vcc * 5.0) / 4096, 3);
  ui->slider_top_vcc->setValue(top_vcc);

  double bot_i_out = diag_json_["volt_temp"]["bot"]["i_out"].as_double();
  bot_i_out = roundAt((bot_i_out * 5.0) / 4096, 4);
  bot_i_out = roundAt(10.0 * (bot_i_out - 2.5), 3);
  ui->slider_bot_i_out->setValue(bot_i_out);

  double bot_temp = diag_json_["volt_temp"]["bot"]["lm20_temp"].as_double();
  bot_temp = roundAt((bot_temp * 5.0) / 4096, 4);
  bot_temp = roundAt(-1481.96 + std::sqrt(2.1962e6 + ((1.8639 - bot_temp) / 3.88e-6)), 3);
  ui->slider_bot_temp->setValue(bot_temp);

  double bot_1_2v = diag_json_["volt_temp"]["bot"]["pwr_1_2v"].as_double();
  bot_1_2v = roundAt((bot_1_2v * 5.0) / 4096, 3);
  ui->slider_bot_1_2v->setValue(bot_1_2v);

  double bot_5v = diag_json_["volt_temp"]["bot"]["pwr_5v"].as_double();
  bot_5v = roundAt((bot_5v * 5.0) / 4096 * 2.0, 3);
  ui->slider_bot_5v->setValue(bot_5v);

  double bot_2_5v = diag_json_["volt_temp"]["bot"]["pwr_2_5v"].as_double();
  bot_2_5v = roundAt((bot_2_5v * 5.0) / 4096, 3);
  ui->slider_bot_2_5v->setValue(bot_2_5v);

  double bot_3_3v = diag_json_["volt_temp"]["bot"]["pwr_3_3v"].as_double();
  bot_3_3v = roundAt((bot_3_3v * 5.0) / 4096, 3);
  ui->slider_bot_3_3v->setValue(bot_3_3v);

  double bot_v_in = diag_json_["volt_temp"]["bot"]["pwr_v_in"].as_double();
  bot_v_in = roundAt((bot_v_in * 5.0) / 4096 * 11.0, 3);
  ui->slider_bot_v_in->setValue(bot_v_in);

  double bot_1_25v = diag_json_["volt_temp"]["bot"]["pwr_1_25v"].as_double();
  bot_1_25v = roundAt((bot_1_25v * 5.0) / 4096, 3);
  ui->slider_bot_1_25v->setValue(bot_1_25v);
}

void FakeVelodyneWidget::loadStatusJson(void)
{
  QFile file(":/data/status.json");
  if (file.open(QIODevice::ReadOnly | QIODevice::Text))
    status_json_ = json::value::parse(file.readAll().toStdString());

  std::string state = status_json_["motor"]["state"].as_string();
  ui->pushButton_motor_state->setChecked(state == "On");

  int motor_rpm = status_json_["motor"]["rpm"].as_integer();
  ui->slider_motor_rpm->setValue(motor_rpm);

  state = status_json_["laser"]["state"].as_string();
  ui->pushButton_laser_state->setChecked(state == "On");
}

void FakeVelodyneWidget::loadSettingsJson(void)
{
  QFile file(":/data/settings.json");
  if (file.open(QIODevice::ReadOnly | QIODevice::Text))
    settings_json_ = json::value::parse(file.readAll().toStdString());

  ui->slider_motor_rpm_setting->setValue(settings_json_["rpm"].as_integer());
}

double FakeVelodyneWidget::roundAt(double x, int k)
{
  int ix;
  x = x * pow(10, k);
  ix = std::nearbyint(x);
  x = static_cast<double>(ix) / pow(10, k);
  return x;
}
