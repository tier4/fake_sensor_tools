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
 * @file rqt_fake_gnss_widget.cpp
 * @brief RQt plugin widget class
 */

#include <boost/thread.hpp>
#include <iostream>

#include <rqt_fake_gnss/rqt_fake_gnss_widget.h>
#include <ui_rqt_fake_gnss_widget.h>

static constexpr int MAX_SIZE = 1024;
static constexpr int SLEEP_CNT_1MS = 1000;
static constexpr int ADJUST_COUNT = 130;

std::map<UBX_ID, FakeGnssWidget::HANDLE_FUNC> FakeGnssWidget::handle_map_ = {
  {{0x0A, 0x04}, &FakeGnssWidget::handleUbxMonVER},
  {{0x06, 0x00}, &FakeGnssWidget::handleUbxCfgPRT},
  {{0x06, 0x01}, &FakeGnssWidget::handleUbxCfgMSG},
};

std::map<UBX_ID, FakeGnssWidget::PERIODIC_TRANSMIT> FakeGnssWidget::periodic_map_ = {
  {{0x01, 0x03}, {&FakeGnssWidget::sendUbxNavSTATUS, 0, 0}},
  {{0x01, 0x07}, {&FakeGnssWidget::sendUbxNavPVT, 0, 0}},
  {{0x01, 0x3C}, {&FakeGnssWidget::sendUbxNavRELPOSNED, 0, 0}},
  {{0x0A, 0x09}, {&FakeGnssWidget::sendUbxMonHW, 0, 0}},
  {{0x0A, 0x36}, {&FakeGnssWidget::sendUbxMonCOMMS, 0, 0}},
};

std::map<std::string, PortBlock> FakeGnssWidget::port_blocks_ = {
  {"I2C", {false, 0}}, {"UART1", {false, 0}}, {"UART2", {false, 0}},
  {"USB", {true, 0}},  {"SPI", {false, 0}},
};

FakeGnssWidget::FakeGnssWidget(QWidget * parent)
: QWidget(parent),
  ui(new Ui::FakeGnssWidget),
  mutex_stop_(),
  mutex_port_(),
  stop_thread_(false),
  th_ptr_(nullptr)
{
  ui->setupUi(this);

  ui->comboBox_a_status->addItem("INIT");
  ui->comboBox_a_status->addItem("DONTKNOW");
  ui->comboBox_a_status->addItem("OK");
  ui->comboBox_a_status->addItem("SHORT");
  ui->comboBox_a_status->addItem("OPEN");

  ui->comboBox_jamming_state->addItem("Unknown");
  ui->comboBox_jamming_state->addItem("OK");
  ui->comboBox_jamming_state->addItem("Warning");
  ui->comboBox_jamming_state->addItem("Critical");

  ui->comboBox_port_id->addItem("I2C");
  ui->comboBox_port_id->addItem("UART1");
  ui->comboBox_port_id->addItem("UART2");
  ui->comboBox_port_id->addItem("USB");
  ui->comboBox_port_id->addItem("SPI");

  ui->comboBox_spoof_det_state->addItem("Unknown");
  ui->comboBox_spoof_det_state->addItem("No spoofing");
  ui->comboBox_spoof_det_state->addItem("Spoofing");
  ui->comboBox_spoof_det_state->addItem("Multiple spoofing");

  connect(
    this, SIGNAL(signal_get_checksum_error()), this, SLOT(get_checksum_error()),
    Qt::BlockingQueuedConnection);
  connect(
    this, SIGNAL(signal_get_debug_output()), this, SLOT(get_debug_output()),
    Qt::BlockingQueuedConnection);
  connect(
    this, SIGNAL(signal_get_a_status()), this, SLOT(get_a_status()), Qt::BlockingQueuedConnection);
  connect(
    this, SIGNAL(signal_get_jamming_state()), this, SLOT(get_jamming_state()),
    Qt::BlockingQueuedConnection);
  connect(
    this, SIGNAL(signal_get_spoof_det_state()), this, SLOT(get_spoof_det_state()),
    Qt::BlockingQueuedConnection);
}

FakeGnssWidget::~FakeGnssWidget() { delete ui; }

void FakeGnssWidget::on_pushButton_serial_port_toggled(bool checked)
{
  if (checked) {
    // Start serial port communication
    start();
  } else {
    // Stop serial port communication
    stop();
  }
}

void FakeGnssWidget::on_comboBox_port_id_currentIndexChanged(const QString & arg1)
{
  ui->pushButton_enabled->setChecked(port_blocks_[arg1.toStdString()].port_enabled);
  ui->slider_tx_usage->setValue(port_blocks_[arg1.toStdString()].tx_usage);
}

void FakeGnssWidget::on_pushButton_enabled_toggled(bool checked)
{
  pthread_mutex_lock(&mutex_port_);
  port_blocks_[ui->comboBox_port_id->currentText().toStdString()].port_enabled = checked;
  pthread_mutex_unlock(&mutex_port_);
}

void FakeGnssWidget::on_slider_tx_usage_valueChanged(double value)
{
  pthread_mutex_lock(&mutex_port_);
  port_blocks_[ui->comboBox_port_id->currentText().toStdString()].tx_usage = value;
  pthread_mutex_unlock(&mutex_port_);
}

bool FakeGnssWidget::get_checksum_error(void) { return ui->pushButton_checksum_error->isChecked(); }

bool FakeGnssWidget::get_debug_output(void) { return ui->pushButton_debug_output->isChecked(); }

int FakeGnssWidget::get_a_status() { return ui->comboBox_a_status->currentIndex(); }

int FakeGnssWidget::get_jamming_state() { return ui->comboBox_jamming_state->currentIndex(); }

int FakeGnssWidget::get_spoof_det_state(void) { ui->comboBox_spoof_det_state->currentIndex(); }

void FakeGnssWidget::setDeviceName(const QString & device_name)
{
  ui->lineEdit_device_name->setText(device_name);
}

QString FakeGnssWidget::getDeviceName(void) { return ui->lineEdit_device_name->text(); }

int FakeGnssWidget::start(void)
{
  int ret = 0;

  // Preparation for a subsequent run() invocation
  io_.reset();
  port_ = boost::shared_ptr<as::serial_port>(new as::serial_port(io_));

  // Open the serial port using the specified device name
  try {
    port_->open(ui->lineEdit_device_name->text().toStdString());
  } catch (const boost::system::system_error & e) {
    ret = ENOENT;
    std::cerr << e.what() << std::endl;
    return ret;
  }

  stop_thread_ = false;
  pthread_create(&th_, nullptr, &FakeGnssWidget::threadHelper, this);
  th_ptr_ = &th_;
  return ret;
}

void FakeGnssWidget::stop(void)
{
  pthread_mutex_lock(&mutex_stop_);
  stop_thread_ = true;
  pthread_mutex_unlock(&mutex_stop_);
  if (th_ptr_ != nullptr) pthread_join(th_, NULL);
  th_ptr_ = nullptr;
  io_.stop();
}

void * FakeGnssWidget::thread(void)
{
  boost::thread thr_io(boost::bind(&as::io_service::run, &io_));

  // asynchronously read data
  uint8_t data[MAX_SIZE] = "";
  port_->async_read_some(
    as::buffer(data), boost::bind(
                        &FakeGnssWidget::onRead, this, as::placeholders::error,
                        as::placeholders::bytes_transferred, data));
  while (true) {
    bool b;
    pthread_mutex_lock(&mutex_stop_);
    b = stop_thread_;
    pthread_mutex_unlock(&mutex_stop_);
    if (b) break;
    // Handle periodic transmission
    handlePeriodicTransmit();
    usleep(SLEEP_CNT_1MS);
  }

  return nullptr;
}

void FakeGnssWidget::handlePeriodicTransmit(void)
{
  for (auto & p : periodic_map_) {
    if (p.second.rate_ > 0) {
      --p.second.cnt_;
      if (p.second.cnt_ <= 0) {
        (this->*(p.second.func_))();
        p.second.cnt_ = p.second.rate_;
      }
    }
  }
}

void FakeGnssWidget::dump(Direction dir, const uint8_t * data, std::size_t size)
{
  printf("%s ", (dir == Read) ? ">" : "<");

  for (std::size_t i = 0; i < size; ++i) {
    printf("%02X", data[i]);
    if (i + 1 <= size) printf(" ");
  }
  printf("\n");
}

void FakeGnssWidget::onRead(
  const boost::system::error_code & error, std::size_t bytes_transfered, const uint8_t * data)
{
  if (error) {
    std::cout << error.message() << std::endl;
  } else {
    if (emit signal_get_debug_output()) {
      dump(Read, data, bytes_transfered);
    }

    if (data[0] == 0xB5 && data[1] == 0x62) {
      handleUbx(data);
    }
    // asynchronously read data
    uint8_t next[MAX_SIZE] = "";
    port_->async_read_some(
      as::buffer(next), boost::bind(
                          &FakeGnssWidget::onRead, this, as::placeholders::error,
                          as::placeholders::bytes_transferred, next));
  }
}

void FakeGnssWidget::onWrite(
  const boost::system::error_code & error, std::size_t bytes_transfered,
  const std::vector<uint8_t> & data)
{
  if (emit signal_get_debug_output()) {
    dump(Write, &data[0], bytes_transfered);
  }
}

void FakeGnssWidget::handleUbx(const uint8_t * data)
{
  auto it = handle_map_.find(UBX_ID(data[2], data[3]));

  if (it != handle_map_.end()) {
    (this->*(it->second))(data);
  } else {
    // UBX-CFG-???
    if (data[2] == 0x06) {
      sendUbxAck(true, data[2], data[3]);
    }
  }
}

// UBX-MON-VER
void FakeGnssWidget::handleUbxMonVER(const uint8_t * data)
{
  uint8_t d[] = {
    0xB5, 0x62, 0x0A, 0x04, 0xDC, 0x00, 0x45, 0x58, 0x54, 0x20, 0x43, 0x4F, 0x52, 0x45, 0x20, 0x31,
    0x2E, 0x30, 0x30, 0x20, 0x28, 0x36, 0x31, 0x62, 0x32, 0x64, 0x64, 0x29, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x31, 0x39, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00, 0x52, 0x4F,
    0x4D, 0x20, 0x42, 0x41, 0x53, 0x45, 0x20, 0x30, 0x78, 0x31, 0x31, 0x38, 0x42, 0x32, 0x30, 0x36,
    0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46, 0x57, 0x56, 0x45,
    0x52, 0x3D, 0x48, 0x50, 0x47, 0x20, 0x31, 0x2E, 0x31, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x52, 0x4F, 0x54, 0x56, 0x45,
    0x52, 0x3D, 0x32, 0x37, 0x2E, 0x31, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4D, 0x4F, 0x44, 0x3D, 0x5A, 0x45, 0x44, 0x2D,
    0x46, 0x39, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x50, 0x53, 0x3B, 0x47, 0x4C, 0x4F, 0x3B, 0x47, 0x41,
    0x4C, 0x3B, 0x42, 0x44, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x51, 0x5A, 0x53, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xBD, 0x36};

  send(d, sizeof(d));
}

void FakeGnssWidget::handleUbxCfgPRT(const uint8_t * data)
{
  uint8_t d[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
                 0xC0, 0x08, 0x00, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x07, 0x00,
                 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xC3};

  send(d, sizeof(d));
}

void FakeGnssWidget::handleUbxCfgMSG(const uint8_t * data)
{
  auto it = periodic_map_.find(UBX_ID(data[6], data[7]));
  bool f = false;

  if (it != periodic_map_.end()) {
    it->second.rate_ = (SLEEP_CNT_1MS / data[8]) - ADJUST_COUNT;
    it->second.cnt_ = it->second.rate_;
    f = true;
  }

  sendUbxAck(f, data[2], data[3]);
}

void FakeGnssWidget::sendUbxAck(bool ack, uint8_t message_class, uint8_t message_id)
{
  uint8_t data[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
  data[3] = ack ? 0x01 : 0x00;
  data[6] = message_class;
  data[7] = message_id;

  send(data, sizeof(data));
}

void FakeGnssWidget::sendUbxNavSTATUS(void)
{
  uint8_t d[] = {0xB5, 0x62, 0x01, 0x03, 0x10, 0x00, 0x38, 0x4A, 0x79, 0x17, 0x03, 0xDD,
                 0x00, 0x00, 0xC4, 0x05, 0x00, 0x00, 0xE6, 0x1C, 0x09, 0x00, 0xDA, 0xF0};

  d[13] = ui->comboBox_spoof_det_state->currentIndex() << 3;
  send(d, sizeof(d));
}

void FakeGnssWidget::sendUbxNavPVT(void)
{
  uint8_t d[] = {0xB5, 0x62, 0x01, 0x07, 0x5C, 0x00, 0x50, 0x32, 0x20, 0x17, 0xE3, 0x07, 0x0A,
                 0x11, 0x0B, 0x2E, 0x08, 0x37, 0x0F, 0x00, 0x00, 0x00, 0x2E, 0x3A, 0x01, 0x00,
                 0x03, 0x01, 0xEA, 0x0F, 0x46, 0x4E, 0x44, 0x04, 0xFE, 0x81, 0x90, 0x1E, 0x03,
                 0x9A, 0x03, 0x00, 0x46, 0xE4, 0x02, 0x00, 0xD3, 0x10, 0x00, 0x00, 0xB9, 0x1B,
                 0x00, 0x00, 0x89, 0xFF, 0xFF, 0xFF, 0xE5, 0xFF, 0xFF, 0xFF, 0x7C, 0xFF, 0xFF,
                 0xFF, 0x7A, 0x00, 0x00, 0x00, 0x63, 0xFD, 0xC9, 0x00, 0xFF, 0x01, 0x00, 0x00,
                 0x1C, 0x0D, 0x4B, 0x00, 0x8D, 0x00, 0x00, 0x00, 0xB8, 0x41, 0x47, 0x3D, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9D, 0x7C};

  time_t nowt = time(nullptr);
  struct tm now;
  gmtime_r(&nowt, &now);
  uint16_t * year = reinterpret_cast<uint16_t *>(&d[10]);
  *year = now.tm_year + 1900;
  d[12] = now.tm_mon + 1;
  d[13] = now.tm_mday;
  d[14] = now.tm_hour;
  d[15] = now.tm_min;
  d[16] = now.tm_sec;

  send(d, sizeof(d));
}

void FakeGnssWidget::sendUbxNavRELPOSNED(void)
{
  uint8_t d[] = {0xB5, 0x62, 0x01, 0x3C, 0x40, 0x00, 0x01, 0x00, 0x00, 0x00, 0xB0, 0xAF,
                 0x47, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x3C, 0x86};

  send(d, sizeof(d));
}

void FakeGnssWidget::sendUbxMonHW(void)
{
  uint8_t d[] = {0xB5, 0x62, 0x0A, 0x09, 0x3C, 0x00, 0x00, 0xC4, 0x01, 0x00, 0x00, 0x28, 0x00, 0x00,
                 0x00, 0x00, 0x01, 0x00, 0xEF, 0xC7, 0x01, 0x00, 0x78, 0x00, 0x5C, 0x0A, 0x02, 0x01,
                 0x00, 0x84, 0xFF, 0x7B, 0x01, 0x00, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x01, 0x00,
                 0x02, 0x03, 0xFF, 0x05, 0x11, 0x04, 0x13, 0xFF, 0x35, 0x08, 0x0F, 0x5E, 0x00, 0x00,
                 0x00, 0x00, 0x80, 0xEF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6A, 0xEB};

  d[26] = emit signal_get_a_status();
  d[28] = emit signal_get_jamming_state() << 2;
  send(d, sizeof(d));
}

void FakeGnssWidget::sendUbxMonCOMMS(void)
{
  std::vector<uint8_t> d{0xB5, 0x62, 0x0A, 0x36, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  uint16_t id = 0;

  pthread_mutex_lock(&mutex_port_);
  for (const auto & p : port_blocks_) {
    uint8_t b[40] = {};
    if (p.second.port_enabled) {
      // nPorts
      ++d[7];
      uint16_t * portId = reinterpret_cast<uint16_t *>(&b[0]);
      *portId = id;
      b[8] = p.second.tx_usage;
      d.insert(d.end(), &b[0], &b[sizeof(b)]);
    }
    ++id;
  }
  pthread_mutex_unlock(&mutex_port_);

  uint16_t * length = reinterpret_cast<uint16_t *>(&d[4]);
  *length = d.size() - 6;
  d.resize(d.size() + 2);
  send(&d[0], d.size());
}

void FakeGnssWidget::send(uint8_t * data, int size)
{
  calculateChecksum(&data[2], size - 4, data[size - 2], data[size - 1]);
  std::vector<uint8_t> frame(data, data + size);

  if (emit signal_get_checksum_error()) {
    frame[frame.size() - 1] = '?';
    frame[frame.size() - 2] = '?';
  }

  // asynchronously write data
  port_->async_write_some(
    as::buffer(frame), boost::bind(
                         &FakeGnssWidget::onWrite, this, as::placeholders::error,
                         as::placeholders::bytes_transferred, frame));
}

void FakeGnssWidget::calculateChecksum(
  const uint8_t * data, int size, uint8_t & ck_a, uint8_t & ck_b)
{
  ck_a = 0;
  ck_b = 0;
  for (int i = 0; i < size; ++i) {
    ck_a = ck_a + data[i];
    ck_b = ck_b + ck_a;
  }
}
