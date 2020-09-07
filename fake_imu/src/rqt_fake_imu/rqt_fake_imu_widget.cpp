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
 * @file rqt_fake_imu_widget.cpp
 * @brief RQt plugin widget class
 */

#include <libgen.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <iostream>

#include <rqt_fake_imu/rqt_fake_imu_widget.h>
#include <ui_rqt_fake_imu_widget.h>
#include <QDirIterator>
#include <QFileInfo>

namespace fs = boost::filesystem;

static constexpr int MAX_SIZE = 1024;
static constexpr int MAX_BIN_SIZE = 58;

FakeImuWidget::FakeImuWidget(QWidget * parent)
: QWidget(parent), ui(new Ui::FakeImuWidget), mutex_stop_(), stop_thread_(false), bin_req_(false), th_ptr_(nullptr)
{
  ui->setupUi(this);

  QDirIterator it(":/data", QDirIterator::Subdirectories);
  while (it.hasNext()) {
    QFile file(it.next());
    QFileInfo info(file);
    if (info.completeSuffix() == "bin") {
      ui->comboBox_log_file->insertItem(0, info.fileName());
    }
  }
  ui->comboBox_log_file->setCurrentIndex(0);

  connect(this, SIGNAL(signal_get_checksum_error()), this, SLOT(get_checksum_error()), Qt::BlockingQueuedConnection);
  connect(this, SIGNAL(signal_get_debug_output()), this, SLOT(get_debug_output()), Qt::BlockingQueuedConnection);
}

FakeImuWidget::~FakeImuWidget() { delete ui; }

void FakeImuWidget::on_pushButton_serial_port_toggled(bool checked)
{
  if (checked) {
    // Start serial port communication
    start();
  } else {
    // Stop serial port communication
    stop();
  }
}

bool FakeImuWidget::get_checksum_error(void) { return ui->pushButton_checksum_error->isChecked(); }

bool FakeImuWidget::get_debug_output(void) { return ui->pushButton_debug_output->isChecked(); }

void FakeImuWidget::setDeviceName(const QString & device_name) { ui->lineEdit_device_name->setText(device_name); }

QString FakeImuWidget::getDeviceName(void) { return ui->lineEdit_device_name->text(); }

int FakeImuWidget::start(void)
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

  bin_req_ = false;
  stop_thread_ = false;
  pthread_create(&th_, nullptr, &FakeImuWidget::threadHelper, this);
  th_ptr_ = &th_;
  return ret;
}

void FakeImuWidget::stop()
{
  pthread_mutex_lock(&mutex_stop_);
  stop_thread_ = true;
  pthread_mutex_unlock(&mutex_stop_);
  if (th_ptr_ != nullptr) pthread_join(th_, NULL);
  th_ptr_ = nullptr;
  port_->close();
  io_.stop();
}

void * FakeImuWidget::thread(void)
{
  boost::thread thr_io(boost::bind(&as::io_service::run, &io_));

  // asynchronously read data
  as::async_read_until(
    *port_, buffer_, "\n",
    boost::bind(&FakeImuWidget::onRead, this, as::placeholders::error, as::placeholders::bytes_transferred));

  QFile file(":/data/" + ui->comboBox_log_file->currentText());
  if (!file.open(QIODevice::ReadOnly)) {
    return nullptr;
  }

  while (true) {
    bool b;
    pthread_mutex_lock(&mutex_stop_);
    b = stop_thread_;
    pthread_mutex_unlock(&mutex_stop_);
    if (b) break;

    if (bin_req_) {
      uint8_t data[MAX_BIN_SIZE] = {};
      int len = sizeof(data);
      file.read(reinterpret_cast<char *>(data), len);
      if (file.atEnd()) {
        file.seek(0);
        file.read(reinterpret_cast<char *>(data), len);
      }

      if (emit signal_get_checksum_error()) {
        data[len - 3] = '?';
        data[len - 4] = '?';
      }

      std::vector<uint8_t> frame(data, data + len);
      // asynchronously write data
      port_->async_write_some(
        as::buffer(frame),
        boost::bind(
          &FakeImuWidget::onWrite, this, as::placeholders::error, as::placeholders::bytes_transferred, frame));
    }

    // 30Hz
    usleep(1000000 / 30);
  }

  file.close();

  return nullptr;
}

void FakeImuWidget::dump(Direction dir, const std::string & data, std::size_t size)
{
  printf("%s ", (dir == Read) ? ">" : "<");

  for (std::size_t i = 0; i < size; ++i) {
    printf("%02X", data[i]);
    if (i + 1 <= size) printf(" ");
  }
  printf("\n");
}

void FakeImuWidget::dumpBIN(const uint8_t * data)
{
  printf("< %c%c%c%c%c%c%c%c%c", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
  printf("%02X%02X", data[9], data[10]);
  printf(" %02X%02X", data[11], data[12]);
  printf(" %02X%02X", data[13], data[14]);
  printf(" %02X%02X%02X%02X%02X%02X", data[15], data[16], data[17], data[18], data[19], data[20]);
  printf(" %02X%02X%02X%02X%02X%02X", data[21], data[22], data[23], data[24], data[25], data[26]);
  printf(" %02X%02X%02X%02X%02X%02X", data[27], data[28], data[29], data[30], data[31], data[32]);
  printf(" %02X%02X%02X%02X", data[33], data[34], data[35], data[36]);
  printf(
    " %02X%02X%02X%02X%02X%02X%02X%02X", data[37], data[38], data[39], data[40], data[41], data[42], data[43],
    data[44]);
  printf(" %02X%02X%02X%02X%02X%02X", data[45], data[46], data[47], data[48], data[49], data[50]);
  printf(" %02X%02X", data[51], data[52]);
  printf("%c%c%c%c%c", data[53], data[54], data[55], data[56], data[57]);
}

void FakeImuWidget::onRead(const boost::system::error_code & error, std::size_t bytes_transfered)
{
  if (error) {
    std::cout << error.message() << std::endl;
  } else {
    std::ostringstream oss;
    oss << &buffer_;
    const std::string data = oss.str();

    if (emit signal_get_debug_output()) {
      dump(Read, data, bytes_transfered);
    }

    if (boost::starts_with(data, "$TSC,BIN,")) {
      bin_req_ = true;
    }

    // asynchronously read data
    as::async_read_until(
      *port_, buffer_, "\n",
      boost::bind(&FakeImuWidget::onRead, this, as::placeholders::error, as::placeholders::bytes_transferred));
  }
}

void FakeImuWidget::onWrite(
  const boost::system::error_code & error, std::size_t bytes_transfered, const std::vector<uint8_t> & data)
{
  if (emit signal_get_debug_output()) {
    dumpBIN(&data[0]);
  }
}
