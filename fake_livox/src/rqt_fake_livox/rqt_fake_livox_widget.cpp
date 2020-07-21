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
 * @file rqt_fake_livox_widget.cpp
 * @brief RQt plugin widget class
 */

#include <boost/thread.hpp>
#include <iostream>

#include <rqt_fake_livox/rqt_fake_livox_widget.h>
#include <ui_rqt_fake_livox_widget.h>

std::map<CMD_SET_ID, FakeLivoxWidget::HANDLE_FUNC> FakeLivoxWidget::handle_map_ = {
  {{kCommandSetGeneral, kCommandIDGeneralHandshake}, &FakeLivoxWidget::handleGeneralHandshake},
  {{kCommandSetGeneral, kCommandIDGeneralDeviceInfo}, &FakeLivoxWidget::handleUGeneralDeviceInfo},
  {{kCommandSetGeneral, kCommandIDGeneralHeartbeat}, &FakeLivoxWidget::handleGeneralHeartbeat},
  {{kCommandSetGeneral, kCommandIDGeneralControlSample}, &FakeLivoxWidget::handleGeneralControlSample},
  {{kCommandSetGeneral, kCommandIDGeneralCoordinateSystem}, &FakeLivoxWidget::handleGeneralCoordinateSystem},
  {{kCommandSetLidar, kCommandIDLidarSetPointCloudReturnMode}, &FakeLivoxWidget::handleLidarSetPointCloudReturnMode},
  {{kCommandSetLidar, kCommandIDLidarSetImuPushFrequency}, &FakeLivoxWidget::handleLidarSetImuPushFrequency},
};

FakeLivoxWidget::FakeLivoxWidget(QWidget * parent)
: QWidget(parent),
  ui(new Ui::FakeLivoxWidget),
  mutex_stop_(),
  stop_thread_(false),
  th_ptr_(nullptr),
  crc16_(0x4c49),
  crc32_(0x564f580a)
{
  ui->setupUi(this);

  ui->comboBox_lidar_state->addItem("Initializing");
  ui->comboBox_lidar_state->addItem("Normal");
  ui->comboBox_lidar_state->addItem("Power-Saving");
  ui->comboBox_lidar_state->addItem("Standby");
  ui->comboBox_lidar_state->addItem("Error");
  ui->comboBox_lidar_state->setCurrentIndex(1);

  connect(this, SIGNAL(signal_get_broadcast_code()), this, SLOT(get_broadcast_code()), Qt::BlockingQueuedConnection);
  connect(this, SIGNAL(signal_get_checksum_error()), this, SLOT(get_checksum_error()), Qt::BlockingQueuedConnection);
  connect(this, SIGNAL(signal_get_debug_output()), this, SLOT(get_debug_output()), Qt::BlockingQueuedConnection);

  connect(this, SIGNAL(signal_get_return_code()), this, SLOT(get_return_code()), Qt::BlockingQueuedConnection);
  connect(this, SIGNAL(signal_get_lidar_state()), this, SLOT(get_lidar_state()), Qt::BlockingQueuedConnection);
  connect(
    this, SIGNAL(signal_get_rain_fog_suppression_switch()), this, SLOT(get_rain_fog_suppression_switch()),
    Qt::BlockingQueuedConnection);
  connect(
    this, SIGNAL(signal_get_initialization_progress()), this, SLOT(get_initialization_progress()),
    Qt::BlockingQueuedConnection);
  connect(this, SIGNAL(signal_get_temp_status()), this, SLOT(get_temp_status()), Qt::BlockingQueuedConnection);
  connect(this, SIGNAL(signal_get_volt_status()), this, SLOT(get_volt_status()), Qt::BlockingQueuedConnection);
  connect(this, SIGNAL(signal_get_motor_status()), this, SLOT(get_motor_status()), Qt::BlockingQueuedConnection);
  connect(this, SIGNAL(signal_get_dirty_warn()), this, SLOT(get_dirty_warn()), Qt::BlockingQueuedConnection);
  connect(this, SIGNAL(signal_get_firmware_status()), this, SLOT(get_firmware_status()), Qt::BlockingQueuedConnection);
  connect(this, SIGNAL(signal_get_pps_status()), this, SLOT(get_pps_status()), Qt::BlockingQueuedConnection);
  connect(this, SIGNAL(signal_get_device_status()), this, SLOT(get_device_status()), Qt::BlockingQueuedConnection);
  connect(this, SIGNAL(signal_get_fan_status()), this, SLOT(get_fan_status()), Qt::BlockingQueuedConnection);
  connect(this, SIGNAL(signal_get_self_heating()), this, SLOT(get_self_heating()), Qt::BlockingQueuedConnection);
  connect(this, SIGNAL(signal_get_ptp_status()), this, SLOT(get_ptp_status()), Qt::BlockingQueuedConnection);
  connect(
    this, SIGNAL(signal_get_time_sync_status()), this, SLOT(get_time_sync_status()), Qt::BlockingQueuedConnection);
  connect(this, SIGNAL(signal_get_system_status()), this, SLOT(get_system_status()), Qt::BlockingQueuedConnection);
}

FakeLivoxWidget::~FakeLivoxWidget() { delete ui; }

void FakeLivoxWidget::setBroadcastCode(const QString & broadcast_code)
{
  ui->lineEdit_broadcast_code->setText(broadcast_code);
}

QString FakeLivoxWidget::getBroadcastCode(void) { return ui->lineEdit_broadcast_code->text(); }

void FakeLivoxWidget::on_pushButton_comm_toggled(bool checked)
{
  if (checked) {
    // Start UDP communication
    start();
  } else {
    // Stop UDP communication
    stop();
  }
}

void FakeLivoxWidget::on_comboBox_lidar_state_currentIndexChanged(int index)
{
  int page = (0 == index) ? 0 : 1;
  ui->stackedWidget_1->setCurrentIndex(page);
}

void FakeLivoxWidget::on_radioButton_temp_status_0_toggled(bool checked)
{
  if (checked) status_codes_["temp_status"] = 0;
  updateSystemStatus();
}

void FakeLivoxWidget::on_radioButton_temp_status_1_toggled(bool checked)
{
  if (checked) status_codes_["temp_status"] = 1;
  updateSystemStatus();
}

void FakeLivoxWidget::on_radioButton_temp_status_2_toggled(bool checked)
{
  if (checked) status_codes_["temp_status"] = 2;
  updateSystemStatus();
}

void FakeLivoxWidget::on_radioButton_volt_status_0_toggled(bool checked)
{
  if (checked) status_codes_["volt_status"] = 0;
  updateSystemStatus();
}

void FakeLivoxWidget::on_radioButton_volt_status_1_toggled(bool checked)
{
  if (checked) status_codes_["volt_status"] = 1;
  updateSystemStatus();
}

void FakeLivoxWidget::on_radioButton_volt_status_2_toggled(bool checked)
{
  if (checked) status_codes_["volt_status"] = 2;
  updateSystemStatus();
}

void FakeLivoxWidget::on_radioButton_motor_status_0_toggled(bool checked)
{
  if (checked) status_codes_["motor_status"] = 0;
  updateSystemStatus();
}

void FakeLivoxWidget::on_radioButton_motor_status_1_toggled(bool checked)
{
  if (checked) status_codes_["motor_status"] = 1;
  updateSystemStatus();
}

void FakeLivoxWidget::on_radioButton_motor_status_2_toggled(bool checked)
{
  if (checked) status_codes_["motor_status"] = 2;
  updateSystemStatus();
}

void FakeLivoxWidget::on_radioButton_dirty_warn_0_toggled(bool checked)
{
  if (checked) status_codes_["dirty_warn"] = 0;
  updateSystemStatus();
}

void FakeLivoxWidget::on_radioButton_dirty_warn_1_toggled(bool checked)
{
  if (checked) status_codes_["dirty_warn"] = 1;
  updateSystemStatus();
}

void FakeLivoxWidget::on_radioButton_firmware_status_0_toggled(bool checked)
{
  if (checked) status_codes_["firmware_status"] = 0;
  updateSystemStatus();
}

void FakeLivoxWidget::on_radioButton_firmware_status_1_toggled(bool checked)
{
  if (checked) status_codes_["firmware_status"] = 2;
  updateSystemStatus();
}

void FakeLivoxWidget::on_radioButton_device_status_0_toggled(bool checked)
{
  if (checked) status_codes_["device_status"] = 0;
  updateSystemStatus();
}

void FakeLivoxWidget::on_radioButton_device_status_1_toggled(bool checked)
{
  if (checked) status_codes_["device_status"] = 1;
  updateSystemStatus();
}

void FakeLivoxWidget::on_radioButton_fan_status_0_toggled(bool checked)
{
  if (checked) status_codes_["fan_status"] = 0;
  updateSystemStatus();
}

void FakeLivoxWidget::on_radioButton_fan_status_1_toggled(bool checked)
{
  if (checked) status_codes_["fan_status"] = 1;
  updateSystemStatus();
}

QString FakeLivoxWidget::get_broadcast_code(void) { return ui->lineEdit_broadcast_code->text(); }

bool FakeLivoxWidget::get_checksum_error(void) { return ui->pushButton_checksum_error->isChecked(); }

bool FakeLivoxWidget::get_debug_output(void) { return ui->pushButton_debug_output->isChecked(); }

int FakeLivoxWidget::get_return_code(void) { return (ui->radioButton_return_code_0->isChecked()) ? 0 : 1; }

int FakeLivoxWidget::get_lidar_state(void) { return ui->comboBox_lidar_state->currentIndex(); }

int FakeLivoxWidget::get_rain_fog_suppression_switch(void)
{
  return ui->pushButton_rain_fog_suppression_switch->isChecked();
}
int FakeLivoxWidget::get_initialization_progress(void) { return ui->slider_initialization_progress->value(); }

int FakeLivoxWidget::get_temp_status(void)
{
  return (ui->radioButton_temp_status_0->isChecked()) ? 0 : (ui->radioButton_temp_status_1->isChecked()) ? 1 : 2;
}

int FakeLivoxWidget::get_volt_status(void)
{
  return (ui->radioButton_volt_status_0->isChecked()) ? 0 : (ui->radioButton_volt_status_1->isChecked()) ? 1 : 2;
}

int FakeLivoxWidget::get_motor_status(void)
{
  return (ui->radioButton_motor_status_0->isChecked()) ? 0 : (ui->radioButton_motor_status_1->isChecked()) ? 1 : 2;
}

int FakeLivoxWidget::get_dirty_warn(void) { return (ui->radioButton_dirty_warn_0->isChecked()) ? 0 : 1; }

int FakeLivoxWidget::get_firmware_status(void) { return (ui->radioButton_firmware_status_0->isChecked()) ? 0 : 1; }

int FakeLivoxWidget::get_pps_status(void) { return (ui->radioButton_pps_status_0->isChecked()) ? 0 : 1; }

int FakeLivoxWidget::get_device_status(void) { return (ui->radioButton_device_status_0->isChecked()) ? 0 : 1; }

int FakeLivoxWidget::get_fan_status(void) { return (ui->radioButton_fan_status_0->isChecked()) ? 0 : 1; }

int FakeLivoxWidget::get_self_heating(void) { return (ui->radioButton_self_heating_0->isChecked()) ? 0 : 1; }

int FakeLivoxWidget::get_ptp_status(void) { return (ui->radioButton_ptp_status_0->isChecked()) ? 0 : 1; }

int FakeLivoxWidget::get_time_sync_status(void)
{
  return (ui->radioButton_time_sync_status_0->isChecked())
           ? 0
           : (ui->radioButton_time_sync_status_1->isChecked())
               ? 1
               : (ui->radioButton_time_sync_status_2->isChecked())
                   ? 2
                   : (ui->radioButton_time_sync_status_3->isChecked()) ? 3 : 4;
}

int FakeLivoxWidget::get_system_status(void)
{
  return (ui->radioButton_system_status_0->isChecked()) ? 0 : (ui->radioButton_system_status_1->isChecked()) ? 1 : 2;
}

int FakeLivoxWidget::start(void)
{
  int ret = 0;

  // Preparation for a subsequent run() invocation
  io_.reset();
  socket_ = boost::shared_ptr<ip::udp::socket>(new ip::udp::socket(io_));

  // Open the socket using the specified protocol
  try {
    socket_->open(ip::udp::v4());
    // Allow the socket to be bound to an address that is already in use
    socket_->set_option(ip::udp::socket::reuse_address(true));
    // Permit sending of broadcast messages
    socket_->set_option(as::socket_base::broadcast(true));
    ip::udp::endpoint ep = ip::udp::endpoint(ip::address_v4::any(), 65000);
    socket_->bind(ep);
  } catch (const boost::system::system_error & e) {
    ret = ENOENT;
    std::cerr << e.what() << std::endl;
    return ret;
  }

  stop_thread_ = false;
  pthread_create(&th_, nullptr, &FakeLivoxWidget::threadHelper, this);
  th_ptr_ = &th_;
  return ret;
}

void FakeLivoxWidget::stop()
{
  pthread_mutex_lock(&mutex_stop_);
  stop_thread_ = true;
  pthread_mutex_unlock(&mutex_stop_);
  if (th_ptr_ != nullptr) pthread_join(th_, NULL);
  th_ptr_ = nullptr;
  socket_->close();
  io_.stop();
}

#include <boost/array.hpp>
void * FakeLivoxWidget::thread(void)
{
  // Send broadcast command
  sendGeneralBroadcast();

  boost::thread thr_io(boost::bind(&as::io_service::run, &io_));

  // Start an asynchronous receive
  ip::udp::endpoint ep;
  uint8_t data[kMaxCommandBufferSize] = "";
  socket_->async_receive_from(
    as::buffer(data), ep,
    boost::bind(&FakeLivoxWidget::onRead, this, as::placeholders::error, as::placeholders::bytes_transferred, data));

  while (true) {
    bool b;
    pthread_mutex_lock(&mutex_stop_);
    b = stop_thread_;
    pthread_mutex_unlock(&mutex_stop_);
    if (b) break;

    if (emit signal_get_system_status() != 0) {
      sendPushAbnormalStatus();
    }

    // 10Hz
    usleep(100000);
  }

  return nullptr;
}

void FakeLivoxWidget::dump(Direction dir, const uint8_t * data, std::size_t size)
{
  printf("%s ", (dir == Read) ? ">" : "<");

  for (std::size_t i = 0; i < size; ++i) {
    printf("%02X", data[i]);
    if (i + 1 <= size) printf(" ");
  }
  printf("\n");
}

void FakeLivoxWidget::onRead(
  const boost::system::error_code & error, std::size_t bytes_transfered, const uint8_t * data)
{
  if (error) {
    std::cout << error.message() << std::endl;
  } else {
    if (emit signal_get_debug_output()) {
      dump(Read, data, bytes_transfered);
    }

    SdkPacket * packet = const_cast<SdkPacket *>(reinterpret_cast<const SdkPacket *>(data));
    handleSdkPacket(reinterpret_cast<SdkPacket *>(packet));
  }

  // Start an asynchronous receive
  ip::udp::endpoint ep;
  uint8_t next[kMaxCommandBufferSize] = "";
  socket_->async_receive_from(
    as::buffer(next), ep,
    boost::bind(&FakeLivoxWidget::onRead, this, as::placeholders::error, as::placeholders::bytes_transferred, next));
}

void FakeLivoxWidget::onWrite(
  const boost::system::error_code & error, std::size_t bytes_transfered, const std::vector<uint8_t> & data)
{
  if (emit signal_get_debug_output()) {
    dump(Write, &data[0], bytes_transfered);
  }
}

void FakeLivoxWidget::handleSdkPacket(SdkPacket * packet)
{
  auto it = handle_map_.find(CMD_SET_ID(packet->cmd_set, packet->cmd_id));

  if (it != handle_map_.end()) {
    (this->*(it->second))(packet);
  }
}

void FakeLivoxWidget::handleGeneralHandshake(SdkPacket * packet)
{
  HandshakeRequest * req = reinterpret_cast<HandshakeRequest *>(packet->data);

  char ip[16] = "";
  uint8_t ar[4];
  memcpy(ar, &req->ip_addr, sizeof(req->ip_addr));
  snprintf(ip, sizeof(ip), "%d.%d.%d.%d", ar[0], ar[1], ar[2], ar[3]);
  std::cout << "IP: " << ip << " Data Port: " << req->data_port << " Cmd Port: " << req->cmd_port
            << " IMU Port: " << req->sensor_port << std::endl;

  user_ip_ = ip::address_v4::from_string(ip);
  data_port_ = req->data_port;
  cmd_port_ = req->cmd_port;

  SdkPacket out = {};
  out.packet_type = kCommandTypeAck;
  out.seq_num = packet->seq_num;
  out.cmd_set = packet->cmd_set;
  out.cmd_id = packet->cmd_id;

  GenericResponse payload = {};
  uint8_t * ptr = reinterpret_cast<uint8_t *>(&payload);
  payload.ret_code = kReturnCodeSuccess;

  // Disablesending of broadcast messages
  socket_->set_option(as::socket_base::broadcast(false));

  ip::udp::endpoint ep = ip::udp::endpoint(user_ip_, cmd_port_);
  send(ep, &out, ptr, sizeof(GenericResponse));
}

void FakeLivoxWidget::handleUGeneralDeviceInfo(SdkPacket * packet)
{
  SdkPacket out = {};
  out.packet_type = kCommandTypeAck;
  out.seq_num = packet->seq_num;
  out.cmd_set = packet->cmd_set;
  out.cmd_id = packet->cmd_id;

  DeviceInformationResponse payload = {};
  uint8_t * ptr = reinterpret_cast<uint8_t *>(&payload);
  payload.ret_code = kReturnCodeSuccess;
  payload.firmware_version[0] = 0x06;
  payload.firmware_version[1] = 0x08;

  ip::udp::endpoint ep = ip::udp::endpoint(user_ip_, cmd_port_);
  send(ep, &out, ptr, sizeof(DeviceInformationResponse));
}

void FakeLivoxWidget::handleGeneralHeartbeat(SdkPacket * packet)
{
  SdkPacket out = {};
  out.packet_type = kCommandTypeAck;
  out.seq_num = packet->seq_num;
  out.cmd_set = packet->cmd_set;
  out.cmd_id = packet->cmd_id;

  HeartbeatResponse payload = {};
  uint8_t * ptr = reinterpret_cast<uint8_t *>(&payload);
  payload.ret_code = emit signal_get_return_code();
  payload.state = emit signal_get_lidar_state();
  payload.feature = emit signal_get_rain_fog_suppression_switch();

  if (payload.state == kLidarStateInit) {
    payload.error_union.progress = emit signal_get_initialization_progress();
  } else {
    payload.error_union.status_code.lidar_error_code.temp_status = emit signal_get_temp_status();
    payload.error_union.status_code.lidar_error_code.volt_status = emit signal_get_volt_status();
    payload.error_union.status_code.lidar_error_code.motor_status = emit signal_get_motor_status();
    payload.error_union.status_code.lidar_error_code.dirty_warn = emit signal_get_dirty_warn();
    payload.error_union.status_code.lidar_error_code.firmware_err = emit signal_get_firmware_status();
    payload.error_union.status_code.lidar_error_code.pps_status = emit signal_get_pps_status();
    payload.error_union.status_code.lidar_error_code.device_status = emit signal_get_device_status();
    payload.error_union.status_code.lidar_error_code.fan_status = emit signal_get_fan_status();
    payload.error_union.status_code.lidar_error_code.self_heating = emit signal_get_self_heating();
    payload.error_union.status_code.lidar_error_code.ptp_status = emit signal_get_ptp_status();
    payload.error_union.status_code.lidar_error_code.time_sync_status = emit signal_get_time_sync_status();
    payload.error_union.status_code.lidar_error_code.system_status = emit signal_get_system_status();
  }

  ip::udp::endpoint ep = ip::udp::endpoint(user_ip_, cmd_port_);
  send(ep, &out, ptr, sizeof(HeartbeatResponse));
}

void FakeLivoxWidget::handleGeneralControlSample(SdkPacket * packet)
{
  ControlSampleRequest * req = reinterpret_cast<ControlSampleRequest *>(packet->data);
  std::cout << SamplingToString(req->sample_ctrl) << std::endl;

  SdkPacket out = {};
  out.packet_type = kCommandTypeAck;
  out.seq_num = packet->seq_num;
  out.cmd_set = packet->cmd_set;
  out.cmd_id = packet->cmd_id;

  GenericResponse payload = {};
  uint8_t * ptr = reinterpret_cast<uint8_t *>(&payload);
  payload.ret_code = kReturnCodeSuccess;

  ip::udp::endpoint ep = ip::udp::endpoint(user_ip_, cmd_port_);
  send(ep, &out, ptr, sizeof(GenericResponse));
}

void FakeLivoxWidget::handleGeneralCoordinateSystem(SdkPacket * packet)
{
  CoordinateSystemRequest * req = reinterpret_cast<CoordinateSystemRequest *>(packet->data);
  std::cout << CoordinateTypeToString(req->coordinate_type) << std::endl;

  SdkPacket out = {};
  out.packet_type = kCommandTypeAck;
  out.seq_num = packet->seq_num;
  out.cmd_set = packet->cmd_set;
  out.cmd_id = packet->cmd_id;

  GenericResponse payload = {};
  uint8_t * ptr = reinterpret_cast<uint8_t *>(&payload);
  payload.ret_code = kReturnCodeSuccess;

  ip::udp::endpoint ep = ip::udp::endpoint(user_ip_, cmd_port_);
  send(ep, &out, ptr, sizeof(GenericResponse));
}

void FakeLivoxWidget::handleLidarSetPointCloudReturnMode(SdkPacket * packet)
{
  SetPointCloudReturnModeRequest * req = reinterpret_cast<SetPointCloudReturnModeRequest *>(packet->data);
  std::cout << PointCloudReturnModeToString(req->mode) << std::endl;

  SdkPacket out = {};
  out.packet_type = kCommandTypeAck;
  out.seq_num = packet->seq_num;
  out.cmd_set = packet->cmd_set;
  out.cmd_id = packet->cmd_id;

  GenericResponse payload = {};
  uint8_t * ptr = reinterpret_cast<uint8_t *>(&payload);
  payload.ret_code = kReturnCodeSuccess;

  ip::udp::endpoint ep = ip::udp::endpoint(user_ip_, cmd_port_);
  send(ep, &out, ptr, sizeof(GenericResponse));
}

void FakeLivoxWidget::handleLidarSetImuPushFrequency(SdkPacket * packet)
{
  SetImuPushFrequencyRequest * req = reinterpret_cast<SetImuPushFrequencyRequest *>(packet->data);
  std::cout << ImuFreqToString(req->frequency) << std::endl;

  SdkPacket out = {};
  out.packet_type = kCommandTypeAck;
  out.seq_num = packet->seq_num;
  out.cmd_set = packet->cmd_set;
  out.cmd_id = packet->cmd_id;

  GenericResponse payload = {};
  uint8_t * ptr = reinterpret_cast<uint8_t *>(&payload);
  payload.ret_code = kReturnCodeSuccess;

  ip::udp::endpoint ep = ip::udp::endpoint(user_ip_, cmd_port_);
  send(ep, &out, ptr, sizeof(GenericResponse));
}

void FakeLivoxWidget::send(ip::udp::endpoint ep, SdkPacket * packet, uint8_t * payload, uint16_t payload_size)
{
  packet->sof = kSdkProtocolSof;
  packet->length = sizeof(SdkPacket) - 1 + payload_size + kSdkPacketCrcSize;
  packet->version = kSdkVer0;

  uint8_t * ptr = reinterpret_cast<uint8_t *>(packet);
  packet->preamble_crc = crc16_.mcrf4xx_calc(ptr, 7);

  memcpy(packet->data, payload, payload_size);

  uint32_t crc = crc32_.crc32_calc(ptr, packet->length - kSdkPacketCrcSize);

  if (!emit signal_get_checksum_error()) {
    ptr[packet->length - 4] = crc & 0xFF;
    ptr[packet->length - 3] = (crc >> 8) & 0xFF;
    ptr[packet->length - 2] = (crc >> 16) & 0xFF;
    ptr[packet->length - 1] = (crc >> 24) & 0xFF;
  } else {
    ptr[packet->length - 4] = '?';
    ptr[packet->length - 3] = '?';
    ptr[packet->length - 2] = '?';
    ptr[packet->length - 1] = '?';
  }
  std::vector<uint8_t> frame(ptr, ptr + packet->length);

  // Start an asynchronous send
  socket_->async_send_to(
    as::buffer(frame), ep,
    boost::bind(&FakeLivoxWidget::onWrite, this, as::placeholders::error, as::placeholders::bytes_transferred, frame));
}

void FakeLivoxWidget::sendGeneralBroadcast(void)
{
  SdkPacket out = {};
  out.packet_type = kCommandTypeCmd;
  out.seq_num = 0;
  out.cmd_set = kCommandSetGeneral;
  out.cmd_id = kCommandIDGeneralBroadcast;

  BroadcastDeviceInfo payload = {};
  uint8_t * ptr = reinterpret_cast<uint8_t *>(&payload);
  snprintf(
    payload.broadcast_code, kBroadcastCodeSize - 1, "%s", emit signal_get_broadcast_code().toStdString().c_str());
  payload.dev_type = kDeviceTypeLidarHorizon;

  ip::udp::endpoint ep = ip::udp::endpoint(ip::address_v4::broadcast(), 55000);
  send(ep, &out, ptr, sizeof(BroadcastDeviceInfo));
}

void FakeLivoxWidget::sendPushAbnormalStatus(void)
{
  SdkPacket out = {};
  out.packet_type = kCommandTypeMsg;
  out.seq_num = 0;
  out.cmd_set = kCommandSetGeneral;
  out.cmd_id = kCommandIDGeneralPushAbnormalState;

  LidarErrorCode payload = {};
  uint8_t * ptr = reinterpret_cast<uint8_t *>(&payload);

  payload.temp_status = emit signal_get_temp_status();
  payload.volt_status = emit signal_get_volt_status();
  payload.motor_status = emit signal_get_motor_status();
  payload.dirty_warn = emit signal_get_dirty_warn();
  payload.firmware_err = emit signal_get_firmware_status();
  payload.pps_status = emit signal_get_pps_status();
  payload.device_status = emit signal_get_device_status();
  payload.fan_status = emit signal_get_fan_status();
  payload.self_heating = emit signal_get_self_heating();
  payload.ptp_status = emit signal_get_ptp_status();
  payload.time_sync_status = emit signal_get_time_sync_status();
  payload.system_status = emit signal_get_system_status();

  ip::udp::endpoint ep = ip::udp::endpoint(user_ip_, cmd_port_);
  send(ep, &out, ptr, sizeof(LidarErrorCode));
}

void FakeLivoxWidget::updateSystemStatus(void)
{
  int level = 0;
  for (auto s : status_codes_) {
    level = std::max(s.second, level);
  }
  switch (level) {
    case 0:
      ui->radioButton_system_status_0->setChecked(true);
      break;
    case 1:
      ui->radioButton_system_status_1->setChecked(true);
      break;
    case 2:
      ui->radioButton_system_status_2->setChecked(true);
      break;
    default:
      break;
  }
}
