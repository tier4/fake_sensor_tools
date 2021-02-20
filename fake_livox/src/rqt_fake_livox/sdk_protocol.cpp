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
 * @file sdk_protocol.cpp
 * @brief Handling SDK protocol class
 */

#include <boost/thread.hpp>
#include <iostream>

#include <rqt_fake_livox/sdk_protocol.h>

std::map<CmdSetId, SDKProtocol::HandleFunction> SDKProtocol::handle_map_ = {
  {{kCommandSetGeneral, kCommandIDGeneralHandshake}, &SDKProtocol::handleGeneralHandshake},
  {{kCommandSetGeneral, kCommandIDGeneralDeviceInfo}, &SDKProtocol::handleGeneralDeviceInfo},
  {{kCommandSetGeneral, kCommandIDGeneralHeartbeat}, &SDKProtocol::handleGeneralHeartbeat},
  {{kCommandSetGeneral, kCommandIDGeneralControlSample}, &SDKProtocol::handleGeneralControlSample},
  {{kCommandSetGeneral, kCommandIDGeneralCoordinateSystem}, &SDKProtocol::handleGeneralCoordinateSystem},
  {{kCommandSetLidar, kCommandIDLidarSetPointCloudReturnMode}, &SDKProtocol::handleLidarSetPointCloudReturnMode},
  {{kCommandSetLidar, kCommandIDLidarSetImuPushFrequency}, &SDKProtocol::handleLidarSetImuPushFrequency},
};

SDKProtocol::SDKProtocol()
: th_ptr_(nullptr), stop_thread_(false), crc16_(0x4c49), crc32_(0x564f580a), broadcast_code_("")
{
  initLidarStatus();
}

SDKProtocol::~SDKProtocol() {}

void SDKProtocol::setLidarStatus(const LidarStatus & name, int value)
{
  std::lock_guard<std::mutex> lock(mutex_config_);
  status_[name].value = value;
  setSystemStatus();
}

int SDKProtocol::getSystemStatus()
{
  std::lock_guard<std::mutex> lock(mutex_config_);
  return status_[LidarStatus::SystemStatus].value;
}

int SDKProtocol::start(const asip::address_v4 & livox_ip, const std::string & broadcast_code)
{
  int ret = 0;

  broadcast_code_ = broadcast_code;

  // Preparation for a subsequent run() invocation
  io_.reset();
  socket_ = boost::shared_ptr<asip::udp::socket>(new asip::udp::socket(io_));

  // Open the socket using the specified protocol
  try {
    socket_->open(asip::udp::v4());
    // Allow the socket to be bound to an address that is already in use
    socket_->set_option(asip::udp::socket::reuse_address(true));
    // Permit sending of broadcast messages
    socket_->set_option(as::socket_base::broadcast(true));
    asip::udp::endpoint ep = asip::udp::endpoint(livox_ip, 65000);
    socket_->bind(ep);
  } catch (const boost::system::system_error & e) {
    ret = ENOENT;
    std::cerr << e.what() << std::endl;
    return ret;
  }

  stop_thread_ = false;
  pthread_create(&th_, nullptr, &SDKProtocol::threadHelper, this);
  th_ptr_ = &th_;
  return ret;
}

void SDKProtocol::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_stop_);
    stop_thread_ = true;
  }
  if (th_ptr_ != nullptr) pthread_join(th_, NULL);
  th_ptr_ = nullptr;
  socket_->close();

  timer_.reset();
  io_.stop();
}

void * SDKProtocol::thread()
{
  // Start an asynchronous wait on the timer.
  timer_ = boost::shared_ptr<as::deadline_timer>(new as::deadline_timer(io_, boost::posix_time::seconds(0)));
  timer_->async_wait(boost::bind(&SDKProtocol::onTimer, this, as::placeholders::error));

  boost::thread thr_io(boost::bind(&as::io_service::run, &io_));

  // Start an asynchronous receive
  asip::udp::endpoint ep;
  uint8_t data[kMaxCommandBufferSize] = "";
  socket_->async_receive_from(
    as::buffer(data), ep,
    boost::bind(&SDKProtocol::onRead, this, as::placeholders::error, as::placeholders::bytes_transferred, data));

  while (true) {
    {
      std::lock_guard<std::mutex> lock(mutex_stop_);
      if (stop_thread_) break;
    }

    // 10Hz
    usleep(100000);
  }

  return nullptr;
}

void SDKProtocol::initLidarStatus()
{
  status_[LidarStatus::ChecksumError] = LidarValue(false, 0);
  status_[LidarStatus::DebugOutput] = LidarValue(false, 0);
  status_[LidarStatus::ReturnCode] = LidarValue(false, 0);
  status_[LidarStatus::LidarState] = LidarValue(false, 1);
  status_[LidarStatus::RainFogSuppressionSwitch] = LidarValue(false, 0);
  status_[LidarStatus::InitializationProgress] = LidarValue(false, 0);
  status_[LidarStatus::TempStatus] = LidarValue(true, 0);
  status_[LidarStatus::VoltStatus] = LidarValue(true, 0);
  status_[LidarStatus::MotorStatus] = LidarValue(true, 0);
  status_[LidarStatus::DirtyWarn] = LidarValue(true, 0);
  status_[LidarStatus::FirmwareStatus] = LidarValue(true, 0);
  status_[LidarStatus::PpsStatus] = LidarValue(false, 0);
  status_[LidarStatus::DeviceStatus] = LidarValue(true, 0);
  status_[LidarStatus::FanStatus] = LidarValue(true, 0);
  status_[LidarStatus::SelfHeating] = LidarValue(false, 0);
  status_[LidarStatus::PtpStatus] = LidarValue(false, 0);
  status_[LidarStatus::TimeSyncStatus] = LidarValue(false, 0);
  status_[LidarStatus::SystemStatus] = LidarValue(false, 0);
}

int SDKProtocol::getLidarStatus(const LidarStatus & name)
{
  std::lock_guard<std::mutex> lock(mutex_config_);
  return status_[name].value;
}

void SDKProtocol::setSystemStatus()
{
  int level = 0;
  for (auto s : status_) {
    // If situation will trigger system warning or error
    if (s.second.trigger_system_status) level = std::max(s.second.value, level);
  }
  status_[LidarStatus::SystemStatus].value = level;
}

void SDKProtocol::dump(Direction dir, const uint8_t * data, std::size_t size)
{
  printf("%s ", (dir == Read) ? ">" : "<");

  for (std::size_t i = 0; i < size; ++i) {
    printf("%02X", data[i]);
    if (i + 1 <= size) printf(" ");
  }
  printf("\n");
}

void SDKProtocol::onTimer(const boost::system::error_code & error)
{
  if (error) return;

  // Send broadcast command
  sendGeneralBroadcast();

  // Start an asynchronous wait on the timer.
  timer_->expires_from_now(boost::posix_time::seconds(1));
  timer_->async_wait(boost::bind(&SDKProtocol::onTimer, this, as::placeholders::error));
}

void SDKProtocol::onRead(const boost::system::error_code & error, std::size_t bytes_transferred, const uint8_t * data)
{
  if (error) return;

  if (getLidarStatus(LidarStatus::DebugOutput)) {
    dump(Read, data, bytes_transferred);
  }

  SdkPacket * packet = const_cast<SdkPacket *>(reinterpret_cast<const SdkPacket *>(data));
  handleSdkPacket(reinterpret_cast<SdkPacket *>(packet));

  // Start an asynchronous receive
  asip::udp::endpoint ep;
  uint8_t next[kMaxCommandBufferSize] = "";
  socket_->async_receive_from(
    as::buffer(next), ep,
    boost::bind(&SDKProtocol::onRead, this, as::placeholders::error, as::placeholders::bytes_transferred, next));
}

void SDKProtocol::onWrite(
  const boost::system::error_code & error, std::size_t bytes_transferred, const std::vector<uint8_t> & data)
{
  if (error) return;

  if (getLidarStatus(LidarStatus::DebugOutput)) {
    dump(Write, &data[0], bytes_transferred);
  }
}

void SDKProtocol::handleSdkPacket(SdkPacket * packet)
{
  auto it = handle_map_.find(CmdSetId(packet->cmd_set, packet->cmd_id));

  if (it != handle_map_.end()) {
    (this->*(it->second))(packet);
  }
}

void SDKProtocol::handleGeneralHandshake(SdkPacket * packet)
{
  HandshakeRequest * req = reinterpret_cast<HandshakeRequest *>(packet->data);

  char ip[16] = "";
  uint8_t ar[4];
  memcpy(ar, &req->ip_addr, sizeof(req->ip_addr));
  snprintf(ip, sizeof(ip), "%d.%d.%d.%d", ar[0], ar[1], ar[2], ar[3]);
  std::cout << "IP: " << ip << " Data Port: " << req->data_port << " Cmd Port: " << req->cmd_port
            << " IMU Port: " << req->sensor_port << std::endl;

  user_ip_ = asip::address_v4::from_string(ip);
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

  asip::udp::endpoint ep = asip::udp::endpoint(user_ip_, cmd_port_);
  send(ep, &out, ptr, sizeof(GenericResponse));

  timer_->cancel();
}

void SDKProtocol::handleGeneralDeviceInfo(SdkPacket * packet)
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

  asip::udp::endpoint ep = asip::udp::endpoint(user_ip_, cmd_port_);
  send(ep, &out, ptr, sizeof(DeviceInformationResponse));
}

void SDKProtocol::handleGeneralHeartbeat(SdkPacket * packet)
{
  SdkPacket out = {};
  out.packet_type = kCommandTypeAck;
  out.seq_num = packet->seq_num;
  out.cmd_set = packet->cmd_set;
  out.cmd_id = packet->cmd_id;

  HeartbeatResponse payload = {};
  uint8_t * ptr = reinterpret_cast<uint8_t *>(&payload);
  payload.ret_code = getLidarStatus(LidarStatus::ReturnCode);
  payload.state = getLidarStatus(LidarStatus::LidarState);
  payload.feature = getLidarStatus(LidarStatus::RainFogSuppressionSwitch);

  if (payload.state == kLidarStateInit) {
    payload.error_union.progress = getLidarStatus(LidarStatus::InitializationProgress);
  } else {
    payload.error_union.status_code.lidar_error_code.temp_status = getLidarStatus(LidarStatus::TempStatus);
    payload.error_union.status_code.lidar_error_code.volt_status = getLidarStatus(LidarStatus::VoltStatus);
    payload.error_union.status_code.lidar_error_code.motor_status = getLidarStatus(LidarStatus::MotorStatus);
    payload.error_union.status_code.lidar_error_code.dirty_warn = getLidarStatus(LidarStatus::DirtyWarn);
    payload.error_union.status_code.lidar_error_code.firmware_err = getLidarStatus(LidarStatus::FirmwareStatus);
    payload.error_union.status_code.lidar_error_code.pps_status = getLidarStatus(LidarStatus::PpsStatus);
    payload.error_union.status_code.lidar_error_code.device_status = getLidarStatus(LidarStatus::DeviceStatus);
    payload.error_union.status_code.lidar_error_code.fan_status = getLidarStatus(LidarStatus::FanStatus);
    payload.error_union.status_code.lidar_error_code.self_heating = getLidarStatus(LidarStatus::SelfHeating);
    payload.error_union.status_code.lidar_error_code.ptp_status = getLidarStatus(LidarStatus::PtpStatus);
    payload.error_union.status_code.lidar_error_code.time_sync_status = getLidarStatus(LidarStatus::TimeSyncStatus);
    payload.error_union.status_code.lidar_error_code.system_status = getLidarStatus(LidarStatus::SystemStatus);
  }

  asip::udp::endpoint ep = asip::udp::endpoint(user_ip_, cmd_port_);
  send(ep, &out, ptr, sizeof(HeartbeatResponse));
}

void SDKProtocol::handleGeneralControlSample(SdkPacket * packet)
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

  asip::udp::endpoint ep = asip::udp::endpoint(user_ip_, cmd_port_);
  send(ep, &out, ptr, sizeof(GenericResponse));

  if (callback_) callback_(req, user_ip_, data_port_);
}

void SDKProtocol::handleGeneralCoordinateSystem(SdkPacket * packet)
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

  asip::udp::endpoint ep = asip::udp::endpoint(user_ip_, cmd_port_);
  send(ep, &out, ptr, sizeof(GenericResponse));
}

void SDKProtocol::handleLidarSetPointCloudReturnMode(SdkPacket * packet)
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

  asip::udp::endpoint ep = asip::udp::endpoint(user_ip_, cmd_port_);
  send(ep, &out, ptr, sizeof(GenericResponse));
}

void SDKProtocol::handleLidarSetImuPushFrequency(SdkPacket * packet)
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

  asip::udp::endpoint ep = asip::udp::endpoint(user_ip_, cmd_port_);
  send(ep, &out, ptr, sizeof(GenericResponse));
}

void SDKProtocol::send(asip::udp::endpoint ep, SdkPacket * packet, uint8_t * payload, uint16_t payload_size)
{
  packet->sof = kSdkProtocolSof;
  packet->length = sizeof(SdkPacket) - 1 + payload_size + kSdkPacketCrcSize;
  packet->version = kSdkVer0;

  uint8_t * ptr = reinterpret_cast<uint8_t *>(packet);
  packet->preamble_crc = crc16_.mcrf4xx_calc(ptr, 7);

  memcpy(packet->data, payload, payload_size);

  uint32_t crc = crc32_.crc32_calc(ptr, packet->length - kSdkPacketCrcSize);

  if (!getLidarStatus(LidarStatus::ChecksumError)) {
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
    boost::bind(&SDKProtocol::onWrite, this, as::placeholders::error, as::placeholders::bytes_transferred, frame));
}

void SDKProtocol::sendGeneralBroadcast()
{
  SdkPacket out = {};
  out.packet_type = kCommandTypeCmd;
  out.seq_num = 0;
  out.cmd_set = kCommandSetGeneral;
  out.cmd_id = kCommandIDGeneralBroadcast;

  BroadcastDeviceInfo payload = {};
  uint8_t * ptr = reinterpret_cast<uint8_t *>(&payload);
  snprintf(payload.broadcast_code, kBroadcastCodeSize, "%s", broadcast_code_.c_str());
  payload.dev_type = kDeviceTypeLidarHorizon;

  asip::udp::endpoint ep = asip::udp::endpoint(asip::address_v4::broadcast(), 55000);
  send(ep, &out, ptr, sizeof(BroadcastDeviceInfo));
}
