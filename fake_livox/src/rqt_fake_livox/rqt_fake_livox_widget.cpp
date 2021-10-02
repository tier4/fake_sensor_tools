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

#include <fmt/format.h>
#include <net/ethernet.h>
#include <boost/thread.hpp>
#include <iostream>

#include <QCheckBox>
#include <QFileDialog>
#include <QMessageBox>
#include <QProgressDialog>

#include <rqt_fake_livox/rqt_fake_livox_widget.h>
#include <ui_rqt_fake_livox_widget.h>

FakeLivoxWidget::FakeLivoxWidget(QWidget * parent) : QWidget(parent), ui(new Ui::FakeLivoxWidget)
{
  ui->setupUi(this);

  ui->comboBox_lidar_state->addItem("Initializing");
  ui->comboBox_lidar_state->addItem("Normal");
  ui->comboBox_lidar_state->addItem("Power-Saving");
  ui->comboBox_lidar_state->addItem("Standby");
  ui->comboBox_lidar_state->addItem("Error");
  ui->comboBox_lidar_state->setCurrentIndex(1);

  addButtonGroup();

  model_ = new UDPListModel();
  ui->tableView_pcap_packets->setModel(model_);
  ui->tableView_pcap_packets->setSelectionBehavior(QAbstractItemView::SelectRows);
  ui->tableView_pcap_packets->horizontalHeader()->resizeSection(0, 0);
  ui->tableView_pcap_packets->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Stretch);
  ui->tableView_pcap_packets->horizontalHeader()->setStretchLastSection(false);

  initLidarStatusCode();

  ui->radioButton_pps_status_1->animateClick();
  ui->radioButton_time_sync_status_3->animateClick();

  sdk_protocol_.setCallback(boost::bind(&FakeLivoxWidget::onControlSampleRequest, this, _1, _2, _3, _4));

  // Enumerate network interface
  getNetworkInterfaces();
}

FakeLivoxWidget::~FakeLivoxWidget() { delete ui; }

void FakeLivoxWidget::setName(const QString & name) { ui->lineEdit_name->setText(name); }

QString FakeLivoxWidget::getName() const { return ui->lineEdit_name->text(); }

void FakeLivoxWidget::setBroadcastCode(const QString & broadcast_code)
{
  ui->lineEdit_broadcast_code->setText(broadcast_code);
}

QString FakeLivoxWidget::getBroadcastCode() const { return ui->lineEdit_broadcast_code->text(); }

void FakeLivoxWidget::setNetworkInterface(const QString & interface)
{
  int i = 0;
  for (const auto & network : network_list_) {
    if (network.ifname == interface.toStdString()) {
      ui->comboBox_network_interface->setCurrentIndex(i);
      break;
    }
    ++i;
  }
}

QString FakeLivoxWidget::getNetworkInterface() const
{
  return network_list_[ui->comboBox_network_interface->currentIndex()].ifname.c_str();
}

void FakeLivoxWidget::setPcapPath(const QString & pcap_path) { ui->lineEdit_pcap_path->setText(pcap_path); }

QString FakeLivoxWidget::getPcapPath() const { return ui->lineEdit_pcap_path->text(); }

void FakeLivoxWidget::setPcapLoop(bool pcap_loop) { ui->pushButton_pcap_loop->setChecked(pcap_loop); }

bool FakeLivoxWidget::getPcapLoop() const { return loop_; }

void FakeLivoxWidget::setPcapSourceAddress(const QString & source_address)
{
  if (!source_address.isEmpty()) {
    ui->comboBox_source_address->addItem(source_address);
  }
}

QString FakeLivoxWidget::getPcapSourceAddress() const { return ui->comboBox_source_address->currentText(); }

void FakeLivoxWidget::setPcapFromList(bool pcap_from_list)
{
  ui->checkBox_select_from_list->setChecked(pcap_from_list);
}

bool FakeLivoxWidget::getPcapFromList() const { return ui->checkBox_select_from_list->isChecked(); }

void FakeLivoxWidget::on_pushButton_comm_toggled(bool checked)
{
  if (checked) {
    // Start UDP communication
    const asip::address_v4 address = network_list_[ui->comboBox_network_interface->currentIndex()].address;
    const std::string broadcast_code = ui->lineEdit_broadcast_code->text().toStdString();
    sdk_protocol_.start(address, broadcast_code);
  } else {
    // Stop UDP communication
    sdk_protocol_.stop();
    // Stop point cloud sampling
    point_cloud_.stop();
  }
}

void FakeLivoxWidget::on_pushButton_checksum_error_toggled(bool checked)
{
  sdk_protocol_.setLidarStatus(LidarStatus::ChecksumError, checked);
}

void FakeLivoxWidget::on_pushButton_debug_output_toggled(bool checked)
{
  sdk_protocol_.setLidarStatus(LidarStatus::DebugOutput, checked);
}

void FakeLivoxWidget::onButtonGroupReturnCodeClicked(int id)
{
  sdk_protocol_.setLidarStatus(LidarStatus::ReturnCode, id);
}

void FakeLivoxWidget::on_comboBox_lidar_state_currentIndexChanged(int index)
{
  int page = (0 == index) ? 0 : 1;
  ui->stackedWidget_1->setCurrentIndex(page);
  sdk_protocol_.setLidarStatus(LidarStatus::LidarState, index);
}

void FakeLivoxWidget::on_pushButton_rain_fog_suppression_switch_toggled(bool checked)
{
  sdk_protocol_.setLidarStatus(LidarStatus::RainFogSuppressionSwitch, checked);
}

void FakeLivoxWidget::on_slider_initialization_progress_valueChanged(double value)
{
  sdk_protocol_.setLidarStatus(LidarStatus::InitializationProgress, value);
}

void FakeLivoxWidget::onButtonGroupTempStatusClicked(int id) { notifyLidarStatusCode(LidarStatusCode::TempStatus, id); }

void FakeLivoxWidget::onButtonGroupVoltStatusClicked(int id) { notifyLidarStatusCode(LidarStatusCode::VoltStatus, id); }

void FakeLivoxWidget::onButtonGroupMotorStatusClicked(int id)
{
  notifyLidarStatusCode(LidarStatusCode::MotorStatus, id);
}

void FakeLivoxWidget::onButtonGroupDirtyWarnClicked(int id) { notifyLidarStatusCode(LidarStatusCode::DirtyWarn, id); }

void FakeLivoxWidget::onButtonGroupFirmwareStatusClicked(int id)
{
  notifyLidarStatusCode(LidarStatusCode::FirmwareStatus, id);
}

void FakeLivoxWidget::onButtonGroupPpsStatusClicked(int id) { notifyLidarStatusCode(LidarStatusCode::PpsStatus, id); }

void FakeLivoxWidget::onButtonGroupDeviceStatusClicked(int id)
{
  notifyLidarStatusCode(LidarStatusCode::DeviceStatus, id);
}

void FakeLivoxWidget::onButtonGroupFanStatusClicked(int id) { notifyLidarStatusCode(LidarStatusCode::FanStatus, id); }

void FakeLivoxWidget::onButtonGroupSelfHeatingClicked(int id)
{
  notifyLidarStatusCode(LidarStatusCode::SelfHeating, id);
}

void FakeLivoxWidget::onButtonGroupPtpStatusClicked(int id) { notifyLidarStatusCode(LidarStatusCode::PtpStatus, id); }

void FakeLivoxWidget::onButtonGroupTimeSyncStatusClicked(int id)
{
  notifyLidarStatusCode(LidarStatusCode::TimeSyncStatus, id);
}

void FakeLivoxWidget::onButtonGroupSystemStatusClicked(int id)
{
  notifyLidarStatusCode(LidarStatusCode::SystemStatus, id);
}

void FakeLivoxWidget::on_pushButton_pcap_path_clicked()
{
  QString text = ui->lineEdit_pcap_path->text();
  QString dir = "";
  if (!text.isEmpty()) {
    QFileInfo info(text);
    dir = info.absoluteFilePath();
  }

  QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), dir, tr("PCAP files (*.pcap)"));

  if (!fileName.isEmpty()) {
    ui->lineEdit_pcap_path->setText(fileName);
    // Read pcap
    readPcap(fileName);
  }
}

void FakeLivoxWidget::on_pushButton_pcap_read_clicked()
{
  QString fileName = ui->lineEdit_pcap_path->text();

  if (!fileName.isEmpty()) {
    // Read pcap
    readPcap(fileName);
  }
}

void FakeLivoxWidget::on_pushButton_pcap_loop_toggled(bool checked) { loop_ = checked; }

void FakeLivoxWidget::on_tableView_pcap_packets_doubleClicked(const QModelIndex & index)
{
  model_->toggleTransmit(index);
  // Updates the area
  ui->tableView_pcap_packets->update();
}

void FakeLivoxWidget::on_checkBox_select_from_list_toggled(bool checked)
{
  model_->setEnabled(checked);
  ui->comboBox_source_address->setEnabled(!checked);
  // Updates the area
  ui->tableView_pcap_packets->update();
}

void FakeLivoxWidget::initLidarStatusCode()
{
  status_code_[LidarStatusCode::TempStatus] = LidarStatusCodeValue(true, 0);
  status_code_[LidarStatusCode::VoltStatus] = LidarStatusCodeValue(true, 0);
  status_code_[LidarStatusCode::MotorStatus] = LidarStatusCodeValue(true, 0);
  status_code_[LidarStatusCode::DirtyWarn] = LidarStatusCodeValue(true, 0);
  status_code_[LidarStatusCode::FirmwareStatus] = LidarStatusCodeValue(true, 0);
  status_code_[LidarStatusCode::PpsStatus] = LidarStatusCodeValue(false, 0);
  status_code_[LidarStatusCode::DeviceStatus] = LidarStatusCodeValue(true, 0);
  status_code_[LidarStatusCode::FanStatus] = LidarStatusCodeValue(true, 0);
  status_code_[LidarStatusCode::SelfHeating] = LidarStatusCodeValue(false, 0);
  status_code_[LidarStatusCode::PtpStatus] = LidarStatusCodeValue(false, 0);
  status_code_[LidarStatusCode::TimeSyncStatus] = LidarStatusCodeValue(false, 0);
  status_code_[LidarStatusCode::SystemStatus] = LidarStatusCodeValue(false, 0);
}

uint32_t FakeLivoxWidget::getLidarStatusCode(const LidarStatusCode & name, int value)
{
  status_code_[name].value = value;

  int level = 0;
  for (auto s : status_code_) {
    // If situation will trigger system warning or error
    if (s.second.trigger_system_status) level = std::max(s.second.value, level);
  }
  status_code_[LidarStatusCode::SystemStatus].value = level;
  buttonGroup_system_status_->button(level)->setChecked(true);

  ErrorMessage status_code = {};
  status_code.lidar_error_code.temp_status = status_code_[LidarStatusCode::TempStatus].value;
  status_code.lidar_error_code.volt_status = status_code_[LidarStatusCode::VoltStatus].value;
  status_code.lidar_error_code.motor_status = status_code_[LidarStatusCode::MotorStatus].value;
  status_code.lidar_error_code.dirty_warn = status_code_[LidarStatusCode::DirtyWarn].value;
  status_code.lidar_error_code.firmware_err = status_code_[LidarStatusCode::FirmwareStatus].value;
  status_code.lidar_error_code.pps_status = status_code_[LidarStatusCode::PpsStatus].value;
  status_code.lidar_error_code.device_status = status_code_[LidarStatusCode::DeviceStatus].value;
  status_code.lidar_error_code.fan_status = status_code_[LidarStatusCode::FanStatus].value;
  status_code.lidar_error_code.self_heating = status_code_[LidarStatusCode::SelfHeating].value;
  status_code.lidar_error_code.ptp_status = status_code_[LidarStatusCode::PtpStatus].value;
  status_code.lidar_error_code.time_sync_status = status_code_[LidarStatusCode::TimeSyncStatus].value;
  status_code.lidar_error_code.system_status = status_code_[LidarStatusCode::SystemStatus].value;

  return status_code.error_code;
}

void FakeLivoxWidget::notifyLidarStatusCode(const LidarStatusCode & name, uint32_t value)
{
  uint32_t status_code = getLidarStatusCode(name, value);

  sdk_protocol_.setLidarStatusCode(status_code);
  point_cloud_.setLidarStatusCode(status_code);
}

void FakeLivoxWidget::onControlSampleRequest(
  const ControlSampleRequest * request, const asip::address_v4 & livox_ip, const asip::address_v4 & user_ip,
  uint16_t data_port)
{
  if (!request->sample_ctrl) {
    // Stop point cloud sampling
    point_cloud_.stop();
    return;
  }

  QString fileName = ui->lineEdit_pcap_path->text();
  if (!fileName.isEmpty()) {
    const std::string str(fileName.toLocal8Bit());

    // Start point cloud sampling
    point_cloud_.start(str, createFilter(), livox_ip, user_ip, data_port, loop_);
  }
}

void FakeLivoxWidget::addButtonGroup()
{
  QButtonGroup * buttonGroup_return_code = new QButtonGroup(this);
  buttonGroup_return_code->addButton(ui->radioButton_return_code_0, 0);
  buttonGroup_return_code->addButton(ui->radioButton_return_code_1, 1);
  connect(buttonGroup_return_code, SIGNAL(buttonClicked(int)), this, SLOT(onButtonGroupReturnCodeClicked(int)));

  QButtonGroup * buttonGroup_temp_status = new QButtonGroup(this);
  buttonGroup_temp_status->addButton(ui->radioButton_temp_status_0, 0);
  buttonGroup_temp_status->addButton(ui->radioButton_temp_status_1, 1);
  buttonGroup_temp_status->addButton(ui->radioButton_temp_status_2, 2);
  connect(buttonGroup_temp_status, SIGNAL(buttonClicked(int)), this, SLOT(onButtonGroupTempStatusClicked(int)));

  QButtonGroup * buttonGroup_volt_status = new QButtonGroup(this);
  buttonGroup_volt_status->addButton(ui->radioButton_volt_status_0, 0);
  buttonGroup_volt_status->addButton(ui->radioButton_volt_status_1, 1);
  buttonGroup_volt_status->addButton(ui->radioButton_volt_status_2, 2);
  connect(buttonGroup_volt_status, SIGNAL(buttonClicked(int)), this, SLOT(onButtonGroupVoltStatusClicked(int)));

  QButtonGroup * buttonGroup_motor_status = new QButtonGroup(this);
  buttonGroup_motor_status->addButton(ui->radioButton_motor_status_0, 0);
  buttonGroup_motor_status->addButton(ui->radioButton_motor_status_1, 1);
  buttonGroup_motor_status->addButton(ui->radioButton_motor_status_2, 2);
  connect(buttonGroup_motor_status, SIGNAL(buttonClicked(int)), this, SLOT(onButtonGroupMotorStatusClicked(int)));

  QButtonGroup * buttonGroup_dirty_warn = new QButtonGroup(this);
  buttonGroup_dirty_warn->addButton(ui->radioButton_dirty_warn_0, 0);
  buttonGroup_dirty_warn->addButton(ui->radioButton_dirty_warn_1, 1);
  connect(buttonGroup_dirty_warn, SIGNAL(buttonClicked(int)), this, SLOT(onButtonGroupDirtyWarnClicked(int)));

  QButtonGroup * buttonGroup_firmware_status = new QButtonGroup(this);
  buttonGroup_firmware_status->addButton(ui->radioButton_firmware_status_0, 0);
  buttonGroup_firmware_status->addButton(ui->radioButton_firmware_status_1, 1);
  connect(buttonGroup_firmware_status, SIGNAL(buttonClicked(int)), this, SLOT(onButtonGroupFirmwareStatusClicked(int)));

  QButtonGroup * buttonGroup_pps_status = new QButtonGroup(this);
  buttonGroup_pps_status->addButton(ui->radioButton_pps_status_0, 0);
  buttonGroup_pps_status->addButton(ui->radioButton_pps_status_1, 1);
  connect(buttonGroup_pps_status, SIGNAL(buttonClicked(int)), this, SLOT(onButtonGroupPpsStatusClicked(int)));

  QButtonGroup * buttonGroup_device_status = new QButtonGroup(this);
  buttonGroup_device_status->addButton(ui->radioButton_device_status_0, 0);
  buttonGroup_device_status->addButton(ui->radioButton_device_status_1, 1);
  connect(buttonGroup_device_status, SIGNAL(buttonClicked(int)), this, SLOT(onButtonGroupDeviceStatusClicked(int)));

  QButtonGroup * buttonGroup_fan_status = new QButtonGroup(this);
  buttonGroup_fan_status->addButton(ui->radioButton_fan_status_0, 0);
  buttonGroup_fan_status->addButton(ui->radioButton_fan_status_1, 1);
  connect(buttonGroup_fan_status, SIGNAL(buttonClicked(int)), this, SLOT(onButtonGroupFanStatusClicked(int)));

  QButtonGroup * buttonGroup_self_heating = new QButtonGroup(this);
  buttonGroup_self_heating->addButton(ui->radioButton_self_heating_0, 0);
  buttonGroup_self_heating->addButton(ui->radioButton_self_heating_1, 1);
  connect(buttonGroup_self_heating, SIGNAL(buttonClicked(int)), this, SLOT(onButtonGroupSelfHeatingClicked(int)));

  QButtonGroup * buttonGroup_ptp_status = new QButtonGroup(this);
  buttonGroup_ptp_status->addButton(ui->radioButton_ptp_status_0, 0);
  buttonGroup_ptp_status->addButton(ui->radioButton_ptp_status_1, 1);
  connect(buttonGroup_ptp_status, SIGNAL(buttonClicked(int)), this, SLOT(onButtonGroupPtpStatusClicked(int)));

  QButtonGroup * buttonGroup_time_sync_status = new QButtonGroup(this);
  buttonGroup_time_sync_status->addButton(ui->radioButton_time_sync_status_0, 0);
  buttonGroup_time_sync_status->addButton(ui->radioButton_time_sync_status_1, 1);
  buttonGroup_time_sync_status->addButton(ui->radioButton_time_sync_status_2, 2);
  buttonGroup_time_sync_status->addButton(ui->radioButton_time_sync_status_3, 2);
  buttonGroup_time_sync_status->addButton(ui->radioButton_time_sync_status_4, 2);
  connect(
    buttonGroup_time_sync_status, SIGNAL(buttonClicked(int)), this, SLOT(onButtonGroupTimeSyncStatusClicked(int)));

  buttonGroup_system_status_ = new QButtonGroup(this);
  buttonGroup_system_status_->addButton(ui->radioButton_system_status_0, 0);
  buttonGroup_system_status_->addButton(ui->radioButton_system_status_1, 1);
  buttonGroup_system_status_->addButton(ui->radioButton_system_status_2, 2);
  connect(buttonGroup_system_status_, SIGNAL(buttonClicked(int)), this, SLOT(onButtonGroupSystemStatusClicked(int)));
}

void FakeLivoxWidget::getNetworkInterfaces()
{
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    std::cout << "Failed to create a new socket. " << strerror(errno) << std::endl;
    return;
  }

  // Get array of ifreq structures length in bytes
  struct ifconf ifc = {};
  int ret = ioctl(sock, SIOCGIFCONF, &ifc);
  if (ret != 0) {
    std::cout << "Failed to get array of ifreq structures length. " << strerror(errno) << std::endl;
    ::close(sock);
    return;
  }
  if (ifc.ifc_len < 0) {
    std::cout << "No ifreq structures." << std::endl;
    ::close(sock);
    return;
  }

  // Get list of interface
  ifc.ifc_ifcu.ifcu_buf = (caddr_t)malloc(ifc.ifc_len);
  ret = ioctl(sock, SIOCGIFCONF, &ifc);
  if (ret != 0) {
    std::cout << "Failed to get list of interface. " << strerror(errno) << std::endl;
    ::close(sock);
    free(ifc.ifc_ifcu.ifcu_buf);
    return;
  }

  int num = ifc.ifc_len / sizeof(struct ifreq);
  struct ifreq * ifr = (struct ifreq *)ifc.ifc_ifcu.ifcu_buf;

  for (int i = 0; i < num; ++i) {
    char * name = ifr[i].ifr_name;

    // Get interface address
    struct ifreq ifr;
    strncpy(ifr.ifr_name, name, IFNAMSIZ - 1);
    ret = ioctl(sock, SIOCGIFADDR, &ifr);
    if (ret != 0) {
      std::cout << "Failed to get interface address. " << strerror(errno) << std::endl;
      break;
    }

    char * addr = inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);
    network_list_.emplace_back(name, asip::address_v4::from_string(addr));

    std::string item = fmt::format("{} / {}", name, addr);
    ui->comboBox_network_interface->addItem(item.c_str());
  }

  ::close(sock);
  free(ifc.ifc_ifcu.ifcu_buf);
}

void FakeLivoxWidget::readPcap(const QString & fileName)
{
  if (!fileName.isEmpty()) {
    std::string str(fileName.toLocal8Bit());

    // Get packet count from pcap
    int ret = getPcapPacketCount(str.c_str());
    if (ret < 0) return;
    packet_count_ = ret;

    // Get packet data from pcap
    ret = getPcapPacketData(str.c_str());
    if (ret < 0) return;
  }
}

int FakeLivoxWidget::getPcapPacketCount(const char * fname)
{
  // Open a saved capture file for reading
  char errbuf[PCAP_ERRBUF_SIZE];
  pcap_ = pcap_open_offline(fname, errbuf);
  if (!pcap_) {
    QMessageBox msgBox(QMessageBox::Critical, "Error", errbuf, QMessageBox::Ok, this);
    msgBox.exec();
    return -1;
  }

  // Read the next packet from a pcap_t
  const u_char * p;
  struct pcap_pkthdr h;
  int count = 0;
  while ((p = pcap_next(pcap_, &h))) {
    ++count;
  }
  // Close a capture device or savefile
  pcap_close(pcap_);

  return count;
}

int FakeLivoxWidget::getPcapPacketData(const char * fname)
{
  QProgressDialog progress("Reading file...", "Cancel", 0, packet_count_, this);
  progress.setWindowTitle("PCAP");
  progress.setWindowModality(Qt::WindowModal);
  progress.setFixedWidth(600);
  progress.show();

  // Open a saved capture file for reading
  char errbuf[PCAP_ERRBUF_SIZE];
  pcap_ = pcap_open_offline(fname, errbuf);
  if (!pcap_) {
    QMessageBox msgBox(QMessageBox::Critical, "Error", errbuf, QMessageBox::Ok, this);
    msgBox.exec();
    return -1;
  }

  // Remove all items
  model_->removeAll();

  // Read the next packet from a pcap_t
  const u_char * p;
  struct pcap_pkthdr h;
  int i = 0;
  while ((p = pcap_next(pcap_, &h))) {
    progress.setValue(i);

    if (h.caplen >= sizeof(struct iphdr) + sizeof(struct ether_header)) {
      const u_char * ptr = p;
      ptr += sizeof(struct ether_header);
      const struct iphdr * ip = reinterpret_cast<const struct iphdr *>(ptr);
      if (ip->protocol == IPPROTO_UDP && !IN_MULTICAST(ntohl(ip->daddr))) {
        ptr += ((struct iphdr *)ptr)->ihl * 4;
        const struct udphdr * udp = (struct udphdr *)(ptr);
        addUDPInfo(ip, udp);
      }
    }

    if (progress.wasCanceled()) break;

    ++i;
  }

  progress.setValue(packet_count_);

  // Close a capture device or savefile
  pcap_close(pcap_);

  // Add source addresses to combo box.
  addSourceAddresses();

  return i;
}

void FakeLivoxWidget::addUDPInfo(const struct iphdr * ip, const struct udphdr * udp)
{
  char saddr[64];
  char daddr[64];

  inet_ntop(AF_INET, &ip->saddr, saddr, sizeof(saddr));
  inet_ntop(AF_INET, &ip->daddr, daddr, sizeof(daddr));
  int sport = ntohs(udp->source);
  int dport = ntohs(udp->dest);

  UDPInfo info(tr(saddr), sport, tr(daddr), dport, 1);
  model_->add(info);
}

void FakeLivoxWidget::addSourceAddresses()
{
  // Get current text
  QString current = getPcapSourceAddress();
  ui->comboBox_source_address->clear();

  for (int i = 0; i < model_->rowCount(); ++i) {
    QVariant content = model_->data(model_->index(i, 1), Qt::DisplayRole);
    QString address = content.toString();
    if (ui->comboBox_source_address->findText(address) < 0) {
      ui->comboBox_source_address->addItem(address);
    }
  }

  if (!current.isEmpty()) ui->comboBox_source_address->setCurrentText(current);
}

std::string FakeLivoxWidget::createFilter()
{
  if (getPcapFromList()) {
    return model_->getFilter();
  }

  std::string source = getPcapSourceAddress().toStdString();
  return fmt::format("(src {} and src port 60001) or (src {} and src port 60003)", source, source);
}
