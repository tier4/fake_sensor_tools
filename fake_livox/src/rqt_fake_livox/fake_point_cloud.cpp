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
 * @file fake_point_cloud.cpp
 * @brief Fake point cloud sampling class
 */

#include <net/ethernet.h>
#include <boost/thread.hpp>
#include <iostream>

#include <livox.h>
#include <rqt_fake_livox/fake_point_cloud.h>

FakePointCloud::FakePointCloud() : th_ptr_(nullptr), stop_thread_(false), loop_(false), status_code_(0) {}

FakePointCloud::~FakePointCloud() {}

int FakePointCloud::start(
  const std::string & pcap_path, const std::string & pcap_filter, const asip::address_v4 & livox_ip,
  const asip::address_v4 & user_ip, uint16_t data_port, bool loop)
{
  int ret = 0;

  pcap_path_ = pcap_path;
  pcap_filter_ = pcap_filter;
  user_ip_ = user_ip;
  data_port_ = data_port;
  loop_ = loop;

  // Preparation for a subsequent run() invocation
  io_.reset();
  socket_ = boost::shared_ptr<asip::udp::socket>(new asip::udp::socket(io_));

  // Open the socket using the specified protocol
  try {
    socket_->open(asip::udp::v4());
    // Allow the socket to be bound to an address that is already in use
    socket_->set_option(asip::udp::socket::reuse_address(true));
    asip::udp::endpoint ep = asip::udp::endpoint(livox_ip, data_port + 1);
    socket_->bind(ep);
  } catch (const boost::system::system_error & e) {
    ret = ENOENT;
    std::cerr << e.what() << std::endl;
    return ret;
  }

  stop_thread_ = false;
  pthread_create(&th_, nullptr, &FakePointCloud::threadHelper, this);
  th_ptr_ = &th_;

  return ret;
}

void FakePointCloud::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_stop_);
    stop_thread_ = true;
  }
  if (th_ptr_ != nullptr) {
    pthread_join(th_, NULL);
    th_ptr_ = nullptr;
    socket_->close();
    io_.stop();
    th_ptr_ = nullptr;
  }
}

void FakePointCloud::setLidarStatusCode(uint32_t status_code)
{
  std::lock_guard<std::mutex> lock(mutex_config_);
  status_code_ = status_code;
}

void * FakePointCloud::thread()
{
  boost::thread thr_io(boost::bind(&as::io_service::run, &io_));

  while (true) {
    {
      std::lock_guard<std::mutex> lock(mutex_stop_);
      if (stop_thread_) break;
    }

    // Open pcap file
    if (openPcap() != 0) break;
    // Read pcap file and send packet.
    performPcap();
    // Close pcap file.
    closePcap();

    if (!loop_) break;
  }

  // Close pcap file.
  closePcap();

  return nullptr;
}

int FakePointCloud::openPcap()
{
  // Open a saved capture file for reading
  char errbuf[PCAP_ERRBUF_SIZE];
  pcap_ = pcap_open_offline(pcap_path_.c_str(), errbuf);
  if (!pcap_) {
    std::cout << errbuf << std::endl;
    return -1;
  }

  struct bpf_program filter;
  // Compile a filter expression
  if (pcap_compile(pcap_, &filter, pcap_filter_.c_str(), 1, 0) != 0) {
    std::cout << pcap_geterr(pcap_) << std::endl;
    closePcap();
    return -1;
  }
  // Set the filter
  if (pcap_setfilter(pcap_, &filter) != 0) {
    std::cout << pcap_geterr(pcap_) << std::endl;
    closePcap();
    return -1;
  }

  return 0;
}

void FakePointCloud::closePcap()
{
  if (pcap_ != nullptr) {
    // Close a capture device or savefile
    pcap_close(pcap_);
    pcap_ = nullptr;
  }
}

void FakePointCloud::performPcap()
{
  int reset = 1;
  const u_char * p;
  struct pcap_pkthdr h;

  // Read the next packet from a pcap_t
  while ((p = pcap_next(pcap_, &h))) {
    {
      std::lock_guard<std::mutex> lock(mutex_stop_);
      if (stop_thread_) break;
    }

    // Wait packet time
    packetTimer(h.ts, reset);
    if (reset) reset = 0;

    // Send packet
    send(p);
  }
}

void FakePointCloud::send(const u_char * packet)
{
  u_char * p = const_cast<u_char *>(packet);

  p += sizeof(struct ether_header);
  const struct iphdr * ip = reinterpret_cast<const struct iphdr *>(p);

  if (ip->protocol == IPPROTO_UDP && !IN_MULTICAST(ntohl(ip->daddr))) {
    p += ((struct iphdr *)p)->ihl * 4;
    const struct udphdr * udp = (struct udphdr *)(p);
    p += sizeof(struct udphdr);

    LivoxEthPacket * eth_packet = reinterpret_cast<LivoxEthPacket *>(p);
    if (eth_packet->data_type != PointDataType::kImu) {
      std::lock_guard<std::mutex> lock(mutex_config_);
      eth_packet->err_code = status_code_;
    }

    // 2-byte length of payload including UDP header (8 bytes)
    uint16_t len = ntohs(udp->len) - sizeof(struct udphdr);
    std::vector<uint8_t> frame(p, p + len);

    // Start an synchronous send
    asip::udp::endpoint ep = asip::udp::endpoint(user_ip_, data_port_);
    socket_->send_to(as::buffer(frame), ep);
  }
}

void FakePointCloud::packetTimer(struct timeval t, int reset)
{
  static struct timeval si;
  struct timeval sd;
  struct timeval s;

  static struct timeval ti;
  struct timeval td;

  if (reset) {
    memcpy(&ti, &t, sizeof(struct timeval));
    gettimeofday(&si, NULL);
  }

  timersub(&t, &ti, &td);
  do {
    gettimeofday(&s, NULL);
    timersub(&s, &si, &sd);
  } while (timercmp(&sd, &td, <));
}
