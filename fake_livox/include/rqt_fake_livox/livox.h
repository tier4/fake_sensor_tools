//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef FAKE_LIVOX_INCLUDE_RQT_FAKE_LIVOX_LIVOX_H_
#define FAKE_LIVOX_INCLUDE_RQT_FAKE_LIVOX_LIVOX_H_

static const uint32_t kMaxCommandBufferSize = 1536;  //!< @brief The maximum buffer size of command
const uint8_t kSdkProtocolSof = 0xAA;                //!< @brief Starting Byte, Fixed to be0xAA
const uint32_t kSdkPacketCrcSize = 4;                // crc32

/**
 * @brief Protocol Version
 */
typedef enum { kSdkVerNone, kSdkVer0, kSdkVer1 } SdkVersion;

/**
 * @brief Enum that represents the command type.
 */
typedef enum {
  kCommandTypeCmd = 0,  //!< @brief command type, which requires response from the receiver.
  kCommandTypeAck = 1,  //!< @brief acknowledge type, which is the response of command type.
  kCommandTypeMsg = 2   //!< @brief message type, which is sent at a specified frequency.
} CommandType;

/**
 * @brief Enum that represents the index of command set.
 */
typedef enum {
  kCommandSetGeneral = 0,  //!< @brief general command set.
  kCommandSetLidar,        //!< @brief LiDAR command set.
  kCommandSetHub           //!< @brief hub command set.
} CommandSet;

/**
 * @brief Enum that represents the command id.
 */
typedef enum {
  kCommandIDGeneralBroadcast = 0,                 //!< @brief broadcast command.
  kCommandIDGeneralHandshake = 1,                 //!< @brief handshake command.
  kCommandIDGeneralDeviceInfo = 2,                //!< @brief query the information of device.
  kCommandIDGeneralHeartbeat = 3,                 //!< @brief heartbeat command.
  kCommandIDGeneralControlSample = 4,             //!< @brief enable or disable the sampling.
  kCommandIDGeneralCoordinateSystem = 5,          //!< @brief change the coordinate of point cloud data.
  kCommandIDGeneralDisconnect = 6,                //!< @brief disconnect the device.
  kCommandIDGeneralPushAbnormalState = 7,         //!< @brief a message from a device to notify exceptions.
  kCommandIDGeneralConfigureStaticDynamicIp = 8,  //!< @brief set the IP of the a device.
  kCommandIDGeneralGetDeviceIpInformation = 9,    //!< @brief get the IP of the a device.
  kCommandIDGeneralRebootDevice = 0x0a,           //!< @brief reboot a device.
  kCommandIDGeneralSetDeviceParam = 0x0b,         //!< @brief Set device's parameters.
  kCommandIDGeneralGetDeviceParam = 0x0c,         //!< @brief Get device's parameters.
  kCommandIDGeneralResetDeviceParam = 0x0d,       //!< @brief Reset device's all parameters.
  kCommandIDGeneralCommandCount                   //!< @brief Don't add command id after kCommandIDGeneralCommandCount.
} GeneralCommandID;

/**
 * @brief Device type.
 */
typedef enum {
  kDeviceTypeHub = 0,          //!< @brief Livox Hub.
  kDeviceTypeLidarMid40 = 1,   //!< @brief Mid-40.
  kDeviceTypeLidarTele = 2,    //!< @brief Tele.
  kDeviceTypeLidarHorizon = 3  //!< @brief Horizon.
} DeviceType;

/**
 * @brief Return code.
 */
typedef enum {
  kReturnCodeSuccess = 0,  //!< @brief Success.
  kReturnCodeFail = 1      //!< @brief Fail.
} ReturnCode;

/**
 * @brief Lidar state.
 */
typedef enum {
  kLidarStateInit = 0,         //!< @brief Initialization state.
  kLidarStateNormal = 1,       //!< @brief Normal work state.
  kLidarStatePowerSaving = 2,  //!< @brief Power-saving state.
  kLidarStateStandBy = 3,      //!< @brief Standby state.
  kLidarStateError = 4,        //!< @brief Error state
  kLidarStateUnknown = 5       //!< @brief Unknown state.
} LidarState;

/**
 * @brief Coordinate System
 */
typedef enum {
  kStopSampling = 0,  //!< @brief Stop Sampling
  kStartSampling      //!< @brief Start Sampling
} Sampling;

/**
 * @brief Coordinate System
 */
typedef enum {
  kCoordinateCartesian = 0,  //!< @brief Cartesian Coordinate
  kCoordinateSpherical       //!< @brief Spherical Coordinate
} CoordinateType;

#define SamplingToString(X) \
  (((X) == kStopSampling) ? "Stop Sampling" : ((X) == kStartSampling) ? "Start Sampling" : "?")

#define CoordinateTypeToString(X)                         \
  (((X) == kCoordinateCartesian) ? "Cartesian Coordinate" \
                                 : ((X) == kCoordinateSpherical) ? "Spherical Coordinate" : "?")

/**
 * @brief Enum that represents the command id.
 */
typedef enum {
  kCommandIDLidarSetMode = 0,                    //!< @brief set the working mode and sub working mode of a LiDAR.
  kCommandIDLidarSetExtrinsicParameter = 1,      //!< @brief set the parameters of a LiDAR.
  kCommandIDLidarGetExtrinsicParameter = 2,      //!< @brief get the parameters of a LiDAR.
  kCommandIDLidarControlRainFogSuppression = 3,  //!< @brief enable or disable the rain/fog suppression of a LiDAR.
  kCommandIDLidarControlFan = 4,                 //!< @brief turn on\off fan of a LiDAR.
  kCommandIDLidarGetFanState = 5,                //!< @brief get fan state of a LiDAR.
  kCommandIDLidarSetPointCloudReturnMode = 6,    //!< @brief set point cloud return mode of a LiDAR.
  kCommandIDLidarGetPointCloudReturnMode = 7,    //!< @brief get point cloud return mode of a LiDAR.
  kCommandIDLidarSetImuPushFrequency = 8,        //!< @brief set IMU push frequency of a LiDAR.
  kCommandIDLidarGetImuPushFrequency = 9,        //!< @brief get IMU push frequency of a LiDAR.
  kCommandIDLidarSetSyncTime = 0x0a,             //!< @brief set synchronization time of a LiDAR.
  kCommandIDLidarCommandCount                    //!< @brief Don't add command id after kCommandIDLidarCommandCount
} LidarCommandID;

/**
 * @brief Point cloud return mode.
 */
typedef enum {
  kFirstReturn,      //!< @brief First single return mode.
  kStrongestReturn,  //!< @brief Strongest single return mode.
  kDualReturn        //!< @brief Dual return mode.
} PointCloudReturnMode;

/**
 * @brief IMU push frequency.
 */
typedef enum {
  kImuFreq0Hz,    //!< @brief IMU push closed.
  kImuFreq200Hz,  //!< @brief IMU push frequency 200Hz.
} ImuFreq;

#define PointCloudReturnModeToString(X) \
  (((X) == kFirstReturn)                \
     ? "Single Return First"            \
     : ((X) == kStrongestReturn) ? "Single Return Strongest" : ((X) == kDualReturn) ? "Dual Return" : "?")

#define ImuFreqToString(X) (((X) == kImuFreq0Hz) ? "0Hz(Close IMU push)" : ((X) == kImuFreq200Hz) ? "200Hz" : "?")

#pragma pack(1)

/**
 * @brief The Protocol Frame.
 */
typedef struct
{
  uint8_t sof;            //!< @brief Starting Byte, Fixed to be0xAA
  uint8_t version;        //!< @brief Protocol Version, 1 for The Current Version
  uint16_t length;        //!< @brief Lengthof The Frame, Max Value:1400
  uint8_t packet_type;    //!< @brief Command Type
  uint16_t seq_num;       //!< @brief Frame Sequence Number
  uint16_t preamble_crc;  //!< @brief Frame Header Checksum
  uint8_t cmd_set;        //!< @brief CMD Set
  uint8_t cmd_id;         //!< @brief CMD ID
  uint8_t data[1];        //!< @brief PayloadData
} SdkPacket;

#define kBroadcastCodeSize 16

/**
 * @brief The information of broadcast device.
 */
typedef struct
{
  char broadcast_code[kBroadcastCodeSize];  //!< @brief Device broadcast code.
  uint8_t dev_type;                         //!< @brief Device type.
  uint16_t reserved;                        //!< @brief Reserved.
} BroadcastDeviceInfo;

/**
 * @brief The response body for the command.
 */
typedef struct
{
  uint8_t ret_code;  //!< @brief Return Code.
} GenericResponse;

/**
 * @brief The request body for the command of handshake.
 */
typedef struct
{
  uint32_t ip_addr;      //!< @brief IP address of the device.
  uint16_t data_port;    //!< @brief UDP port of the data connection.
  uint16_t cmd_port;     //!< @brief UDP port of the command connection.
  uint16_t sensor_port;  //!< @brief UDP port of the sensor connection.
} HandshakeRequest;

/**
 * @brief The response body of querying device information.
 */
typedef struct
{
  uint8_t ret_code;             //!< @brief Return code.
  uint8_t firmware_version[4];  //!< @brief Firmware version.
} DeviceInformationResponse;

/**
 * @brief LiDAR error code.
 */
typedef struct
{
  uint32_t temp_status : 2;    //!< @brief 0: Normal State. 1: High or Low. 2: Extremely High or Extremely Low.
  uint32_t volt_status : 2;    //!< @brief 0: Voltage in Normal State. 1: High. 2: Extremely High.
  uint32_t motor_status : 2;   //!< @brief 0: Normal State. 1: Warning State. 2: Error State, Unable to Work.
  uint32_t dirty_warn : 2;     //!< @brief 0: Not Dirty or Blocked. 1: Dirty or Blocked.
  uint32_t firmware_err : 1;   //!< @brief 0: Firmware is OK. 1: Firmware is Abnormal, Need to be Upgraded.
  uint32_t pps_status : 1;     //!< @brief 0: No PPS Signal. 1: PPS Signal is OK.
  uint32_t device_status : 1;  //!< @brief 0: Normal. 1: Warning for Approaching the End of Service Life.
  uint32_t fan_status : 1;     //!< @brief 0: Fan in Normal State. 1: Fan in Warning State.
  uint32_t self_heating : 1;   //!< @brief 0: Normal. 1: Low Temperature Self Heating On.
  uint32_t ptp_status : 1;     //!< @brief 0: No 1588 Signal. 1: 1588 Signal is OK.
  /** 0: System does not start time synchronization.
   * 1: Using PTP 1588 synchronization.
   * 2: Using GPS synchronization.
   * 3: Using PPS synchronization.
   * 4: System time synchronization is abnormal.(The highest priority synchronization signal is abnormal)
  */
  uint32_t time_sync_status : 3;
  uint32_t rsvd : 13;          //!< @brief Reserved.
  uint32_t system_status : 2;  //!< @brief 0: Normal. 1: Warning. 2: Error.
} LidarErrorCode;

/**
 * @brief Device error message.
 */
typedef union {
  uint32_t error_code;              //!< @brief Error code.
  LidarErrorCode lidar_error_code;  //!< @brief Lidar error code.
} ErrorMessage;

/**
 * @brief Information of LiDAR work state.
 */
typedef union {
  uint32_t progress;         //!< @brief LiDAR work state switching progress.
  ErrorMessage status_code;  //!< @brief LiDAR work state status code.
} StatusUnion;

/**
 * @brief The body of heartbeat response.
 */
typedef struct
{
  uint8_t ret_code;         //!< @brief Return code.
  uint8_t state;            //!< @brief Working state.
  uint8_t feature;          //!< @brief LiDAR feature.
  StatusUnion error_union;  //!< @brief LiDAR work state.
} HeartbeatResponse;

/**
 * @brief The request body for the command of Start/Stop Sampling.
 */
typedef struct
{
  uint8_t sample_ctrl;  //!< @brief 0x00: Stop Sampling 0x01: Start Sampling
} ControlSampleRequest;

/**
 * @brief The request body for the command of Change Coordinate System.
 */
typedef struct
{
  uint8_t coordinate_type;  //!< @brief 0x00: Cartesian Coordinate 0x01: Spherical Coordinate
} CoordinateSystemRequest;

/**
 * @brief The request body for the command of Set LiDAR Return Mode.
 */
typedef struct
{
  uint8_t mode;  //!< @brief Return Mode
} SetPointCloudReturnModeRequest;

/**
 * @brief The request body for the command of Change Coordinate System.
 */
typedef struct
{
  uint8_t frequency;  //!< @brief IMU Data Push Frequency
} SetImuPushFrequencyRequest;

#pragma pack()

#endif  // FAKE_LIVOX_INCLUDE_RQT_FAKE_LIVOX_LIVOX_H_
