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
 * @file lidar_status.h
 * @brief List of LiDAR status
 */

#pragma once

#include <stdint.h>

enum class LidarStatus : int8_t {
  ChecksumError,
  DebugOutput,
  ReturnCode,
  LidarState,
  RainFogSuppressionSwitch,
  InitializationProgress,
  TempStatus,
  VoltStatus,
  MotorStatus,
  DirtyWarn,
  FirmwareStatus,
  PpsStatus,
  DeviceStatus,
  FanStatus,
  SelfHeating,
  PtpStatus,
  TimeSyncStatus,
  SystemStatus,
};

struct LidarValue
{
  LidarValue() {}
  LidarValue(bool t, int v) : trigger_system_status(t), value(v) {}

  bool trigger_system_status = false;
  int value = 0;
};
