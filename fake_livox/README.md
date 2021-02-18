# Fake Livox for Autoware

## Instructions

1. Enable Fake Livox plugin.

- Choose from the menu: `Plugins`->`Fake Sensor Tools`->`Fake Livox`
  ![window](docs/readme_01.png)

2. Launch sensor driver.

```
roslaunch livox_ros_driver livox_lidar.launch
```

3. Start communication with sensor driver.

- Specify any broadcast code in `Broadcast code` field.
- If you wish to transmit point cloud data by using .pcap file,
  - Specify any .pcap file on `PCAP` page.
  - Select packets to transmit if you wish to transmit specific packets.
    (All UDP packets will be transmitted if none is selected.)
- Then, turn on `Communication` switch to start communication with Livox ROS Driver.
  ![window](docs/readme_02.png)

Then transmission will be started.

---

## `General` page

| Field            | Description                   | Notes                                                          |
| ---------------- | ----------------------------- | -------------------------------------------------------------- |
| `Broadcast code` | Broadcast code                |                                                                |
| `Communication`  | Start/Stop communication      |                                                                |
| `Checksum error` | Enable/Disable checksum error | The checksum value will be set to `??` if enabled.             |
| `Debug output`   | Enable/Disable debug output   | Transmission data can be seen in a terminal window if enabled. |

---

## `Heartbeat` page

![window](docs/readme_03.png)

| Field                         | Description                 | Notes |
| ----------------------------- | --------------------------- | ----- |
| `Return Code`                 | Return Code                 |       |
| `LiDAR State`                 | LiDAR State                 |       |
| `Rain/Fog Suppression Switch` | Rain/Fog Suppression Switch |       |

### If LiDAR State is not `Initializing`

Status Code can be set using the settings of "Push Abnormal Status".

### If LiDAR State is `Initializing`

![window](docs/readme_04.png)

| Field                     | Description             | Notes |
| ------------------------- | ----------------------- | ----- |
| `Initialization Progress` | Initialization Progress |       |

---

## `Push Abnormal Status` page

![window](docs/readme_05.png)

| Field              | Description                       | Notes                                                                             |
| ------------------ | --------------------------------- | --------------------------------------------------------------------------------- |
| `temp_status`      | Temperature Status                |                                                                                   |
| `volt_status`      | Voltage Status of Internal Module |                                                                                   |
| `motor_status`     | Motor Status                      |                                                                                   |
| `dirty_warn`       | Dirty/Blocked Status              |                                                                                   |
| `firmware_status`  | Firmware Status                   |                                                                                   |
| `pps_status`       | PPS Status                        |                                                                                   |
| `device_status`    | Device SFantatus                  |                                                                                   |
| `fan_status`       | Motor Status                      |                                                                                   |
| `self_heating`     | Self Heating Status               |                                                                                   |
| `ptp_status`       | PTP Status                        |                                                                                   |
| `time_sync_status` | Time synchronization Status       |                                                                                   |
| `system_status`    | System Status                     | This code will be set automatically according to the settings of above errors.\*1 |

\*1 Also, please refer to the following document.

- [Livox SDK Communication Protocol](https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol)

When the `system_status` is changed from 'Normal' to `Warning` or `Error`, the `Push Abnormal Status` message will be immediately transmitted at 10Hz.

## `PCAP` page

Point cloud data can be transmitted by using .pcap file.

![window](docs/readme_06.png)

| Field         | Description                                                   | Notes                                                                                                                                    |
| ------------- | ------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `Path`        | Path of .pcap file                                            |                                                                                                                                          |
| `...`         | Button to select .pcap file                                   |                                                                                                                                          |
| `Read`        | Read .pcap file and show packets information in `UDP Packets` |                                                                                                                                          |
| `Loop`        | Enable/Disable loop playback                                  |                                                                                                                                          |
| `UDP Packets` | UDP packets information                                       | You can apply packet filter. Check a checkbox or double click a table row.<br>(All UDP packets will be transmitted if none is selected.) |

### To save you time...

When you specify any .pcap file by using `...` or press `Read`, the tool will start to read .pcap file.

![window](docs/readme_07.png)

This procedure is to display UDP packets information in the list,
so you can cancel in the middle of this procedure
if you wish to send all packets or when a specific packet that you wish to send is shown in the list.

Multicast packets are filtered, not shown in the list, and not forwarded to the port.
