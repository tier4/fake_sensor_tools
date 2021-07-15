# fake_sensor_tools

Simulate sensor outputs without real sensor.
The tools are used as RQt plugins.

:warning: The tools are generally used for checking diagnostic function.

## Instructions before starting

1. Git clone this repository.

```
git clone git@github.com:tier4/fake_sensor_tools.git
git checkout ros2
```

2. Install `socat`.

```
sudo apt install socat
```
3. Install dependencies

```
sudo apt install qtbase5-dev qttools5-dev-tools qt5-default libqwt-qt5-dev
```

4. Install dependencies using `rosdep`.

```
cd fake_sensor_tools
rosdep update
rosdep install -y --from-paths . --ignore-src --rosdistro $ROS_DISTRO
```

5. Build this workspace.

```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

6. Run RQt.

If roscore is not already running, you need to run roscore on one terminal window.

```
roscore
```

Then, run RQt on another terminal window.

```
source install/setup.bash
rqt --force-discover
```

# Tools

- [Fake IMU](fake_imu/README.md)
- [Fake GNSS](fake_gnss/README.md)
- [Fake Velodyne](fake_velodyne/README.md)
- [Fake Livox](fake_livox/README.md)
