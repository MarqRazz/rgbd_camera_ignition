# rgbd_camera_ignition
Simple package that showcases use of rgbd camera in ignition.

## Requirements
[Ignition Fortress](https://ignitionrobotics.org/docs/fortress/install_ubuntu)

[Ignition Gazebo](https://ignitionrobotics.org/api/gazebo/6.1/install.html)

[Ignition Sensors](https://ignitionrobotics.org/api/sensors/6.0/installation.html)

## Build
```
export COLCON_WS=~/workspace/ws_rgbd
mkdir -p $COLCON_WS/src
cd $COLCON_WS
vcs import src --skip-existing --input src/rgbd_camera_ignition/rgbd_camera_ignition.repos
rosdep install --ignore-src --from-paths src -y -r 
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Launch
```
ros2 launch rgbd_camera_ignition rgbd_ignition.launch.py launch_rviz:=true
```