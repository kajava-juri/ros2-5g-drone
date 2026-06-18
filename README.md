# Setup

Install PX4 autopilot software here and follow instructions

https://github.com/PX4/PX4-Autopilot

### Camera simulation

https://docs.px4.io/main/en/simulation/index#camera-simulation

### PX4 tool to simulate and run SITL

``` sh
make px4_sitl gz_x500_lidar_2d_walls
```

#### running multiple drone instances:
``` sh
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,2" PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=walls ./build/px4_sitl_default/bin/px4 -i 1
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,-2" PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=walls ./build/px4_sitl_default/bin/px4 -i 2
```

Then run:
``` sh
MicroXRCEAgent udp4 -p 8888
```

for more information check https://docs.px4.io/main/en/simulation/

### Bridging lidar topic to ros2

``` sh
ros2 run ros_gz_bridge parameter_bridge /world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
```
