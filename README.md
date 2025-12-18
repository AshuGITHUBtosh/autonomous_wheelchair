# Setting up Velodyne VLP-16 LiDAR and Pixhawk (MAVROS) with ROS 2 Humble

---

## System Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Velodyne VLP-16 LiDAR
- Pixhawk Flight Controller
- Ethernet and USB connectivity

---

## Setting up Velodyne VLP-16 LiDAR

### Install Velodyne ROS package and driver

```
sudo apt update
sudo apt install ros-humble-velodyne

 ```

## Configure network interface for Velodyne
```
sudo ip addr add 192.168.7.11/24 dev enp2s0
```
## Run the Velodyne driver
```
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
ros2 run topic_tools relay /odom_rf2o /odom
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link velodyne
os2 run slam_toolbox sync_slam_toolbox_node --ros-args -p scan_topic:=/scan -p use_odom:=false -p base_frame:=base_link -p odom_frame:=odom -p map_frame:=map -p resolution:=0.05 -p mode:=mapping
```
## Setting up MAVROS and Pixhawk
### Install QGroundControl
```
cd ~
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
./QGroundControl.AppImag
```

## Install MAVROS and dependencies
```
sudo apt install ros-humble-mavros ros-humble-mavros-extras
sudo apt install geographiclib-tools
sudo geographiclib-get-geoids egm96-5
```
Flash Pixhawk firmware
- Connect Pixhawk to the laptop using a USB B-type cable
- Open QGroundControl
- Go to Firmware Setup
- Flash ArduPilot firmware
- Disable all pre-arm and safety checks (for testing purposes only)

  ### Run MAVROS
  ```
  ros2 run mavros mavros_node --ros-args \
  -p fcu_url:="serial:///dev/ttyACM0:115200" \
  -p tf.send:=true \
  -p tf.frame_id:="odom" \
  -p tf.child_frame_id:="base_link" \
  -p odom.frame_id:="odom" \
  -p odom.child_frame_id:="base_link"
  ```
### Set MAVLink stream rate
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
"{stream_id: 0, message_rate: 10, on_off: true}"

### Arm Pixhawk (testing only)
```
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

```
Verify MAVROS topics
ros2 topic echo /mavros/imu/data
```
## Running SLAM Toolbox
```
ros2 launch slam_toolbox online_async_launch.py

```






