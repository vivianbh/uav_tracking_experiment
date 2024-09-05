## Environment Setup

Install mavros
```bash
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
```

Install python3 library
```bash
sudo apt install python3-pip
pip3 install pyserial pynput rospkg
```

Downlaod the package and build the dependencies
```bash
git clone git@github.com:vivianbh/tracking_experiment_outdoor.git
cd ..
catk_make
```

## Pre-usage
### Change Port Permission
On jetson orin nx
```bash
sudo chmod 777 /dev/ttyMav /dev/ttyUAV /dev/ttySiyi
```

## Usage

### Custom communication protocol
1. Gimbal Camera: 同中科院計畫中的尋標器硬體迴路模擬
2. Telemtry: define in `tracking_experiment_outdoor/doc/`

### Telemetry (GCS <-> UAV)
On UAV
```bash
$ roscd tracking_experiment/scripts
$ python3 telem_uav.py -s u
```
  | args | description | default | option |
  | -------- | -------- | -------- | -------- |
  | `-s`     | port number     | `0` (/dev/ttyUSB0) | number, `u`, `g`  |
  | `-f`     | firmware type      | `p` (px4) |  `p`, `a`  |

  - publish command received from GCS by rostopic `/exp/telem/gcs_cmd`
  - send FCU state and offboard waypoint info to GCS telemetry
    - not include camera info yet 
  
On GCS
```bash
$ roscd tracking_experiment/scripts
$ python3 telem_gcs.py -s 0
```
  | args | description | default | option |
  | -------- | -------- | -------- | -------- |
  | `-s`     | port number (/dev/ttyUSBx)      | `0` (/dev/ttyUSB0) | number  |
  | `-f`     | firmware type      | `p` (px4) |  `p`, `a`  |

  - key control definition
      | key |  Object  |  function | 
      | -------- | -------- | -------- |
      | `r`     | uav  |  stanby or exit offboard mode | 
      | `t`     | uav  |  offboard start   | 
      | `x`     | gimbal  |  go center      |
      | `w`     | gimbal  |  tilt up      |
      | `s`     | gimbal  |  tilt down      |
      | `d`     | gimbal  |  pan right      |
      | `a`     | gimbal  |  pan left      |

### UAV Offboard Mode with Position Control
Start Mavros 
```bash
# for Fixed-wing
$ roslaunch tracking_experiment mavros_px4.launch

# for rotor
$ roslaunch tracking_experiment mavros_apm.launch
```
  - set connection information for mavros in launch file
  ```launch
<!-- port name & baudrate (same as the setting in FCU) -->
  <arg name="fcu_url" default="/dev/ttyMav:230400" />
<!-- uav id given in FCU -->
<arg name="tgt_system" default="6" />
  ```
  - check if mavros is ready
  ```bash
  $ rostopic echo /uav0/mavros/state
  ```
Modify parameters in yaml file
```bash
$ roscd tracking_experiment/config
$ vim params_waypoints.yaml
```
  - 4 waypoints
  - 1 angle difference with north pole
    - clockwise: positive
    - range: 0-359
    - units: degree
  - 1 distance threshold for triggering next waypoint

Start offboard node
```bash
$ roslaunch tracking_experiment navigation.launch
```
  - select the firmware which FUC use
  ```launch
  <!-- system of fcu: apm or px4 -->
<arg name="fcu_sys" default="px4" />
  ```
  - Default State: wait for GCS command to enter offboard mode and do waypoints tracking
  - functions trigger by rostopic `/exp/telem/gcs_cmd`
  - publish current navigate waypoint by rostopic `/exp/offboard/waypoints`

### SIYI Camera
Stream by RTSP
1. Display
```bash
$ cd tracking_experiment_outdoor/scripts/test_stream/
$ python3 camera_rtsp_display.py
```
2. ROS images
```bash
$ cd tracking_experiment_outdoor/scripts/test_stream/
$ python3 camera_rtsp_ros.py
```
  - publish to rostopic `/exp/camera/raw`

Gimbal Control by UART
1. HITL
  ```bash
  $ cd tracking_experiment_outdoor/scripts
  $ python3 siyi_control_hil.py
  ```
  - Default Status: gimbal on center mode
  - functions trigger by rostopic `/eo/trigger/i`
  - publish current state by rostopic `/eo/state/o`
    - zoom
        | key | function | 
        | -------- | -------- |
        | `z`     | zoom to 1x      | 
        | `q`     | zoom +1x      | 
        | `a`     | zoom -1x      | 
    - gimbal
        | key | function | 
        | -------- | -------- |
        | `w`     | angle control      | 
        | `s`     | angular velocity control      | 
        | `c`     | center      |
        | `o`     | terminate program      |
2. GCS Remote Control
  ```bash
  $ cd tracking_experiment_outdoor/scripts
  $ python3 siyi_control_remote.py
  ```
  - Default Status: gimbal on center mode
  - functions trigger by rostopic `/exp/telem/gcs_cmd`

### Tracking Mission
Modify parameters used in IBVS controller
```bash
$ roscd tracking_experiment/config
$ vim params_ibvs.yaml
```
  - 3 for controller
    - control gain
    - max. rotation rate
    - min. rotation rate
  - 2 for dnn
    - class
    - confidence
  - 3 for uav
    - vehicle type
    - focal length
    - geometry of gimbal based on uav

Start tracking
```bash
$ roslaunch tracking_experiment tracking_estimate.launch
```
  - Functions trigger by rostopic `/uav0/tracking/function_select`
  - custom message definition in `tracking_experiment_outdoor/msg/FunctionSwitch.msg`
    ```text
    std_msgs/Bool doTrack        # switch between manual control and auto, default:0 (manual)
    std_msgs/Int16 trackMode     # 0:Track(p3d), 64:Yolo+UKF, 128:Yolo+UKF+QP, default:0 (p3d)
    std_msgs/Bool depthType      # switch between GT and measurement, default:0 (GT)
    ```

### Record Rosbag
Record rosbag every 15 minus in `tracking_experiment_outdoor/rosbag/exp` directory
```bash
$ roslaunch tracking_experiment rosbag.launch
```
