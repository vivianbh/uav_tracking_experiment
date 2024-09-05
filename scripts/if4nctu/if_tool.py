BYTEORDER='little'


IF4EO_OUT="0x82-OUT"
IF4EO_IN="0x02-IN"
EO4IF_OUT=IF4EO_IN+"-REV"
EO4IF_IN=IF4EO_OUT+"-REV"

icd_file="if4nctu/if4nctu.ini"
Serial = "/dev/ttyUSB"
SerialUAV = "/dev/ttyUAV"
SerialGCS = "/dev/ttyGCS"

EO_Frequency = 20 #HZ

class Resolution:
    r1280x720 = 0
    r1920x1080 = 2
    Default = r1280x720

Txt2ID_Resolution={
    "1280x720":Resolution.r1280x720,
    "1920x1080":Resolution.r1920x1080
}

ID2Txt_Resolution={
    Resolution.r1280x720:"1280x720",
    Resolution.r1920x1080:"1920x1080"
}

class Eo_mode:
    FrontView = 0
    Joystick = 1
    Manual = 2
    Auto = 3
    Default = FrontView

ID2Txt_eo_mode= {
    Eo_mode.FrontView: "FrontView(0)",           # 前視
    Eo_mode.Joystick: "Joystick(1)",               # 速度控制
    Eo_mode.Manual: "Manual(2)",             # 角度控制
    Eo_mode.Auto: "Auto(3)",                 # 視角鎖定
}
Txt2ID_eo_mode= {
    ID2Txt_eo_mode[Eo_mode.FrontView]:Eo_mode.FrontView,
    ID2Txt_eo_mode[Eo_mode.Joystick]:Eo_mode.Joystick,
    ID2Txt_eo_mode[Eo_mode.Manual]:Eo_mode.Manual,
    ID2Txt_eo_mode[Eo_mode.Auto]:Eo_mode.Auto,
}

FirmwarePX4 = "p"
FirmwareAMP = "a"

## PX4
class Flight_mode:
    Loiter = 0
    Offboard = 1
    Mission = 2
    Acro = 3
    Manual = 4
    Altitude = 5
    Other = 6
    Default = Loiter

ID2Txt_flight_mode= {
    Flight_mode.Loiter: "AUTO.LOITER",
    Flight_mode.Offboard: "OFFBOARD",
    Flight_mode.Mission: "AUTO.MISSION",
    Flight_mode.Acro: "ACRO",
    Flight_mode.Manual: "MANUAL",
    Flight_mode.Altitude: "ALTCTL",     # Altitude
    Flight_mode.Other: "Other"
}

Txt2ID_flight_mode= {
    ID2Txt_flight_mode[Flight_mode.Loiter]:Flight_mode.Loiter,
    ID2Txt_flight_mode[Flight_mode.Offboard]:Flight_mode.Offboard,
    ID2Txt_flight_mode[Flight_mode.Mission]:Flight_mode.Mission,
    ID2Txt_flight_mode[Flight_mode.Acro]:Flight_mode.Acro,
    ID2Txt_flight_mode[Flight_mode.Manual]:Flight_mode.Manual,
    ID2Txt_flight_mode[Flight_mode.Altitude]:Flight_mode.Altitude,
    ID2Txt_flight_mode[Flight_mode.Other]:Flight_mode.Other
}

## Ardupilot
class Flight_mode_apm:
    Stabilize = 0
    Altitude = 1
    Loiter = 2
    Land = 3
    Offboard = 4
    Mission = 5
    Other = 6
    Default = Stabilize

ID2Txt_flight_mode_apm= {
    Flight_mode_apm.Stabilize: "STABILIZE",
    Flight_mode_apm.Altitude: "ALT_HOLD",
    Flight_mode_apm.Loiter: "LOITER",
    Flight_mode_apm.Land: "LAND",
    Flight_mode_apm.Offboard: "GUIDED",
    Flight_mode_apm.Mission: "AUTO",
    Flight_mode_apm.Other: "Other"
}

Txt2ID_flight_mode_apm= {
    ID2Txt_flight_mode_apm[Flight_mode_apm.Stabilize]:Flight_mode_apm.Stabilize,
    ID2Txt_flight_mode_apm[Flight_mode_apm.Altitude]:Flight_mode_apm.Altitude,
    ID2Txt_flight_mode_apm[Flight_mode_apm.Loiter]:Flight_mode_apm.Loiter,
    ID2Txt_flight_mode_apm[Flight_mode_apm.Land]:Flight_mode_apm.Land,
    ID2Txt_flight_mode_apm[Flight_mode_apm.Offboard]:Flight_mode_apm.Offboard,
    ID2Txt_flight_mode_apm[Flight_mode_apm.Mission]:Flight_mode_apm.Mission,
    ID2Txt_flight_mode_apm[Flight_mode_apm.Other]:Flight_mode_apm.Other
}

class Cmd_type_uav:
    Standby = 0
    Off_Navgate = 1
    Default = Standby

ID2Txt_cmd_type_uav= {
    Cmd_type_uav.Standby: "Standby(0)",
    Cmd_type_uav.Off_Navgate: "Offboard Navgate(1)"
}

Txt2ID_cmd_type_uav= {
    ID2Txt_cmd_type_uav[Cmd_type_uav.Standby]:Cmd_type_uav.Standby,
    ID2Txt_cmd_type_uav[Cmd_type_uav.Off_Navgate]:Cmd_type_uav.Off_Navgate
}

class Cmd_type_algo:
    Lock = 0
    Track_p3d = 1
    Track_yolo_ukf = 2  
    Track_yolo_qp = 3     
    Default = Lock

ID2Txt_cmd_type_algo= {
    Cmd_type_algo.Lock: "Standby(0)",
    Cmd_type_algo.Track_p3d: "Track P3D(1)",
    Cmd_type_algo.Track_yolo_ukf: "Track Yolo+UKF(2)",
    Cmd_type_algo.Track_yolo_qp: "Track Yolo+QP(3)"
}

Txt2ID_cmd_type_algo= {
    ID2Txt_cmd_type_algo[Cmd_type_algo.Lock]:Cmd_type_algo.Lock,
    ID2Txt_cmd_type_algo[Cmd_type_algo.Track_p3d]:Cmd_type_algo.Track_p3d,
    ID2Txt_cmd_type_algo[Cmd_type_algo.Track_yolo_ukf]:Cmd_type_algo.Track_yolo_ukf,
    ID2Txt_cmd_type_algo[Cmd_type_algo.Track_yolo_qp]:Cmd_type_algo.Track_yolo_qp,
}

class Cmd_type_depth:
    GT = 0
    Cal = 1
    Default = GT

ID2Txt_cmd_type_depth= {
    Cmd_type_depth.GT: "Ground truth(0)",
    Cmd_type_depth.Cal: "Calculation(1)"
}

Txt2ID_cmd_type_depth= {
    ID2Txt_cmd_type_depth[Cmd_type_depth.GT]:Cmd_type_depth.GT,
    ID2Txt_cmd_type_depth[Cmd_type_depth.Cal]:Cmd_type_depth.Cal
}

c_eo_mode =  'c_eo_mode_0_u'# 設定環架模式
c_eo_joystick_pan = 'c_eo_joystick_pan_0_s' 
c_eo_joystick_tilt = 'c_eo_joystick_tilt_0_s' 
c_eo_angle_pan =  'c_eo_angle_pan_2_s'# 設定環架角度PAN
c_eo_angle_tilt =  'c_eo_angle_tilt_2_s'# 設定環架角度TILT
c_eo_zoom =  'c_eo_zoom_1_u'# 設定鏡頭倍率
c_user_command_uav = 'c_user_command_uav_0_u'
c_user_command_algo = 'c_user_command_algo_0_u'
c_user_command_depth = 'c_user_command_depth_0_u'

resolution =  'resolution_0_u'# 回報影像解析度
eo_mode =  'eo_mode_0_u'# 回報環架模式
eo_now_angular_velocity_pan = 'eo_now_angular_velocity_pan_2_s'# 回報目前環架角速度PAN
eo_now_angular_velocity_tilt = 'eo_now_angular_velocity_tilt_2_s'# 回報目前環架角速度TILT
eo_now_angle_pan =  'eo_now_angle_pan_2_s'# 回報目前環架角度PAN
eo_now_angle_tilt =  'eo_now_angle_tilt_2_s'# 回報目前環架角度TILT
eo_set_angular_velocity_pan =  'eo_set_angular_velocity_pan_2_s'# 回報環架角速度PAN
eo_set_angular_velocity_tilt =  'eo_set_angular_velocity_tilt_2_s'# 回報環架角速度TILT
eo_set_angle_pan =  'eo_set_angle_pan_2_s'# 回報環架角度PAN
eo_set_angle_tilt =  'eo_set_angle_tilt_2_s'# 回報環架角度TILT
eo_zoom =  'eo_zoom_1_u'# 回報鏡頭倍率
uav_mavlink_connect = 'uav_mavlink_connect_0_u'
uav_arm_state = 'uav_arm_state_0_u'
uav_flight_mode = 'uav_flight_mode_0_u'
uav_navigate_point = 'uav_navigate_point_0_u'
uav_nav_point_x = 'uav_nav_point_x_4_s'
uav_nav_point_y = 'uav_nav_point_y_4_s'
uav_nav_point_z = 'uav_nav_point_z_4_s'
user_curr_uav = 'user_curr_uav_0_u'          
user_curr_algo = 'user_curr_algo_0_u'
user_curr_depth = 'user_curr_depth_0_u'