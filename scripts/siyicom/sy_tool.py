BYTEORDER='little'

SY4EO_OUT="0x01-OUT"
SY4EO_IN="0x02-IN"

icd_file="siyicom/siyi.ini"
SY4EO_Serial = "/dev/ttyUSB2"
EO4SY_Serial = "/dev/ttyUSB2"
Serial = "/dev/ttySiyi"

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

class Cmd_type:
    ZoomManual = 'EO Zoom Manual'
    ZoomAbsolute = 'EO Zoom Absolute'
    PTVelocity = 'PT Velocity Mode'
    PTCenter = 'PT Center'
    PTAngle = 'PT Angle Mode'
    PTInfo = 'PT Info'

cmd_id = {
    Cmd_type.ZoomManual:'0x05',
    Cmd_type.ZoomAbsolute:'0x0F',
    Cmd_type.PTVelocity:'0x07',
    Cmd_type.PTCenter:'0x08',
    Cmd_type.PTAngle:'0x0E',
    Cmd_type.PTInfo:'0x0D'
}

ID2Txt_cmdID = {
    cmd_id[Cmd_type.ZoomManual]: Cmd_type.ZoomManual,           
    cmd_id[Cmd_type.ZoomAbsolute]: Cmd_type.ZoomAbsolute,
    cmd_id[Cmd_type.PTVelocity]: Cmd_type.PTVelocity,
    cmd_id[Cmd_type.PTCenter]: Cmd_type.PTCenter,
    cmd_id[Cmd_type.PTAngle]: Cmd_type.PTAngle,
    cmd_id[Cmd_type.PTInfo]: Cmd_type.PTInfo
}

eo_zoom = 'eo_zoom_1_u'
eo_zoom_absolute_cmd = 'eo_zoom_absolute_cmd_0_u'
gimbal_rate_cmd = 'gimbal_rate_cmd_0_u'
gimbal_center_cmd = 'gimbal_center_cmd_0_u'
gimbal_now_ang_yaw = 'gimbal_now_ang_yaw_1_s'
gimbal_now_ang_pitch = 'gimbal_now_ang_pitch_1_s'
gimbal_now_ang_roll = 'gimbal_now_ang_roll_1_s'
gimbal_now_rate_yaw = 'gimbal_now_rate_yaw_1_s'
gimbal_now_rate_pitch = 'gimbal_now_rate_pitch_1_s'
gimbal_now_rate_roll = 'gimbal_now_rate_roll_1_s'

c_cmd_id = 'c_cmd_id_0_t'
c_zoom_manual = 'c_zoom_manual_0_u'
c_zoom_absolute_int = 'c_zoom_absolute_int_0_u'
c_zoom_absolute_float = 'c_zoom_absolute_float_0_u'
c_rate_yaw = 'c_rate_yaw_0_s'
c_rate_pitch = 'c_rate_pitch_0_s'
c_pose_center = 'c_pose_center_0_u'
c_ang_yaw = 'c_ang_yaw_1_s'
c_ang_pitch = 'c_ang_pitch_1_s'
c_pose_info = 'c_pose_info_0_u'