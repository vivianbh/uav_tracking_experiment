from if4nctu import communication as comm
from if4nctu import task
import if4nctu.if_tool as ift

import rospy
from mavros_msgs.msg import State
from tracking_experiment.msg import TelemGCS
from tracking_experiment.msg import TelemUAV
import math
import argparse

parser = argparse.ArgumentParser(prog="python telem_uav.py")
parser.add_argument("-s","--serial_port", dest="port_number", default=0, help="serial_port_number: g or u or number")
parser.add_argument("-f","--firmware", dest="firmware_type", default="p", help="firmware_type_first_letter: p or a")
args = parser.parse_args()
port_num = ift.Serial+args.port_number
firmware = args.firmware_type
if args.port_number == 'g':
    port_num = ift.SerialGCS
if args.port_number == 'u':
    port_num = ift.SerialUAV

eo4if_comm = comm.cr_serial(port=port_num)
eo4if_task = task.cr_task(config_file=ift.icd_file, conn_list=[ift.EO4IF_OUT, ift.EO4IF_IN],
                          icomm=eo4if_comm, freq=ift.EO_Frequency)
eo4if_task.start(bSend=True, bRecv=True)

trigger=''
gsc_cmd = TelemGCS()
uav_data = TelemUAV()

class MavrosState:
    connected = False
    armed = False
    guided = False
    mode = ift.Flight_mode.Default

mav = MavrosState()

def mav_cb(msg):
    global mav_state
    mav_state = msg
    mav.connected = bool(mav_state.connected)
    mav.armed = bool(mav_state.armed)
    mav.guided = bool(mav_state.guided)
    mav.mode = mav_state.mode

def nav_cb(msg):
    global uav_data
    uav_data = msg


def showEOInfo():
    inTab=eo4if_task.get_inTable()
    eoMode=''
    #resolution=''
    cmdType=''

    # Map id to text
    if inTab[ift.c_eo_mode]==ift.Eo_mode.FrontView:
        eoMode = ift.ID2Txt_eo_mode[inTab[ift.c_eo_mode]]
    if inTab[ift.c_eo_mode]==ift.Eo_mode.Joystick:
        eoMode = ift.ID2Txt_eo_mode[inTab[ift.c_eo_mode]]
    if inTab[ift.c_eo_mode]==ift.Eo_mode.Manual:
        eoMode = ift.ID2Txt_eo_mode[inTab[ift.c_eo_mode]]
    if inTab[ift.c_eo_mode]==ift.Eo_mode.Auto:
        eoMode = ift.ID2Txt_eo_mode[inTab[ift.c_eo_mode]]

    if inTab[ift.c_user_command_uav]==ift.Cmd_type_uav.Standby:
        cmdType = ift.ID2Txt_cmd_type_uav[inTab[ift.c_user_command_uav]]
    if inTab[ift.c_user_command_uav]==ift.Cmd_type_uav.Off_Navgate:
        cmdType = ift.ID2Txt_cmd_type_uav[inTab[ift.c_user_command_uav]]

    print('\n---------------------------------')
    #print('Resolution: ', resolution)
    print('Eo Mode: ', eoMode)
    print('set Angle Pan: ', inTab[ift.c_eo_joystick_pan])
    print('set Angle Tilt: ', inTab[ift.c_eo_joystick_tilt])
    print('set Angle Pan: ', inTab[ift.c_eo_angle_pan])
    print('set Angle Tilt: ', inTab[ift.c_eo_angle_tilt])
    print('uav command: ', cmdType)
    print('---------------------------------')

rospy.init_node('telem_uav', anonymous=True)
cmd_pub = rospy.Publisher('/exp/telem/gcs_cmd', TelemGCS, queue_size=1)
mav_sub = rospy.Subscriber('/uav0/mavros/state', State, mav_cb, queue_size=1)
nav_sub = rospy.Subscriber('/exp/offboard/waypoints', TelemUAV, nav_cb, queue_size=1)
rate = rospy.Rate(ift.EO_Frequency)

while not rospy.is_shutdown():
    #inTab ls only
    inTab=eo4if_task.get_inTable()
    outTab=eo4if_task.get_outTable()

    showEOInfo()

    '''
    Send [Vihecle] Data to [GCS]
    '''
    newOutTable={}
    newOutTable[ift.eo_now_angle_pan] = 0
    newOutTable[ift.eo_now_angle_tilt] = 0
    newOutTable[ift.eo_zoom] = 1
    # FCU state
    newOutTable[ift.uav_mavlink_connect] = mav.connected
    newOutTable[ift.uav_arm_state] = mav.armed
    # flight mode
    if firmware == ift.FirmwarePX4:
        if(mav.mode == ift.ID2Txt_flight_mode[ift.Flight_mode.Loiter]):
            newOutTable[ift.uav_flight_mode] = ift.Flight_mode.Loiter
        elif(mav.mode == ift.ID2Txt_flight_mode[ift.Flight_mode.Offboard]):
            newOutTable[ift.uav_flight_mode] = ift.Flight_mode.Offboard
        elif(mav.mode == ift.ID2Txt_flight_mode[ift.Flight_mode.Mission]):
            newOutTable[ift.uav_flight_mode] = ift.Flight_mode.Mission
        elif(mav.mode == ift.ID2Txt_flight_mode[ift.Flight_mode.Acro]):
            newOutTable[ift.uav_flight_mode] = ift.Flight_mode.Acro
        elif(mav.mode == ift.ID2Txt_flight_mode[ift.Flight_mode.Manual]):
            newOutTable[ift.uav_flight_mode] = ift.Flight_mode.Manual
        elif(mav.mode == ift.ID2Txt_flight_mode[ift.Flight_mode.Altitude]):
            newOutTable[ift.uav_flight_mode] = ift.Flight_mode.Altitude
        else:
            newOutTable[ift.uav_flight_mode] = ift.Flight_mode.Other
    if firmware == ift.FirmwareAMP:
        if(mav.mode == ift.ID2Txt_flight_mode_apm[ift.Flight_mode_apm.Stabilize]):
            newOutTable[ift.uav_flight_mode] = ift.Flight_mode_apm.Stabilize
        elif(mav.mode == ift.ID2Txt_flight_mode_apm[ift.Flight_mode_apm.Altitude]):
            newOutTable[ift.uav_flight_mode] = ift.Flight_mode_apm.Altitude
        elif(mav.mode == ift.ID2Txt_flight_mode_apm[ift.Flight_mode_apm.Loiter]):
            newOutTable[ift.uav_flight_mode] = ift.Flight_mode_apm.Loiter
        elif(mav.mode == ift.ID2Txt_flight_mode_apm[ift.Flight_mode_apm.Land]):
            newOutTable[ift.uav_flight_mode] = ift.Flight_mode_apm.Land
        elif(mav.mode == ift.ID2Txt_flight_mode_apm[ift.Flight_mode_apm.Offboard]):
            newOutTable[ift.uav_flight_mode] = ift.Flight_mode_apm.Offboard
        elif(mav.mode == ift.ID2Txt_flight_mode_apm[ift.Flight_mode_apm.Mission]):
            newOutTable[ift.uav_flight_mode] = ift.Flight_mode_apm.Mission
        else:
            newOutTable[ift.uav_flight_mode] = ift.Flight_mode_apm.Other

    # current waypoint
    if uav_data.nav_waypoint_num.data != math.nan:
        pass
    if uav_data.nav_waypoint_num.data == math.nan:
        uav_data.nav_waypoint_num.data = 0

    newOutTable[ift.uav_navigate_point] = uav_data.nav_waypoint_num.data
    eo4if_task.update_partialOutTable(newOutTable)

    '''
    Acquire [GCS] Data to [Vihecle] Offboard Node
    '''
    if inTab[ift.c_user_command_uav] == ift.Cmd_type_uav.Standby or inTab[ift.c_user_command_uav] == ift.Cmd_type_uav.Off_Navgate:
        gsc_cmd.user_cmd.data = inTab[ift.c_user_command_uav]
        #cmd_pub.publish(gsc_cmd)
    '''
    Acquire [GCS] Data to [Gimbal] Direct Control Node
    '''
    if inTab[ift.c_eo_mode] == ift.Eo_mode.FrontView:
        gsc_cmd.cmd_eo_mode.data = inTab[ift.c_eo_mode]
        gsc_cmd.move_pan.data = inTab[ift.c_eo_angle_pan]
        gsc_cmd.move_tilt.data = inTab[ift.c_eo_angle_tilt]

    if inTab[ift.c_eo_mode] == ift.Eo_mode.Joystick:
        gsc_cmd.cmd_eo_mode.data = inTab[ift.c_eo_mode]
        gsc_cmd.move_pan.data = inTab[ift.c_eo_joystick_pan]
        gsc_cmd.move_tilt.data = inTab[ift.c_eo_joystick_tilt]
    cmd_pub.publish(gsc_cmd)

    rate.sleep()
    

