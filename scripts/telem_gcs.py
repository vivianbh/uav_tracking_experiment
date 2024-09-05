from if4nctu import communication as comm
from if4nctu import task
import if4nctu.if_tool as ift
from tools import keyboard_listen as key
import argparse
import time

parser = argparse.ArgumentParser(prog="python telem_gcs.py")
parser.add_argument("-s","--serial_port", dest="port_number", default=0, help="serial_port_number")
parser.add_argument("-f","--firmware", dest="firmware_type", default="p", help="firmware_type_first_letter: p or a")
args = parser.parse_args()
port_num = args.port_number
firmware = args.firmware_type

if4eo_comm = comm.cr_serial(port=ift.Serial+port_num)
if4eo_task = task.cr_task(config_file=ift.icd_file, conn_list=[ift.IF4EO_OUT, ift.IF4EO_IN],
                          icomm=if4eo_comm, freq=ift.EO_Frequency)
if4eo_task.start(bSend=True, bRecv=True)

listener = key.keyboard.Listener(on_press=key.on_press, on_release=key.on_release)
listener.start()

cnt = 0
trigger=''
old_trigger=''
class User_cmd:
    cmd_eo = ift.Eo_mode.Default
    cmd_node_uav = ift.Cmd_type_uav.Default
keyEoMove = ['a', 'd', 'w', 's']
keyEoCenter = ['x']

class JoyStick:
    cntPan_p = 0
    cntPan_n = 0
    cntTilt_p = 0
    cntTilt_n = 0

stick = JoyStick()

def showEOInfo():
    inTab=if4eo_task.get_inTable()
    eoMode=''
    resolution=''
    mavlink=False
    armed=False
    uavMode=''

    # Map id to text
    if inTab[ift.resolution]==ift.Resolution.r1280x720:
        resolution = ift.ID2Txt_Resolution[inTab[ift.resolution]]
    if inTab[ift.resolution]==ift.Resolution.r1920x1080:
        resolution = ift.ID2Txt_Resolution[inTab[ift.resolution]]
    
    if inTab[ift.eo_mode]==ift.Eo_mode.FrontView:
        eoMode = ift.ID2Txt_eo_mode[inTab[ift.eo_mode]]
    if inTab[ift.eo_mode]==ift.Eo_mode.Joystick:
        eoMode = ift.ID2Txt_eo_mode[inTab[ift.eo_mode]]
    if inTab[ift.eo_mode]==ift.Eo_mode.Manual:
        eoMode = ift.ID2Txt_eo_mode[inTab[ift.eo_mode]]
    if inTab[ift.eo_mode]==ift.Eo_mode.Auto:
        eoMode = ift.ID2Txt_eo_mode[inTab[ift.eo_mode]]

    if inTab[ift.uav_mavlink_connect]==1:
        mavlink=True
    if inTab[ift.uav_arm_state]==1:
        armed=True
    
    # flight mode
    if inTab[ift.uav_flight_mode] in range(0, 7):
        if firmware == ift.FirmwarePX4:
            uavMode = ift.ID2Txt_flight_mode[inTab[ift.uav_flight_mode]]
        if firmware == ift.FirmwareAMP:
            uavMode = ift.ID2Txt_flight_mode_apm[inTab[ift.uav_flight_mode]]

    print('\n\n---------------------------------\n---------------------------------')
    print('Resolution: ', resolution)
    print('Eo Mode: ', eoMode)
    print('now Zoom: ', inTab[ift.eo_zoom])
    print('now Velocity Pan: ', round(inTab[ift.eo_now_angular_velocity_pan],2))
    print('now Velocity Tilt: ', round(inTab[ift.eo_now_angular_velocity_tilt],2))
    print('now Angle Pan: ', round(inTab[ift.eo_now_angle_pan],2))
    print('now Angle Tilt: ', round(inTab[ift.eo_now_angle_tilt],2))
    print('set velocity pan: ', round(inTab[ift.eo_set_angular_velocity_pan],2))
    print('set velocity tilt: ', round(inTab[ift.eo_set_angular_velocity_tilt],2))
    print('set angle pan: ', round(inTab[ift.eo_set_angle_pan],2))
    print('set angle tilt: ', round(inTab[ift.eo_set_angle_tilt],2))
    print('---------------------------------')
    print('Mavlink: ', mavlink)
    print('Armed: ', armed)
    print('Flight Mode: ', uavMode)
    print('naviagte point: ', inTab[ift.uav_navigate_point])
    print('naviagte X: ', round(inTab[ift.uav_nav_point_x],2))
    print('naviagte Y: ', round(inTab[ift.uav_nav_point_y],2))
    print('naviagte Z: ', round(inTab[ift.uav_nav_point_z],2))
    print('---------------------------------')

try:
    while True:
        inTab=if4eo_task.get_inTable()
        outTab=if4eo_task.get_outTable()
        showEOInfo()
        trigger = key.usercmd
        eo_trigger = key.usercmd
        key.usercmd = ''

        if old_trigger != trigger and trigger != '':
            old_trigger = trigger

        print('key insert: ', key.usercmd, ' ', trigger, ' ', old_trigger)
        newOutTable={}

        '''
        [Offboard]: position command
        '''
        if old_trigger == 't':
            newOutTable[ift.c_user_command_uav] = ift.Cmd_type_uav.Off_Navgate
            User_cmd.cmd_node_uav = ift.Cmd_type_uav.Off_Navgate
        if old_trigger == 'r':
            newOutTable[ift.c_user_command_uav] = ift.Cmd_type_uav.Standby
            User_cmd.cmd_node_uav = ift.Cmd_type_uav.Standby
        '''
        [Gimbal]: angle command
        '''
        if old_trigger in keyEoMove:
            newOutTable[ift.c_eo_mode] = ift.Eo_mode.Joystick
            User_cmd.cmd_eo = ift.Eo_mode.Joystick
        if old_trigger in keyEoCenter:
            newOutTable[ift.c_eo_mode] = ift.Eo_mode.FrontView
            User_cmd.cmd_eo = ift.Eo_mode.FrontView

        if User_cmd.cmd_eo == ift.Eo_mode.Joystick:
            if eo_trigger == 'a': #left
                newOutTable[ift.c_eo_joystick_pan] = -1
            elif eo_trigger == 'd': #right
                newOutTable[ift.c_eo_joystick_pan] = 1
            elif eo_trigger == 'w': #up
                newOutTable[ift.c_eo_joystick_tilt] = 1
            elif eo_trigger == 's': #down
                newOutTable[ift.c_eo_joystick_tilt] = -1
            else:
                newOutTable[ift.c_eo_joystick_pan] = 0
                newOutTable[ift.c_eo_joystick_tilt] = 0
        
        if User_cmd.cmd_eo == ift.Eo_mode.FrontView:
            newOutTable[ift.c_eo_angle_pan] = 0
            newOutTable[ift.c_eo_angle_tilt] = 0

        if4eo_task.update_partialOutTable(newOutTable)
        print("EO State: ", ift.ID2Txt_eo_mode[User_cmd.cmd_eo])
        print("NODE UAV: ", ift.ID2Txt_cmd_type_uav[User_cmd.cmd_node_uav])
        print('---------------------------------\n---------------------------------')

        time.sleep(1/ift.EO_Frequency)

except KeyboardInterrupt:
    if4eo_task.stop()
    print("\n\nProgram Terminated by User.\n")
    

