from siyicom import communication as comm
from siyicom import task
import siyicom.sy_tool as syt
from tools import keyboard_listen as key

import rospy
import time
import argparse
import math

from tracking_uav_control.msg import EoCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import String

parser = argparse.ArgumentParser(prog="python siyi_control_hil.py")
parser.add_argument("-s","--serial_port", dest="port_number", default='2', help="serial_port_number")
args = parser.parse_args()
port_num = args.port_number

pt_comm = comm.cr_serial(port=syt.Serial+port_num)
pt_task = task.cr_task(config_file=syt.icd_file, conn_list=[syt.SY4EO_OUT, syt.SY4EO_IN],
                          icomm=pt_comm, freq=syt.EO_Frequency)
pt_task.start(bSend=True, bRecv=True)

# constant value
MAX_VEL = 75
MIN_VEL = 0
MAP_RATIO = 100/75
MAX_PAN = 135
MIN_PAN = -135
MAX_TILT = 90
MIN_TILT = -25
INIT_ZOOM = 1

# init value
mode = True
zoomFlag = False
counter = 0
cmd_pan=0
cmd_tilt=0
cmd_zoom=0
gimbal_pan=0
gimbal_tilt=0
trigger = ''
old_trigger = ''
zoom_trigger = ''
eo_state = EoCommand()
zoomKey = ['q', 'a', 'z']
CMD_ZOOM = 1

# msg from [GimbalControl] ros node
def callback(msg):
    global cmd_pan
    global cmd_tilt
    global cmd_zoom

    # msg units: [degree]
    cmd_pan = msg.pan_wv.data
    cmd_tilt = msg.tilt_wv.data
    cmd_zoom = msg.zoom.data

# gimbal angle from [Gazebo]
def callback1(state):
    global gimbal_pan, gimbal_tilt

    # msg units: [rad]
    gimbal_tilt = state.position[1] * (180/3.1415)
    gimbal_pan = state.position[2] * (180/3.1415)

# [User] trigger through rostopic
def callback2(msg):
    global trigger
    trigger = msg.data

def map_angle(pan, tilt):
    if pan > MAX_PAN:
        pan = MAX_PAN
    if pan < MIN_PAN:
        pan = MIN_PAN
    if tilt > MAX_TILT:
        tilt = MAX_TILT
    if tilt < MIN_TILT:
        tilt = MIN_TILT
    pan = round(pan, 1)
    tilt = round(tilt, 1)
    return pan, tilt

def map_velocity(rate):
    rate_perc = 0
    if abs(rate) > MAX_VEL:
        rate = math.copysign(MAX_VEL, rate)
    if abs(rate) < MIN_VEL:
        rate = math.copysign(MIN_VEL, rate)
    rate_perc = round(rate * MAP_RATIO)
    return rate_perc

def showCmdAng(cmd_pan, cmd_tilt):
    print('set angle pan: ', cmd_pan)
    print('set angle tilt: ', cmd_tilt)
    print('---------------------------------')

def showCmdVal(cmd_pan, cmd_tilt):
    print('set angular rate pan: ', cmd_pan)
    print('set angular rate tilt: ', cmd_tilt)
    print('---------------------------------')

def showCmdValRatio(cmd_pan, cmd_tilt):
    print('set velocity ratio pan: ', cmd_pan)
    print('set velocity ratio tilt: ', cmd_tilt)
    print('---------------------------------')

rospy.init_node('siyi_control', anonymous=True)
cmd_sub = rospy.Subscriber('/eo/cmd/angular_velocity', EoCommand, callback, queue_size=1)
gimbal_sub = rospy.Subscriber('/uav0/gimbal/joint_states', JointState, callback1, queue_size=1)
trigger_sub = rospy.Subscriber('/eo/trigger/i', String, callback2, queue_size=1)
state_pub = rospy.Publisher('/eo/state/o', EoCommand, queue_size=1)
rate = rospy.Rate(syt.EO_Frequency)

#listener = key.keyboard.Listener(on_press=key.on_press, on_release=key.on_release)
#listener.start()


def showEOInfo():
    inTab=pt_task.get_inTable()
    print('\n---------------------------------')
    print('eo zoom: ', inTab[syt.eo_zoom])
    print('eo zoom abs cmd: ', inTab[syt.eo_zoom_absolute_cmd])
    print('gimbal rate cmd: ', inTab[syt.gimbal_rate_cmd])
    print('gimbal center cmd: ', inTab[syt.gimbal_center_cmd])
    print('now angle pan: ', inTab[syt.gimbal_now_ang_yaw])
    print('now angle tilt: ', inTab[syt.gimbal_now_ang_pitch])
    print('now angle roll: ', inTab[syt.gimbal_now_ang_roll])
    print('now velocity pan: ', inTab[syt.gimbal_now_rate_yaw])
    print('now velocity tilt: ', inTab[syt.gimbal_now_rate_pitch])
    print('now velocity roll: ', inTab[syt.gimbal_now_rate_roll])
    print('---------------------------------')

    # publish eo state
    eo_state.pan_wv.data = inTab[syt.gimbal_now_rate_yaw]
    eo_state.tilt_wv.data = inTab[syt.gimbal_now_rate_pitch]
    eo_state.pan_ang.data = inTab[syt.gimbal_now_ang_yaw]
    eo_state.tilt_ang.data = inTab[syt.gimbal_now_ang_pitch]
    eo_state.zoom.data = inTab[syt.eo_zoom]
    state_pub.publish(eo_state)

while not rospy.is_shutdown():
    #inTab ls only
    inTab = pt_task.get_inTable()
    outTab = pt_task.get_outTable()

    showEOInfo()
    
    #trigger = key.usercmd
    if zoom_trigger != trigger:
        if trigger in zoomKey:
            zoom_trigger = trigger
            zoomFlag = True
    else:
        zoomFlag = False
    if old_trigger != trigger:
        if not trigger in zoomKey and trigger != 'p':
            old_trigger = trigger


    print('trigger: ', trigger)
    print('zoom_trigger: ', zoom_trigger, '  ', zoomFlag)
    print('old_trigger: ', old_trigger)
    print('cnt: ', counter, '\n')
    
    newOutTable={}
    if counter <= 20:
        #newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.PTInfo]
        pass
    if counter > 20:
        if mode:
            newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.PTInfo]
            if zoom_trigger == 'z' and zoomFlag == True:
                CMD_ZOOM = INIT_ZOOM
                newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.ZoomAbsolute]
                newOutTable[syt.c_zoom_absolute_int] = INIT_ZOOM
                newOutTable[syt.c_zoom_absolute_float] = 0
            if zoom_trigger == 'q' and zoomFlag == True:
                CMD_ZOOM += 1
                newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.ZoomAbsolute]
                newOutTable[syt.c_zoom_absolute_int] = round(CMD_ZOOM)
                newOutTable[syt.c_zoom_absolute_float] = 0
            if zoom_trigger == 'a' and zoomFlag == True:
                CMD_ZOOM -= 1
                newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.ZoomAbsolute]
                newOutTable[syt.c_zoom_absolute_int] = round(CMD_ZOOM)
                newOutTable[syt.c_zoom_absolute_float] = 0
            print('[Zoom] ', round(CMD_ZOOM))
        if not mode:
            if old_trigger == 'w':
                # angle cmd
                print('[Angle Control]')
                showCmdAng(gimbal_pan, gimbal_tilt)
                ang_p, ang_t = map_angle(gimbal_pan, gimbal_tilt)
                newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.PTAngle]
                newOutTable[syt.c_ang_yaw] = ang_p
                newOutTable[syt.c_ang_pitch] = ang_t
                showCmdAng(newOutTable[syt.c_ang_yaw], newOutTable[syt.c_ang_pitch])
            elif old_trigger == 's':
                # velocity cmd
                print('[Velocity Control]')
                showCmdVal(cmd_pan, -cmd_tilt)
                newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.PTVelocity]
                newOutTable[syt.c_rate_yaw] = map_velocity(cmd_pan)
                newOutTable[syt.c_rate_pitch] = map_velocity(-cmd_tilt)
                showCmdValRatio(newOutTable[syt.c_rate_yaw], newOutTable[syt.c_rate_pitch])
            elif old_trigger == 'c':
                print('[Angle Center]')
                ang_p, ang_t = map_angle(gimbal_pan, gimbal_tilt)
                newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.PTCenter]
                newOutTable[syt.c_pose_center] = 1 
            else:
                pass
    
    pt_task.update_partialOutTable(newOutTable)

    if old_trigger == 'o':
        pt_task.stop()
        print("\nProgram terminated by user.")
        break
  
    counter += 1
    mode = not mode
    rate.sleep()
    
    

