from siyicom import communication as comm
from siyicom import task
import siyicom.sy_tool as syt
import if4nctu.if_tool as ift

import rospy
import argparse
import math
from std_msgs.msg import String
from tracking_experiment.msg import TelemGCS

parser = argparse.ArgumentParser(prog="python siyi_control_hil.py")
#parser.add_argument("-s","--serial_port", dest="port_number", default='2', help="serial_port_number")
#args = parser.parse_args()
#port_num = args.port_number

pt_comm = comm.cr_serial(port=syt.Serial)
pt_task = task.cr_task(config_file=syt.icd_file, conn_list=[syt.SY4EO_OUT, syt.SY4EO_IN],
                          icomm=pt_comm, freq=syt.EO_Frequency)
pt_task.start(bSend=True, bRecv=True)

# constant value
MAX_PAN = 135
MIN_PAN = -135
MAX_TILT = 90
MIN_TILT = -25
INIT_ZOOM = 1

# init value
counter = 0
cntup=0
cntdn=0
flag = 0
gimbal_pan=0
gimbal_tilt=0
eo_zoom=1
CMD_ZOOM = 1

class GCS4GIMBAL:
    nodeCmd = 0
    eoCmd = 0
    mvPan = 0
    mvTilt = 0
    eoZoom = 0

upLink = GCS4GIMBAL()

# [User] Gimbal Command through rostopic
def callback(msg):
    global upLink
    upLink.nodeCmd = msg.user_cmd.data
    upLink.eoCmd = msg.cmd_eo_mode.data
    upLink.mvPan = msg.move_pan.data
    upLink.mvTilt = msg.move_tilt.data
    upLink.eoZoom = msg.eo_zoom.data

rospy.init_node('siyi_control', anonymous=True)
cmd_sub = rospy.Subscriber('/exp/telem/gcs_cmd', TelemGCS, callback, queue_size=1)
rate = rospy.Rate(syt.EO_Frequency)

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

def showCmdAng(cmd_pan, cmd_tilt, cmd_zoom):
    print('set angle pan: ', cmd_pan)
    print('set angle tilt: ', cmd_tilt)
    print('set angle zoom: ', cmd_zoom)
    print('---------------------------------')

def showEOInfo():
    inTab=pt_task.get_inTable()
    print('\n---------------------------------')
    print('eo zoom: ', inTab[syt.eo_zoom])
    print('eo zoom abs cmd: ', inTab[syt.eo_zoom_absolute_cmd])
    print('gimbal center cmd: ', inTab[syt.gimbal_center_cmd])
    print('now angle pan: ', inTab[syt.gimbal_now_ang_yaw])
    print('now angle tilt: ', inTab[syt.gimbal_now_ang_pitch])
    print('---------------------------------')

try:
    while not rospy.is_shutdown():
        #inTab ls only
        inTab = pt_task.get_inTable()
        outTab = pt_task.get_outTable()

        showEOInfo()
        
        newOutTable={}
        if counter > 5:
            if upLink.eoCmd == ift.Eo_mode.Joystick:
                print('[Angle Control]')
                if upLink.mvPan > 0:
                    gimbal_pan += 2
                if upLink.mvPan < 0:
                    gimbal_pan -= 2
                if upLink.mvTilt > 0:
                    gimbal_tilt += 2
                if upLink.mvTilt < 0:
                    gimbal_tilt -= 2
                ang_p, ang_t = map_angle(gimbal_pan, gimbal_tilt)
                newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.PTAngle]
                newOutTable[syt.c_ang_yaw] = ang_p
                newOutTable[syt.c_ang_pitch] = -ang_t
                showCmdAng(newOutTable[syt.c_ang_yaw], newOutTable[syt.c_ang_pitch], eo_zoom)
            elif upLink.eoCmd == ift.Eo_mode.FrontView:
                print('[Angle Center]')
                gimbal_pan = 0
                gimbal_tilt = 0
                newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.PTCenter]
                newOutTable[syt.c_pose_center] = 1 
                showCmdAng(gimbal_pan, gimbal_tilt, eo_zoom)
            else:
                pass
            '''
            if upLink.eoCmd == ift.Eo_mode.Zoom:
                print('[Absolute Zoom]')
                if upLink.eoZoom == 1:
                    flag = 10
                if upLink.eoZoom == 2:
                    flag = 20
                
                if cntup < 1 and flag == 10:
                    print('yexs')
                    eo_zoom += 1
                    cntup += 1
                    newOutTable[syt.c_zoom_manual] = 1
                    if eo_zoom > 6:
                        eo_zoom = 6
                elif cntdn < 1 and flag == 20:
                    print('noo')
                    eo_zoom += -1
                    cntdn += 1
                    newOutTable[syt.c_zoom_manual] = -1
                    if eo_zoom < 1:
                        eo_zoom = 1
                else:
                    print('yoopk')
                    newOutTable[syt.c_zoom_manual] = 0
                
                if cntup >= 1:
                    cntup = 0
                    flag = 0
                if cntdn >= 1:
                    cntdn = 0
                    flag = 0
                newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.ZoomManual]
                showCmdAng(gimbal_pan, gimbal_tilt, eo_zoom)
                print(newOutTable[syt.c_zoom_manual], ' ', upLink.eoZoom)
                '''
        
        pt_task.update_partialOutTable(newOutTable)
        counter += 1
        rate.sleep()
except KeyboardInterrupt:
    print("\nProgram terminated by user.")
    pt_task.stop()