from siyicom import communication as comm
from siyicom import task
import siyicom.sy_tool as syt
import time

pt_comm = comm.cr_serial(port=syt.SY4EO_Serial)

pt_task = task.cr_task(config_file=syt.icd_file, conn_list=[syt.SY4EO_OUT, syt.SY4EO_IN],
                          icomm=pt_comm, freq=syt.EO_Frequency)
pt_task.start(bSend=True, bRecv=True)

counter=0
mode = True

def getInfo():
    # inTab read only
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

try:
    while True:
        getInfo()
        outTab=pt_task.get_outTable()
        if counter>=6:
            print(f"set cmd from [{syt.ID2Txt_cmdID[outTab[syt.c_cmd_id]]}] to [{syt.Cmd_type.PTAngle}]")
            newOutTable={}
            #newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.PTCenter]
            #newOutTable[syt.c_pose_center] = 1
            #pt_task.update_partialOutTable(newOutTable)
            
            if mode:
                #newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.PTInfo]
                pt_task.update_partialOutTable(newOutTable)
            if not mode:
                #newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.PTVelocity]
                #newOutTable[syt.c_rate_yaw] = 5
                #newOutTable[syt.c_rate_pitch] = 0
                #newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.PTAngle]
                #newOutTable[syt.c_ang_yaw] = 0
                #newOutTable[syt.c_ang_pitch] = -20
                #pt_task.update_partialOutTable(newOutTable)
                #newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.PTCenter]
                #newOutTable[syt.c_pose_center] = 1
                pass
               
            newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.ZoomAbsolute]
            newOutTable[syt.c_zoom_absolute_int] = 2
            newOutTable[syt.c_zoom_absolute_float] = 5

            #newOutTable[syt.c_cmd_id] = syt.cmd_id[syt.Cmd_type.ZoomManual]
            #newOutTable[syt.c_zoom_manual] = -1
            
            pt_task.update_partialOutTable(newOutTable)
        mode = not mode
        #mode = True
        time.sleep(0.5)
        counter+=1
        print(counter)
except KeyboardInterrupt:
    print("\nProgram terminated by user.")
    pt_task.stop()


'''
[pan velocity]
100% <-> 75 deg/s
 50% <-> 37.5 deg/s
 20% <-> 15 deg/s

[tilt velocity]
100% <-> 75 deg/s
 50% <-> 37.5 deg/s
 20% <-> 15 deg/s
'''
