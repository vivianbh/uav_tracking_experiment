import serial
import time
import siyicom.communication as com

#ser = serial.Serial("/dev/ttyUSB4", 115200) 
ser = com.Serial(port='/dev/ttyUSB2')
cn = 0

try:
    '''
    msg = b'\x03'
    msg_len = 1
    control_id = 1
    cmd_id = 12
    ser.send(msg, msg_len, control_id, cmd_id)
    time.sleep(0.2)
    bs = ser.read()

    msg = b''
    msg_len = 0
    control_id = 1
    cmd_id = 10
    ser.send(msg, msg_len, control_id, cmd_id)
    time.sleep(0.2)
    bs = ser.read()
    '''
    while True:
        if cn == 0:
            start = time.time()
        
        msg = b'\x04\x05'
        msg_len = 2
        control_id = 1
        cmd_id = 15
        ser.send(msg, msg_len, control_id, cmd_id)
        bs = ser.read()
        #print(bs)
        cn = 1
        if len(bs[2]) > 0:
            period = time.time() - start
            print('period ', period)
            print('freq ', 1/period)
            print('read ', bs)
            cn = 0
        #time.sleep(0.1)
except KeyboardInterrupt:
    print("\nProgram terminated by user.")