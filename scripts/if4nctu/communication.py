from abc import ABCMeta, abstractmethod
from if4nctu.if_tool import BYTEORDER
import serial
import socket
import logging

logger = logger = logging.getLogger('INTF')

class IComm(metaclass=ABCMeta):
    name = None

    def set_name(self, n):
        self.name = n


    @abstractmethod
    def get_Connstr(self):
        pass

    @abstractmethod
    def read(self):
        pass

    @abstractmethod
    def send(self, msg):
        pass  # return data

    @abstractmethod
    def close(self):
        pass

class RD_SATATUS:
    Success = 0
    Header_err = 1
    StartCode_err = 2
    Packed_err = 3
    Data_err = 4
    Checksum_err = 5


class BaseComm(IComm):
    START_CODE = '$DAT'.encode('ascii')
    END_CODE = (0x0a).to_bytes(1, byteorder=BYTEORDER, signed=False)  # 'LF'
    def __init__(self):
        self.connStr=""

    def get_Connstr(self):
        return self.connStr

    def read(self):
        pass

    def send(self, msg):
        pass  # return data

    def checksum_calculator(self, header):  # checksum
        checksum = 0
        for b in header:
            checksum += b
        return checksum & 0xff

    def genPacket(self, msg: bytes, msgID):
        msg_ID_byte = msgID.to_bytes(1, byteorder=BYTEORDER, signed=False)
        start_code = self.START_CODE
        # 1+len(msg)=len(msg_ID)+len(msg)
        control_byte = (1 + len(msg)).to_bytes(1, byteorder=BYTEORDER, signed=False)
        # msg_ID
        data = msg
        checksum = self.checksum_calculator(control_byte + msg_ID_byte + data)
        checksum = checksum.to_bytes(1, byteorder=BYTEORDER, signed=False)
        end_code = self.END_CODE
        header = start_code + control_byte + msg_ID_byte
        packet = header + data + checksum + end_code
        logger.debug(f'header: {header}, checksum: {checksum}')
        return packet

    def parsePacket(self, packed):
        suc = False
        msg_ID, data = 0, bytes([])

        if len(packed) < 6:
            logger.debug(f'Socket_header len < 6.')
        else:
            # 4(start_code)+1(control_byte)+1(msg_ID)
            header_len = 6
            header = packed[0:header_len]
            #print('packet: ', packed)

            if header[0:4] == self.START_CODE:
                control_byte = header[4:5]
                control_byte_val = int.from_bytes(control_byte, byteorder=BYTEORDER, signed=False)
                # control_byte - 1= control_byte-len(msg_ID_byte)
                data_len = control_byte_val - 1
                msg_ID_byte = header[5:6]
                msg_ID = int.from_bytes(msg_ID_byte, byteorder=BYTEORDER, signed=False)

                if data_len >= 0:
                    # data_len + 1 + 1= data_len + len(checksum) + len(end_code)
                    tail_len = data_len + 1 + 1

                    if (header_len + tail_len) != len(packed):
                        logger.debug(f'Socket_packed len should be {header_len + tail_len} but {len(packed)}.')
                    else:
                        tail = packed[header_len:header_len + tail_len]
                        data = tail[0:data_len]
                        checksum = tail[data_len:data_len + 1]
                        checksum = int.from_bytes(checksum, byteorder=BYTEORDER, signed=False)
                        end_code = tail[data_len + 1:data_len + 2]
                        # 計算資料欄位checksum
                        do_checksum = self.checksum_calculator(control_byte + msg_ID_byte + data)
                        logger.debug(f'Socket_do_checksum: {do_checksum}')
                        # CheckSum比對錯誤則回傳0,掠過此比資料

                        if do_checksum == checksum:
                            suc = True
                            msg_ID, data = msg_ID, data
                        else:
                            logger.debug(f'checksum error.: {do_checksum},{checksum}')
                else:
                    logger.debug('data len is 0.')
            else:
                logger.debug('START_CODE errs.')
        return msg_ID, data

    def close(self):
        pass

class Serial(BaseComm):
    ser = None
    seekerRcvheader = b''
    seekerRcvBuff = b''
    state = 0
    count = 0

    def __init__(self,port='/dev/ttyUSB0', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=1):
        logger.info('Serial INIT')
        try:
            self.connStr=f"port={port}, baudrate={baudrate}, bytesize={bytesize}, parity={parity}, stopbits={stopbits}, timeout={timeout}"
            self.ser = self.openSerial(port=port, baudrate=baudrate, bytesize=bytesize, parity=parity, stopbits=stopbits, timeout=timeout)
        except Exception as e:
            logger.error(self.get_Connstr())
            logger.error('open serial fail.')

    def send(self, msg,msg_ID):
        # print("socket send")
        packed = self.genPacket(msg,msg_ID)
        try:
            self.ser.write(packed)
        except Exception as e:
            logger.error(self.get_Connstr())
            logger.error(e)
    
    def read(self):
        logger.debug(f'serial_read')
        packed = bytearray(0)
        uavPacketReceived = 0
        Rxbyte = self.ser.read(1)
        #print('read: ', Rxbyte )

        if self.state == 0:
            if Rxbyte == '$'.encode('ascii'):
                self.seekerRcvheader += Rxbyte
                self.state = 1
            else:
                self.state = 0
                self.seekerRcvheader = b''
                self.seekerRcvBuff = b''
        elif self.state == 1:
            if Rxbyte == 'D'.encode('ascii'):
                self.seekerRcvheader += Rxbyte
                self.state = 2
            else:
                self.state = 0
                self.seekerRcvheader = b''
                self.seekerRcvBuff = b''
        elif self.state == 2:
            if Rxbyte == 'A'.encode('ascii'):
                self.seekerRcvheader += Rxbyte
                self.state = 3
            else:
                self.state = 0
                self.seekerRcvheader = b''
                self.seekerRcvBuff = b''
        elif self.state == 3:
            if Rxbyte == 'T'.encode('ascii'):
                self.seekerRcvheader += Rxbyte
                self.state = 4
            else:
                self.state = 0
                self.seekerRcvheader = b''
                self.seekerRcvBuff = b''
        elif self.state == 4:
            self.seekerRcvheader += Rxbyte
            self.state = 5
        elif self.state == 5:
            msgID = Rxbyte
            if msgID == b'\x02' or msgID == b'\x82':
                self.seekerRcvheader += Rxbyte
                self.count = 0
                self.state = 6
            else:
                self.state = 0
                self.seekerRcvheader = b''
                self.seekerRcvBuff = b''
        elif self.state == 6:
            data_len = self.seekerRcvheader[4] - 1
            if self.count < data_len:
                self.seekerRcvBuff += Rxbyte
                self.count += 1
                if self.count >= data_len:
                    self.state = 7
            else:
                self.state = 0
                self.seekerRcvheader = b''
                self.seekerRcvBuff = b''
        elif self.state == 7:
            self.seekerRcvBuff += Rxbyte
            self.state = 8
        elif self.state == 8:
            endcode = Rxbyte
            if endcode == b'\n':
                self.seekerRcvBuff += Rxbyte
                packed = self.seekerRcvheader + self.seekerRcvBuff
                uavPacketReceived += 1
            else:
                uavPacketReceived = 0
            #print('uavPacketReceived ', uavPacketReceived)
            self.state = 0
            self.seekerRcvheader = b''
            self.seekerRcvBuff = b''
            #print('packet: ', packed)
        #print('state: ', self.state)
        #print('uavPacketReceived ', uavPacketReceived)
        return self.parsePacket(packed)

    def openSerial(self,port='/dev/ttyTHS0',baudrate=115200,bytesize=8,parity='N',stopbits=1,timeout=1):
        return serial.Serial(port=port,baudrate=baudrate,bytesize=bytesize,parity=parity,stopbits=stopbits,timeout=timeout)

    def close(self):
        pass

def cr_serial(port='/dev/ttyUSB0', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=1):
    ser=Serial(port=port, baudrate=baudrate, bytesize=bytesize, parity=parity, stopbits=stopbits, timeout=timeout)
    return ser

