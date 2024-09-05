from abc import ABCMeta, abstractmethod
from siyicom.sy_tool import BYTEORDER
import serial
import logging

logger = logging.getLogger('SIYI')

crc16_tab = [
        0x0,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
        0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
        0x1231,0x210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
        0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
        0x2462,0x3443,0x420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
        0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
        0x3653,0x2672,0x1611,0x630,0x76d7,0x66f6,0x5695,0x46b4,
        0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
        0x48c4,0x58e5,0x6886,0x78a7,0x840,0x1861,0x2802,0x3823,
        0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
        0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0xa50,0x3a33,0x2a12,
        0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
        0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0xc60,0x1c41,
        0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
        0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0xe70,
        0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
        0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
        0x1080,0xa1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
        0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
        0x2b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
        0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
        0x34e2,0x24c3,0x14a0,0x481,0x7466,0x6447,0x5424,0x4405,
        0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
        0x26d3,0x36f2,0x691,0x16b0,0x6657,0x7676,0x4615,0x5634,
        0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
        0x5844,0x4865,0x7806,0x6827,0x18c0,0x8e1,0x3882,0x28a3,
        0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
        0x4a75,0x5a54,0x6a37,0x7a16,0xaf1,0x1ad0,0x2ab3,0x3a92,
        0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
        0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0xcc1,
        0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
        0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0xed1,0x1ef0]

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


class BaseComm(IComm):
    # (0x55, 0x66): 'Uf'
    START_CODE = 'Uf'.encode('ascii')
    def __init__(self):
        self.connStr=""

    def get_Connstr(self):
        return self.connStr

    def read(self):
        pass

    def send(self, msg):
        pass  # return data
    
    def checkcrc16_calculator(self, msg, msg_len):  # check CRC16
        crc = 0
        old_crc = 0
        index = 0
        while index != msg_len:
            temp = (old_crc>>8)&0xff
            oldcrc16 = crc16_tab[msg[index]^temp]
            crc = (old_crc<<8)^oldcrc16
            if len(hex(crc)) > 6:
                old_crc = hex(crc)
                old_crc = old_crc[len(old_crc)-4:]
                old_crc = int(old_crc, 16)
            else:
                old_crc = crc
            index = index +1
        
        result = old_crc
        return result

    def genPacket(self, msg: bytes, msg_len, control_id, cmd_id):
        control_byte = control_id.to_bytes(1, byteorder=BYTEORDER, signed=False)
        start_code = self.START_CODE
        data_len_byte = (msg_len).to_bytes(2, byteorder=BYTEORDER, signed=False)
        seq_byte = b'\x00\x00'
        #if cmd_id == 15:
        #    seq_byte = b'\x01\x00'
        cmd_id_byte = (cmd_id).to_bytes(1, byteorder=BYTEORDER, signed=False)
        #print(cmd_id_byte)
        data = msg
        checkcrc = self.checkcrc16_calculator(start_code + control_byte + data_len_byte + seq_byte + cmd_id_byte + data, 8+msg_len)
        checkcrc = checkcrc.to_bytes(2, byteorder=BYTEORDER, signed=False)
        header = start_code + control_byte + data_len_byte + seq_byte + cmd_id_byte
        packet = header + data + checkcrc
        logger.debug(f'header: {header}, checkcrc: {checkcrc}')
        #print(packet)
        return packet

    def parsePacket(self, packed):
        suc = False
        ctr_ID, cmd_ID, data = 0, 0, bytes([])

        if len(packed) <= 8:
            logger.debug(f'Socket_header len < 8.')
        else:
            # 2(start_code)+1(control_byte)+2(data_len)+2(seq)+1(cmd_id)
            header_len = 8
            header = packed[0:header_len]
            #print('packet: ', packed)
            
            if header[0:2] == self.START_CODE:
                control_byte = header[2:3]
                control_id = int.from_bytes(control_byte, byteorder=BYTEORDER, signed=False)
                data_len_byte = header[3:5]
                data_len = int.from_bytes(data_len_byte, byteorder=BYTEORDER, signed=False)
                seq_byte = header[5:7]
                cmd_id_byte = header[7:8]
                cmd_id = int.from_bytes(cmd_id_byte, byteorder=BYTEORDER, signed=False)

                if data_len >= 0:
                    # data_len + 2 = data_len + len(crc16)
                    tail_len = data_len + 2

                    if (header_len + tail_len) != len(packed):
                        logger.debug(f'Socket_packed len should be {header_len + tail_len} but {len(packed)}.')
                    else:
                        tail = packed[header_len:header_len + tail_len]
                        data = tail[0:data_len]
                        checkcrc = tail[data_len:data_len + 2]
                        checkcrc = checkcrc[0].to_bytes(1, byteorder=BYTEORDER, signed=False)+ checkcrc[1].to_bytes(1, byteorder=BYTEORDER, signed=False)
                        #end_code = tail[data_len + 1:data_len + 2]
                        # 計算資料欄位checkcrc16
                        do_checkcrc = self.checkcrc16_calculator(packed[0:header_len + data_len], 8+data_len)
                        do_checkcrc = do_checkcrc.to_bytes(2, byteorder=BYTEORDER, signed=False)
                        logger.debug(f'Socket_do_checkcrc: {do_checkcrc}')
                        # CheckCRC16比對錯誤則回傳0,掠過此比資料

                        if do_checkcrc == checkcrc:
                            suc = True
                            ctr_ID, cmd_ID, data = control_id, cmd_id, data
                            #print("Data received!")
                        else:
                            logger.debug(f'checksum error.: {do_checkcrc},{checkcrc}')
                else:
                    logger.debug('data len is 0.')
            else:
                logger.debug('START_CODE errs.')
        return ctr_ID, cmd_ID, data

    def close(self):
        pass

class Serial(BaseComm):
    ser = None
    seekerRcvheader = b''
    seekerRcvBuff = b''
    state = 0
    count = 0

    def __init__(self,port='/dev/ttyUSB2', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=1):
        logger.info('Serial INIT')
        try:
            self.connStr=f"port={port}, baudrate={baudrate}, bytesize={bytesize}, parity={parity}, stopbits={stopbits}, timeout={timeout}"
            self.ser = self.openSerial(port=port, baudrate=baudrate, bytesize=bytesize, parity=parity, stopbits=stopbits, timeout=timeout)
        except Exception as e:
            logger.error(self.get_Connstr())
            logger.error('open serial fail.')

    def send(self, msg, msg_len, control_id, cmd_id):
        #print("socket send")
        packed = self.genPacket(msg, msg_len, control_id, cmd_id)
        #print('packet send: ', packed)
        try:
            self.ser.write(packed)
        except Exception as e:
            logger.error(self.get_Connstr())
            logger.error(e)

    def read(self):
        logger.debug(f'serial_read')
        # 2(start_code)+1(control_byte)+2(data_len)+2(seq)+1(cmd_id)
        headerLen = 8
        packed = bytearray(0)
        Rxbyte = self.ser.read(1)
        #print('read: ', Rxbyte )

        if self.state == 0:
            if Rxbyte == 'U'.encode('ascii'):
                self.seekerRcvheader += Rxbyte
                self.state = 1
            else:
                self.state = 0
                self.seekerRcvheader = b''
                self.seekerRcvBuff = b''
        elif self.state == 1:
            if Rxbyte == 'f'.encode('ascii'):
                self.seekerRcvheader += Rxbyte
                self.state = 2
            else:
                self.state = 0
                self.seekerRcvheader = b''
                self.seekerRcvBuff = b''
        elif self.state == 2:
            if Rxbyte == b'\x02':
                self.seekerRcvheader += Rxbyte
                self.state = 3
            else:
                self.state = 0
                self.seekerRcvheader = b''
                self.seekerRcvBuff = b''
        elif self.state == 3:
            msgLenL = Rxbyte
            self.seekerRcvheader += Rxbyte
            self.state = 4
        elif self.state == 4:
            msgLenH = Rxbyte
            self.seekerRcvheader += Rxbyte
            self.state = 5
        elif self.state == 5:
            msgSeqL = Rxbyte
            self.seekerRcvheader += Rxbyte
            self.state = 6
        elif self.state == 6:
            msgSeqH = Rxbyte
            self.seekerRcvheader += Rxbyte
            self.state = 7
        elif self.state == 7:
            msgId = Rxbyte
            self.seekerRcvheader += Rxbyte
            self.count = 0
            self.state = 8
        elif self.state == 8:
            data_len_byte = self.seekerRcvheader[3:5]
            if self.count < int.from_bytes(data_len_byte, byteorder=BYTEORDER, signed=False):
                self.seekerRcvBuff += Rxbyte
                self.count += 1
                if self.count >= int.from_bytes(data_len_byte, byteorder=BYTEORDER, signed=False):
                    self.state = 9
            else:
                self.state = 0
                self.seekerRcvheader = b''
                self.seekerRcvBuff = b''
        elif self.state == 9:
            self.seekerRcvBuff += Rxbyte
            self.state = 10
        elif self.state == 10:
            self.seekerRcvBuff += Rxbyte
            dataLen = self.count
            msgLen = headerLen + dataLen + 2
            packed = self.seekerRcvheader + self.seekerRcvBuff
            self.state = 0
            self.seekerRcvheader = b''
            self.seekerRcvBuff = b''
        #print('state: ', self.state)
        
        return self.parsePacket(packed)

    def openSerial(self,port='/dev/ttyUSB2',baudrate=115200,bytesize=8,parity='N',stopbits=1,timeout=1):
        return serial.Serial(port=port,baudrate=baudrate,bytesize=bytesize,parity=parity,stopbits=stopbits,timeout=timeout)

    def close(self):
        pass

def cr_serial(port='/dev/ttyUSB0', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=1):
    ser=Serial(port=port, baudrate=baudrate, bytesize=bytesize, parity=parity, stopbits=stopbits, timeout=timeout)
    return ser

