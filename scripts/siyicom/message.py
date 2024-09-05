import traceback
import logging
from siyicom.sy_tool import BYTEORDER, c_cmd_id

logger = logging.getLogger('SIYI')
bigtable = {}

def get_BigTable():
    global bigtable
    return bigtable


def convert_Byte2Value(bigTable):
    bigTable_local=bigTable.copy()
    bigTableValue = {}
    for k,v in bigTable_local.items():
        ksp = k.split('_')
        if ksp[-1] == 's':
            value = int.from_bytes(bigTable_local[k], byteorder=BYTEORDER, signed=True)
        elif ksp[-1] == 'u':
            value = int.from_bytes(bigTable_local[k], byteorder=BYTEORDER, signed=False)

        if ksp[-2] == '2':
            value = value / 100.0
        elif ksp[-2] == '1':
            value = value / 10.0
        elif ksp[-2] == '3':
            value = value / 1000.0
        elif ksp[-2] == '4':
            value = value / 10000.0
        elif ksp[-2] == '7':
            value = value / 10000000.0
        bigTableValue[k] = value
    return bigTableValue


def convert_Byte2Value_KV(k,v):
    ksp = k.split('_')
    if ksp[-1] == 's':
        value = int.from_bytes(v, byteorder=BYTEORDER, signed=True)
    elif ksp[-1] == 'u':
        value = int.from_bytes(v, byteorder=BYTEORDER, signed=False)
    if ksp[-2] == '2':
        value = value / 100.0
    elif ksp[-2] == '1':
        value = value / 10.0
    elif ksp[-2] == '3':
        value = value / 1000.0
    elif ksp[-2] == '4':
        value = value / 10000.0
    elif ksp[-2] == '7':
        value = value / 10000000.0
    return value

def get_BigTableValue():
    global bigtable
    return convert_Byte2Value(bigtable)


def getMaxMinIntVal(byte_len,signed):
    if signed:
        maxVal=2**(byte_len*8-1)-1
        minVal=-2**(byte_len*8-1)
    else:
        maxVal=2**(byte_len*8)-1
        minVal=0
    return maxVal,minVal

MaxMinTab_Signed=[getMaxMinIntVal(i,True) for i in range(9)]
MaxMinTab_UnSigned=[getMaxMinIntVal(i,False) for i in range(9)]

def value2Bytes(key,value,byte_len):
    ksp = key.split('_')
    if ksp[-2] == '2':
        value = value * 100.0
    elif ksp[-2] == '1':
        value = value * 10.0
    elif ksp[-2] == '3':
        value = value * 1000.0
    elif ksp[-2] == '4':
        value = value * 10000.0
    elif ksp[-2] == '7':
        value = value * 10000000.0
    if ksp[-1] == 's':
        value = int(min(max(value,MaxMinTab_Signed[byte_len][1]),MaxMinTab_Signed[byte_len][0])).to_bytes(byte_len, byteorder=BYTEORDER, signed=True)
    elif ksp[-1] == 'u':
        value = int(min(max(value,MaxMinTab_UnSigned[byte_len][1]),MaxMinTab_UnSigned[byte_len][0])).to_bytes(byte_len, byteorder=BYTEORDER, signed=False)
    elif ksp[-1] == 't':
        value = value
    else:
        value = None
    return value

class Msg:
    def __init__(self, section, bisection, ctr_ID = 0xff):
        """
        :param section:
        :param ctr_ID:
        """
        global bigtable
        self.bigtable = bigtable
        self.send_msg_len = 0
        self.send_cmd_id = 0
        self._icd = {}
        self._bisection = {}
        self.ctr_ID = ctr_ID
        for k, v in bisection.items():
            for vi in v.replace(" ", "").split(',')[:2]:
                if vi not in self._bisection.keys():
                    self._bisection[vi] = []
                    self._bisection[vi].append(k)
                else:
                    self._bisection[vi].append(k)

        for k, v in section.items():
            byte_len_str, default = v.replace(" ", "").split(',')[:2]
            byte_len = int(byte_len_str)
            self._icd[k] = byte_len
            if k.split('_')[-1] == 's':
                self.bigtable[k] = int(default).to_bytes(byte_len, byteorder=BYTEORDER, signed=True)
            elif k.split('_')[-1] == 'u':
                self.bigtable[k] = int(default).to_bytes(byte_len, byteorder=BYTEORDER, signed=False)
            elif k.split('_')[-1] == 't':
                self.bigtable[k] = default
    
    
    def getMsg_bytes(self):
        return self.total_bytes


    def encode(self):
        msg_bytes = b""
        cmd_ID_str = self.bigtable[c_cmd_id]
        for k in self._bisection[cmd_ID_str]:
            msg_bytes += self.bigtable[k]
        logger.info(f'sendMsg: {msg_bytes}')
        self.send_msg_len = len(msg_bytes)
        self.send_cmd_id = eval(cmd_ID_str)
        #print('sendMsg:', cmd_ID_str)
        return msg_bytes
    

    def decode(self, cmd_ID, msg_bytes):
        ptr = 0
        local_tab={}
        cmd_ID_str = ''
        #print('check: ', int(cmd_ID).to_bytes(1, byteorder=BYTEORDER, signed=False))
        temp = str(int(cmd_ID).to_bytes(1, byteorder=BYTEORDER, signed=False))
        temp = temp.replace(" ", "").split('\\')[1]
        #print('temp: ', temp)
        if len(temp) < 3:
            if cmd_ID <= 0x0f:
                cmd_ID_str = '0x0' + hex(cmd_ID).replace(" ", "").split('x')[1].upper()
            else:
                cmd_ID_str = '0x' + hex(cmd_ID).replace(" ", "").split('x')[1].upper()
        elif len(temp) == 4:
            cmd_ID_str = '0x' + str(int(cmd_ID).to_bytes(1, byteorder=BYTEORDER, signed=False))[-3:-1].upper()
        for k in self._bisection[cmd_ID_str]:
            local_tab[k] = msg_bytes[ptr:ptr + self._icd[k]]
            ptr += self._icd[k]

        self.bigtable.update(local_tab)
        logger.info(f'decode bigtable: {self.bigtable}')


    def get_msgLen(self):
        return self.send_msg_len
    

    def get_cmdID(self):
        return self.send_cmd_id
    

    def update_partialBigtable(self, msgTable: {}):
        localTab={}
        for k, v in msgTable.items():
            byte_len = self._icd[k]
            value = msgTable[k]
            value = value2Bytes(k, value, byte_len)
            localTab[k] = value
            #print('value: ', value)
        self.bigtable.update(localTab)
        logger.info(f'update partial Bigtable: {self.bigtable}')

    def get_msgTable(self):
        msgTable = {}
        localBigTab=self.bigtable.copy()
        value = None
        for k, byte_len in self._icd.items():
            ksp=k.split('_')
            if ksp[-1] == 's':
                value = int.from_bytes(localBigTab[k], byteorder=BYTEORDER, signed=True)
            elif ksp[-1] == 'u':
                value = int.from_bytes(localBigTab[k], byteorder=BYTEORDER, signed=False)
            elif ksp[-1] == 't':
                value = localBigTab[k]

            if ksp[-2]=='2':
                value=value/100.0
            elif ksp[-2]=='1':
                value = value / 10.0
            elif ksp[-2]=='3':
                value = value / 1000.0
            elif ksp[-2] == '4':
                value = value / 10000.0
            elif ksp[-2] == '7':
                value = value / 10000000.0
            msgTable[k]=value
        logger.info(f'return msg:{self.ctr_ID} ,Table: {msgTable}')
        return msgTable
