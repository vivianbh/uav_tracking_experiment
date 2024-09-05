import traceback
import logging
from if4nctu.if_tool import BYTEORDER

logger = logging.getLogger('INTF')
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
    else:
        value = None
    return value

class Msg:
    def __init__(self, section, msg_ID = 0xff):
        """
        :param section:
        :param msg_ID:
        """
        global bigtable
        self.bigtable = bigtable
        self._icd = {}
        self.msg_ID=msg_ID
        self.total_bytes=section
        for k, v in section.items():
            byte_len_str, default = v.replace(" ", "").split(',')[:2]
            byte_len = int(byte_len_str)
            self._icd[k] = byte_len
            if k.split('_')[-1] == 's':
                self.bigtable[k] = int(default).to_bytes(byte_len, byteorder=BYTEORDER, signed=True)
            elif k.split('_')[-1] == 'u':
                self.bigtable[k] = int(default).to_bytes(byte_len, byteorder=BYTEORDER, signed=False)


    def getMsg_bytes(self):
        return self.total_bytes


    def encode(self):
        msg_bytes = b""
        for k in self._icd.keys():
            msg_bytes += self.bigtable[k]
        logger.info(f'sendMsg: {msg_bytes}')
        # print('sendMsg:', msg)
        return msg_bytes

    #def save_to_bigtable(self, data):
    def decode(self, msg_bytes):
        ptr = 0
        local_tab={}
        for k, byte_len in self._icd.items():
            local_tab[k] = msg_bytes[ptr:ptr + byte_len]
            ptr += byte_len

        self.bigtable.update(local_tab)
        logger.info(f'decode bigtable: {self.bigtable}')

    def get_msgID(self):
        return self.msg_ID


    def update_bigtable(self, msgTable: {}):
        localTab = {}
        for k, byte_len in self._icd.items():
            try:
                value=msgTable[k]
                value=value2Bytes(k,value,byte_len)
                localTab[k] = value
            except:
                emsg=f"key,value,len={k},{value},{byte_len}"
                logger.error(emsg)
                traceback.print_exc()
        self.bigtable.update(localTab)
        logger.info(f'update bigtable: {self.bigtable}')
        # print('bigtable:', self.bigtable)

    def update_partialBigtable(self, msgTable: {}):
        localTab={}
        for k, v in msgTable.items():
            byte_len=self._icd[k]
            value=msgTable[k]
            value=value2Bytes(k,value,byte_len)
            localTab[k] = value
        self.bigtable.update(localTab)
        logger.info(f'update partial Bigtable: {self.bigtable}')

    def get_msgTable(self):
        msgTable = {}
        localBigTab=self.bigtable.copy()
        for k, byte_len in self._icd.items():
            ksp=k.split('_')
            if ksp[-1] == 's':
                value = int.from_bytes(localBigTab[k], byteorder=BYTEORDER, signed=True)
            elif ksp[-1] == 'u':
                value = int.from_bytes(localBigTab[k], byteorder=BYTEORDER, signed=False)

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
        logger.info(f'return msg:{self.msg_ID} ,Table: {msgTable}')
        return msgTable
