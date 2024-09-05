import configparser
import time
from siyicom.message import Msg, bigtable
import threading
import traceback
import logging
logger = logging.getLogger('SIYI')


class Job:
    def __init__(self, config_file, job_list, comm_list=[]):
        self.bRun=False
        self.comm_list = comm_list
        self.msg_send = {}
        self.msg_read = {}
        config = configparser.ConfigParser()
        read_ok=config.read(config_file,encoding='utf-8')
        print("ini path=",read_ok)

        for j in job_list:
            j_sp = j.split('-')
            ctr_ID_str, io = j_sp

            ctr_ID = eval(ctr_ID_str)
            for comm in self.comm_list:
                comm.set_name(ctr_ID_str)
            if io == "IN":
                self.msg_read[ctr_ID] = Msg(config[j], config[ctr_ID_str+"-CMD"], ctr_ID)

            elif io == "OUT":
                self.msg_send[ctr_ID] = Msg(config[j], config[ctr_ID_str+"-CMD"], ctr_ID)

        logger.debug(f'job send: {self.msg_send}')

    def _sendMsg(self):
        for ctr_ID in self.msg_send.keys():
            logger.debug(f'Send {ctr_ID}:')
            self.comm_list[0].send(self.msg_send[ctr_ID].encode(), self.msg_send[ctr_ID].get_msgLen(), ctr_ID, self.msg_send[ctr_ID].get_cmdID())


    def _readMsg(self,comm):
        ctr_ID, cmd_ID, msg_bytes = comm.read()
        #print('msg: ', ctr_ID, cmd_ID, msg_bytes)
        if self.msg_read.get(ctr_ID):
            if len(msg_bytes) == 0 and self.msg_read[ctr_ID].getMsg_bytes() !=0 :
                logger.warning(f"Len(msg_bytes) of the  msg.  {ctr_ID} is 0.")
            else:
                logger.debug(f'Read {ctr_ID}:')
                self.msg_read[ctr_ID].decode(cmd_ID, msg_bytes)
        else:
            logger.debug(f"this msg. {ctr_ID} is isn't belong to this interfice {comm.get_Connstr()}.")

    def exec_send(self, hz):
        self.bRun = True
        while self.bRun:
            st = time.time()
            self._sendMsg()
            end = time.time()
            use_time = end - st
            sleep_time = (1 / hz) - use_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                logger.warning(f'job:exec_send() time_out(msgIDS:{self.msg_send.keys()}, {hz} HZ): {use_time}')

    def exec_read(self,comm):
        self.bRun=True
        while self.bRun:
            try:
                self._readMsg(comm)
            except Exception as e:
                traceback.print_exc()
                logger.error(comm.get_Connstr())
                logger.error(e)

    def get_BigTable(self):
        return bigtable


    def get_outTable(self):
        outTable = {}
        for ctr_ID in self.msg_send.keys():
            usrTable=self.msg_send[ctr_ID].get_msgTable()
            outTable.update(usrTable)
        return outTable

    def update_outTable(self,outTable):
        for msg_ID in self.msg_send.keys():
            self.msg_send[msg_ID].update_bigtable(outTable)

    def update_partialOutTable(self,outTable):
        #print('yes')
        for msg_ID in self.msg_send.keys():
            self.msg_send[msg_ID].update_partialBigtable(outTable)

    def get_inTable(self):
        inTable = {}
        for ctr_ID in self.msg_read.keys():
            usrTable=self.msg_read[ctr_ID].get_msgTable()
            inTable.update(usrTable)
        return inTable

class SY_Job(Job):
    def __init__(self, config_file, job_list, comm, freq=30):
        if type(comm) is list:
            comm_list=comm
        else:
            comm_list=[comm]
        super(SY_Job, self).__init__(config_file, job_list, comm_list)

        self.freq = freq
        self.send_th_list=[]
        self.read_th_list = []

    def start(self, bSend=True, bRecv=True, oldBigTable=None):
        if not oldBigTable is None:
            bigtable=self.get_BigTable()
            bigtable.update(oldBigTable)
            for ctr_ID in self.msg_send.keys():
                self.msg_send[ctr_ID].setValofCounterKey(0)
        if bSend:
            self.send_th = threading.Thread(target=self.exec_send, args=(self.freq,), daemon=True)
            self.send_th_list.append(self.send_th)
            self.send_th.start()
        if bRecv:
            for comm in self.comm_list:
                recv_th=threading.Thread(target=self.exec_read, args=(comm,), daemon=True)
                self.read_th_list.append(recv_th)
                recv_th.start()

    def stop(self):
        self.bRun=False
        for send_th in self.send_th_list:
            send_th.join()
        self.send_th_list.clear()
        for comm in self.comm_list:
            try:
                comm.close()
            except Exception as e:
                logger.error(e)
        for read_th in self.read_th_list:
            read_th.join()
        self.read_th_list.clear()
        return self.get_BigTable().copy()





