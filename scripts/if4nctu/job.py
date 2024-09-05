import configparser
import time
from if4nctu.message import Msg, bigtable
import threading
from queue import Queue
import traceback
import logging
logger = logger = logging.getLogger('INTF')


class Job:
    def __init__(self, config_file, job_list, comm_list=[]):
        self.bRun=False
        self.comm_list = comm_list
        self.msg_send = {}
        self.msg_read = {}
        self.outQueue_list = []
        config = configparser.ConfigParser()
        read_ok=config.read(config_file,encoding='utf-8')
        print("ini path=",read_ok)

        for j in job_list:
            j_sp = j.split('-')
            if len(j_sp) == 2:
                msg_ID_str, io = j_sp
            else:
                msg_ID_str, io, _ = j_sp
                j = msg_ID_str+"-"+io
                if io == "IN":
                    io = "OUT"
                else:
                    io = "IN"

            msg_ID = eval(msg_ID_str)
            for comm in self.comm_list:
                comm.set_name(msg_ID_str)
            if io == "IN":
                self.msg_read[msg_ID] = Msg(config[j], msg_ID)
            elif io == "OUT":
                self.msg_send[msg_ID] = Msg(config[j], msg_ID)

        logger.debug(f'job send: {self.msg_send}')


    def _sendMsgByQueu(self,comm,outQueue,hz):
        sleep_dt=1.0/hz*0.5
        while self.bRun:
            if not outQueue.empty():
                msg_ID=outQueue.get()
                comm.send(self.msg_send[msg_ID].encode(), msg_ID)
            else:
                time.sleep(sleep_dt)

    def _sendMsg(self):
        if len(self.outQueue_list)>1:
            for msg_ID in self.msg_send.keys():
                self.msg_send[msg_ID].incValofCounterKey()
                for outQueue in self.outQueue_list:
                    if not outQueue.full():
                        outQueue.put(msg_ID)
        else:
            for msg_ID in self.msg_send.keys():
                logger.debug(f'Send {msg_ID}:')
                self.comm_list[0].send(self.msg_send[msg_ID].encode(), msg_ID)


    def _readMsg(self,comm):
        msg_ID, msg_bytes = comm.read()
        if self.msg_read.get(msg_ID):
            if len(msg_bytes) == 0 and self.msg_read[msg_ID].getMsg_bytes() !=0 :
                logger.warning(f"Len(msg_bytes) of the  msg.  {msg_ID} is 0.")
            else:
                logger.debug(f'Read {msg_ID}:')
                self.msg_read[msg_ID].decode(msg_bytes)
        else:
            pass
            #logger.error(f"this msg. {msg_ID} is isn't belong to this interfice {comm.get_Connstr()}.")

    def exec_send(self, hz):
        self.bRun = True
        queue_num=len(self.comm_list)
        if queue_num>1:
            for i in range(queue_num):
                outQueue=Queue(maxsize=6)
                self.outQueue_list.append(outQueue)
                threading.Thread(target=self._sendMsgByQueu,args=(self.comm_list[i],outQueue,hz),daemon=True).start()

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

    def _get_msgTable(self):
        table = {}
        for msg_ID in self.msg_dict.keys():
            msgTable=self.msg_dict[msg_ID].get_msgTable()
            table.update(msgTable)
        return table


    def get_outTable(self):
        outTable = {}
        for msg_ID in self.msg_send.keys():
            usrTable=self.msg_send[msg_ID].get_msgTable()
            outTable.update(usrTable)
        return outTable

    def update_outTable(self,outTable):
        for msg_ID in self.msg_send.keys():
            self.msg_send[msg_ID].update_bigtable(outTable)

    def update_partialOutTable(self,outTable):
        for msg_ID in self.msg_send.keys():
            self.msg_send[msg_ID].update_partialBigtable(outTable)

    def get_inTable(self):
        inTable = {}
        for msg_ID in self.msg_read.keys():
            usrTable=self.msg_read[msg_ID].get_msgTable()
            inTable.update(usrTable)
        return inTable

class IF_Job(Job):
    def __init__(self, config_file, job_list, comm, freq=30):
        if type(comm) is list:
            comm_list=comm
        else:
            comm_list=[comm]
        super(IF_Job, self).__init__(config_file, job_list, comm_list)

        self.freq = freq
        self.send_th_list=[]
        self.read_th_list = []

    def start(self, bSend=True, bRecv=True,oldBigTable=None):
        if not oldBigTable is None:
            bigtable=self.get_BigTable()
            bigtable.update(oldBigTable)
            for msg_ID in self.msg_send.keys():
                self.msg_send[msg_ID].setValofCounterKey(0)

        if bSend:
            self.send_th = threading.Thread(target=self.exec_send, args=(self.freq,),daemon=True)
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
        self.outQueue_list.clear()
        for comm in self.comm_list:
            try:
                comm.close()
            except Exception as e:
                logger.error(e)
        for read_th in self.read_th_list:
            read_th.join()
        self.read_th_list.clear()
        return self.get_BigTable().copy()





