
from socket import socket, AF_INET, SOCK_DGRAM

from collections import namedtuple

import rencode


header_tuple=namedtuple('header', ['ack', 'echo', 'logout', 'test', 'find_path', 'path'])

Msg=namedtuple('Msg', ['index', 'header', 'need_ack', 'data'])

class Net():
    def __init__(self, recv_from=('', 20000), send_to=('localhost', 20001)):
        self.msg_index=0
        self.binds={}
        self.needing_ack={}
        self.header=header_tuple(*[i for i in range(6)])

        self.recv_from=recv_from
        self.send_to=send_to

        self.recv_socket = socket(AF_INET, SOCK_DGRAM)
        self.recv_socket.bind(self.recv_from)
        self.recv_socket.settimeout(1.0/60.0)

        self.send_socket = socket(AF_INET, SOCK_DGRAM)

        taskMgr.setupTaskChain('net_task_chain', numThreads = 1, tickClock = None, frameSync = True)
        taskMgr.setupTaskChain('exe_task_chain', numThreads = 1, tickClock = None, frameSync = True)

        taskMgr.add(self._update, 'update_task', taskChain = 'net_task_chain')
        taskMgr.doMethodLater(0.5, self.resend, 'resend_task', taskChain = 'net_task_chain')

    def exe(self, cmd, args, task):
        #print (cmd, args)
        cmd(args)
        return task.done

    def bind_call(self, header, function):
        self.binds[int(header)]=function

    def encode_msg(self, msg):
        return rencode.dumps(msg)

    def decode_msg(self, msg):
        return Msg(*rencode.loads(msg))

    def send_ack(self, msg, adress):
        self.send( header=self.header.ack, msg='', index=msg.index, need_ack=False)

    def send(self, header, msg, index=None, need_ack=False, adress=None):
        if adress is None:
            adress=self.send_to
        if index is None:
            index=self.msg_index
            self.msg_index+=1
        #print('Sending: ', index, header, msg)
        encoded_msg=self.encode_msg([index, int(header), int(need_ack), msg])
        self.send_socket.sendto(encoded_msg, adress)
        if need_ack:
            self.needing_ack[index]=(encoded_msg, adress)

    def resend(self, task):
        for msg in self.needing_ack.values():
            self.send_socket.sendto(*msg)
        return task.again

    def _update(self, task):
        try:
            raw_msg, addr = self.recv_socket.recvfrom(4096)
            if raw_msg:
                msg=self.decode_msg(raw_msg)
                #print('got msg: ', msg, addr)
                if msg.header in self.binds:
                    taskMgr.doMethodLater(delayTime=0.0,
                                        funcOrTask=self.exe,
                                        name='exe_task',
                                        extraArgs=[self.binds[msg.header],msg],
                                        appendTask=True,
                                        taskChain = 'exe_task_chain')
                if msg.need_ack == 1:
                    self.send_ack(msg, addr)
                if msg.header == self.header.ack:
                    if msg.index in self.needing_ack:
                        del self.needing_ack[msg.index]
        except:
            pass
        return task.cont
