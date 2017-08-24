from __future__ import print_function
from panda3d.core import *
load_prc_file_data('', 'window-type none')
load_prc_file_data('', 'audio-library-name null')
load_prc_file_data('', 'client-sleep 0.01')
from direct.showbase.ShowBase import ShowBase
import random
import os
import time

from net import Net

from navgraph import NavGraph


class Server(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        self.network=Net(recv_from=('', 20000), send_to=('localhost', 20001))

        self.network.bind_call(self.network.header.logout, self.exit)
        self.network.bind_call(self.network.header.find_path, self.find_path)
        print('Server started')

        mesh=loader.loadModel('mesh')
        self.graph=NavGraph(mesh, smooth=0.5, max_moves=20000, debug=False, draw_graph=False)

    def find_path(self, msg):
        start=Vec3(*msg.data[0])
        end=Vec3(*msg.data[1])
        path=self.graph.find_path(start, end)
        if path:
            path=[tuple(i) for i in path]
            self.network.send(header=self.network.header.path, msg=path)

    def echo(self, msg):
        print(msg.data)

    def exit(self, msg):
        print('Server terminated!')
        os._exit(1)

app = Server()
app.run()
