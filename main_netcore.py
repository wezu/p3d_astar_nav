from __future__ import print_function
from panda3d.core import *
from panda3d.bullet import *
from direct.showbase import ShowBase
from direct.showbase.DirectObject import DirectObject
from direct.gui.OnscreenText import OnscreenText

from pathfollower import PathFollower
from navgraph import NavGraph

import subprocess
import os
import signal
import sys

from net import Net


class Demo(DirectObject):
    def __init__(self):
        base = ShowBase.ShowBase()

        base.win.set_close_request_event('exit-event')
        self.accept('exit-event', self.on_exit)

        self.network=Net(recv_from=('', 20001), send_to=('localhost', 20000))
        self.network.bind_call(self.network.header.path, self.follow_path)

        self.server_pid=subprocess.Popen([sys.executable,"nav_server.py"]).pid


        self.helptxt = OnscreenText(text="Click on the mesh to make Frowney move", parent=base.a2dTopLeft,
                                       fg=(1, 0, 0, 1), pos=(0.06, -0.1),
                                       align=TextNode.ALeft, scale=.1)

        base.trackball.node().setHpr(0, 40, 0)
        base.trackball.node().setPos(0, 100, 0)

        mesh=loader.loadModel('mesh')
        mesh.reparentTo(render)
        mesh.setRenderModeFilledWireframe((0,1,0, 1))

        self.frowney=loader.loadModel('frowney')
        self.frowney.reparentTo(render)
        self.frowney.setH(180.0) #pathfallower walks backwards...
        self.frowney.setZ(0.5)
        self.frowney.setScale(0.5)
        self.frowney.flattenStrong()
        #Pathfollower is the class that moves a node along a path
        self.seeker=PathFollower(node=self.frowney, move_speed=8.0, min_distance=1.0, draw_line=False)
        #NavGraph is the class that finds a path between two points
        #self.graph=NavGraph(mesh, smooth=0.5, max_moves=2000, debug=True, draw_graph=False)

        self.start=Point3(0,0,0)
        self.end=None

        #mouse picking/collision detection
        self.world_node = render.attachNewNode('World')
        self.world = BulletWorld()
        triMeshData = BulletTriangleMesh()
        triMeshData.addGeom(mesh.getChild(0).node().getGeom(0))
        shape = BulletTriangleMeshShape(triMeshData, dynamic=False)
        geometry = self.world_node.attachNewNode(BulletRigidBodyNode('StaticGeometry'))
        geometry.node().addShape(shape)
        geometry.node().setMass(0.0)
        self.world.attachRigidBody(geometry.node())

        self.accept('mouse1', self.set_target)

    def on_exit(self):
        self.network.send(self.network.header.logout, '')
        try:
            os.kill(self.server_pid, signal.SIGTERM) #or signal.SIGKILL
        except:
            pass
        base.userExit()

    def follow_path(self, msg):
        path=msg.data
        print('Got path from, to',path[0], path[-1])
        path=[Vec3(*i) for i in path]
        self.seeker.follow_path(path)

    def set_target(self, flee=False):
        mpos=self.get_mouse_pos()
        if mpos is None:
            return
        if self.end is None:
            self.end = mpos
        else:
            self.end=mpos
            self.start=self.seeker.node.getPos()

        if self.start is not None and self.end is not None:
            self.seeker.stop()
            print('Requesting path from, to:',self.start, self.end )
            self.network.send(self.network.header.find_path, (tuple(self.start), tuple(self.end)))

            #path=self.graph.find_path(self.start, self.end)
            #if path:
            #    self.seeker.follow_path(path)
            #else:
            #    print ('WARRNING: No path or path too long!')
            #    self.seeker.stop()

    def get_mouse_pos(self):
        if base.mouseWatcherNode.hasMouse():
            pMouse = base.mouseWatcherNode.getMouse()
            pFrom = Point3()
            pTo = Point3()
            base.camLens.extrude(pMouse, pFrom, pTo)
            # Transform to global coordinates
            pFrom = render.getRelativePoint(base.cam, pFrom)
            pTo = render.getRelativePoint(base.cam, pTo)
            result = self.world.rayTestClosest(pFrom, pTo)
            if result.hasHit():
                return result.getHitPos()
        return None

d=Demo()
base.run()
