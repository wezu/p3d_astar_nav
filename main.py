from __future__ import print_function
from panda3d.core import *
from panda3d.bullet import *
from direct.showbase import ShowBase
from direct.showbase.DirectObject import DirectObject
from direct.gui.OnscreenText import OnscreenText

from pathfollower import Pathfollower
from navgraph import NavGraph

class Demo(DirectObject):
    def __init__(self):
        base = ShowBase.ShowBase()

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
        self.seeker=PathFollower(node=self.frowney, min_distance=1.0, draw_line=False)
        #NavGraph is the class that finds a path between two points
        self.graph=NavGraph(mesh, smooth=0.5, max_moves=2000, debug=True, draw_graph=False)

        self.start=Point3(0,0,0)
        self.end=None

        #mouse picking/collision detection
        self.world_node = render.attachNewNode('World')
        self.world = BulletWorld()
        triMeshData = BulletTriangleMesh()
        geom_node=mesh.node()
        if type(geom_node).__name__=='ModelRoot':
            geom_node=mesh.getChild(0).node()
        for geom in geom_node.getGeoms():
            triMeshData.addGeom(geom)
        shape = BulletTriangleMeshShape(triMeshData, dynamic=False)
        geometry = self.world_node.attachNewNode(BulletRigidBodyNode('StaticGeometry'))
        geometry.node().addShape(shape)
        geometry.node().setMass(0.0)
        self.world.attachRigidBody(geometry.node())

        self.accept('mouse1', self.set_target)

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
            path=self.graph.find_path(self.start, self.end)
            if path:
                self.seeker.followPath(path)
            else:
                print ('WARRNING: No path or path too long!')
                self.seeker.stop()

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
