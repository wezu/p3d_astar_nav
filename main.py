from panda3d.core import *
from direct.showbase import ShowBase
from direct.showbase.DirectObject import DirectObject
from direct.gui.OnscreenText import OnscreenText
from direct.showutil.Rope import Rope

from pathfollower import Pathfollower

import itertools
import heapq
from timeit import default_timer as timer

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

class Demo(DirectObject):
    def __init__(self):
        base = ShowBase.ShowBase()

        self.helptxt = OnscreenText(text="Click on the mesh to make Frowney move", parent=base.a2dTopLeft,
                                       fg=(1, 0, 0, 1), pos=(0.06, -0.1),
                                       align=TextNode.ALeft, scale=.1)

        base.trackball.node().setHpr(0, 40, 0)
        base.trackball.node().setPos(0, 500, 0)

        mesh=loader.loadModel('mesh1')
        mesh.reparentTo(render)
        #mesh.setRenderModeFilledWireframe((0,1,0, 1))

        self.frowney=loader.loadModel('frowney')
        self.frowney.reparentTo(render)
        self.frowney.setZ(2.0)
        self.frowney.setScale(2.0)
        self.frowney.flattenStrong()
        self.seeker=Pathfollower(node=self.frowney, move_speed=50.0, turn_speed=800.0, min_distance=5.0)

        self.plane = Plane(Vec3(0, 0, 1), Point3(0, 0, -0.01))

        self.graph=self.make_nav_graph(mesh)
        self.start=Point3(0,0,0)
        self.end=None

        self.accept('mouse1', self.put_target)


    def put_target(self):
        if self.start is None:
            self.start=self.getMousePos()
            self.frowney.setPos(self.start)
        elif self.end is None:
            self.end = self.getMousePos()
            path=self.find_path(self.start, self.end, self.graph)
            self.curve=self.draw_curve(path)
            smooth_path=self.curve.getPoints(len(path)*2)
            self.seeker.followPath(smooth_path)
        else:
            self.start=self.end
            if self.seeker.active:
                self.seeker.stop()
                self.start=self.seeker.node.getPos()
            self.end=self.getMousePos()
            path=self.find_path(self.start, self.end, self.graph)
            self.curve=self.draw_curve(path)
            smooth_path=self.curve.getPoints(len(path)*2)
            self.seeker.followPath(smooth_path)

    def getMousePos(self):
        if base.mouseWatcherNode.hasMouse():
            mpos = base.mouseWatcherNode.getMouse()
            pos3d = Point3()
            nearPoint = Point3()
            farPoint = Point3()
            base.camLens.extrude(mpos, nearPoint, farPoint)
            if self.plane.intersectsLine(pos3d, render.getRelativePoint(camera, nearPoint),render.getRelativePoint(camera, farPoint)):
                return pos3d
            return None

    def draw_curve(self, path):
        r=Rope()
        verts=[]
        for point in path:
            verts.append((None, point))
        r.setup(order=4, verts=verts, knots = None)
        #r.ropeNode.setThickness(5.0)
        #r.reparentTo(render)
        #r.setColor(1,0,1, 1)
        return r

    def find_path(self, start, end, graph):
        start_time = timer()
        #find the nearest node in the graph to the start and end
        best_start=None
        best_end=None
        start_node=None
        end_node=None
        for node, pos in graph['pos'].items():
            d=self.distance(start, pos)
            if best_start is None:
                start_node=node
                best_start=d
            elif d < best_start:
                start_node=node
                best_start=d
            d=self.distance(end, pos)
            if best_end is None:
                end_node=node
                best_end=d
            elif d < best_end:
                end_node=node
                best_end=d
        path=[start]+self.a_star_search(graph, start_node, end_node)
        path.append(end)
        end_time = timer()
        print "path found in: ",(end_time - start_time)
        return path

    def make_nav_graph(self, mesh):
        #make a list of the triangles
        #get the id of each vert in each triangle and
        #get the position of each vert
        start_time = timer()
        self.triangles=[]
        geomNodeCollection = mesh.findAllMatches('**/+GeomNode')
        for nodePath in geomNodeCollection:
            geomNode = nodePath.node()
            for geom in geomNode.getGeoms():
                geom.decompose()
                vdata = geom.getVertexData()
                vertex = GeomVertexReader(vdata, 'vertex')
                for prim in geom.getPrimitives():
                    for p in range(prim.getNumPrimitives()):
                        s = prim.getPrimitiveStart(p)
                        e = prim.getPrimitiveEnd(p)
                        triangle={'vertex_id':[], 'vertex_pos':[]}
                        for i in range(s, e):
                            vi = prim.getVertex(i)
                            vertex.setRow(vi)
                            v = vertex.getData3f()
                            triangle['vertex_id'].append(vi)
                            triangle['vertex_pos'].append(v)
                        self.triangles.append(triangle)
        #get centers and neighbors
        for triangle in self.triangles:
            triangle['center']=self.get_center(triangle['vertex_pos'])
            triangle['neighbors']=self.get_neighbors(triangle['vertex_id'])
        #construct the dict
        edges={}
        cost={}
        positions={}
        for i, triangle in enumerate(self.triangles):
            edges[i]=triangle['neighbors']
            cost[i]={}
            start=triangle['center']
            positions[i]=start
            for neighbor in triangle['neighbors']:
                cost[i][neighbor]=self.distance(start, self.triangles[neighbor]['center'])
        end_time = timer()
        print "nav graph made in: ",(end_time - start_time)
        return {'neighbors':edges, 'cost':cost, 'pos':positions}

    def distance(self, start, end):
        v=end-start
        # we use the distane to find nearest nodes
        # lengthSquared() should be faster and good enough
        #return v.length()
        return v.lengthSquared()

    def get_center(self, vertex):
        return Vec3((vertex[0][0]+vertex[1][0]+vertex[2][0])/3.0, (vertex[0][1]+vertex[1][1]+vertex[2][1])/3.0, (vertex[0][2]+vertex[1][2]+vertex[2][2])/3.0)

    def get_neighbors(self, vertex):
        looking_for=list(itertools.combinations(vertex, 2))
        neighbors=[]
        for i, triangle in enumerate(self.triangles):
            for combo in looking_for:
                if combo[0] in triangle['vertex_id'] and combo[1] in triangle['vertex_id']:
                    if triangle['vertex_id']!=vertex:
                        neighbors.append(i)
        return neighbors

    def heuristic(self, graph, a, b):
        return self.distance(graph['pos'][a], graph['pos'][b])

    def a_star_search(self, graph, start, goal):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break

            for next in graph['neighbors'][current]:
                new_cost = cost_so_far[current] + graph['cost'][current][next]
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(graph, goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current

        current = goal
        path = [graph['pos'][current]]
        while current != start:
            current = came_from[current]
            path.append(graph['pos'][current])
        path.reverse()
        return path

d=Demo()
base.run()
