from __future__ import print_function
from panda3d.core import *
from panda3d.bullet import *
from direct.showbase import ShowBase
from direct.showbase.DirectObject import DirectObject
from direct.gui.OnscreenText import OnscreenText
from direct.showutil.Rope import Rope

from pathfollower import Pathfollower

import itertools
import heapq
from collections import defaultdict
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
        self.seeker=Pathfollower(node=self.frowney, draw_line=False)

        self.graph=self.make_nav_graph(mesh)
        #self.draw_connections(self.graph)


        self.start=Point3(0,0,0)
        self.end=None
        self.curve=None

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

    def draw_connections(self, graph):
        l=LineSegs()
        l.setColor(1,0,0,1)
        l.setThickness(2)
        for start_node, ends in self.graph['neighbors'].items():
            start_pos=self.graph['pos'][start_node]
            for end in ends:
                end_pos=self.graph['pos'][end]
                l.moveTo(start_pos)
                l.drawTo(end_pos)
        render.attachNewNode(l.create())

    def set_target(self, flee=False):
        mpos=self.get_mouse_pos()
        if mpos is None:
            return
        if self.start is None:
            self.start=mpos
            self.frowney.setPos(self.start)
        elif self.end is None:
            self.end = mpos
        else:
            self.end=mpos
            self.start=self.seeker.node.getPos()

        if self.start is not None and self.end is not None:
            path=self.find_path(self.start, self.end, self.graph)
            if path:
                self.curve=self.draw_curve(path)
                if self.curve is not None:
                    path=self.curve.getPoints(len(path))
                self.seeker.followPath(path)

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

    def draw_curve(self, path):
        if len(path)<4:
            return None
        r=Rope()
        verts=[]
        for point in path:
            verts.append((None, point))
        r.setup(order=4, verts=verts, knots = None)
        r.ropeNode.setThickness(2.0)
        #r.reparentTo(render)
        r.setColor(1,0,1, 1)
        r.setZ(0.5)
        return r

    def round_vec3_to_tuple(self, vec):
        for i in range(3):
            vec[i]=round(vec[i]*4.0)/4.0
        return tuple(vec)

    def find_nearest_node(self, pos, graph):
        pos=self.round_vec3_to_tuple(pos)
        if pos in self.graph['lookup']:
            return self.graph['lookup'][pos]

        dist={0.0}
        for i in range(50):
            dist.add(i*0.25)
            dist.add(i*-0.25)
            for x in itertools.permutations(dist, 3):
                key=(pos[0]+x[0], pos[1]+x[1], pos[2]+x[2])
                if key in self.graph['lookup']:
                    return self.graph['lookup'][key]
        return None

    def find_path(self, start, end, graph):
        #find the nearest node in the graph to the start and end
        start_time = timer()
        start_node=self.find_nearest_node(start, graph)
        end_node=self.find_nearest_node(end, graph)
        path=self.a_star_search(graph, start_node, end_node, self.distance)
        end_time = timer()
        if path:
            path=[start]+path
            path.append(end)
            print( "path found in: ",(end_time - start_time))
        else:
            print( "no path found, time : ",(end_time - start_time))
        return path

    def make_nav_graph(self, mesh):
        #make a list of the triangles
        #get the id of each vert in each triangle and
        #get the position of each vert
        start_time = timer()
        self.triangles=[]
        vert_dict=defaultdict(set)
        triangle_pos={}
        dup=defaultdict(set)
        geom_node=mesh.node()
        if type(geom_node).__name__=='ModelRoot':
            geom_node=mesh.getChild(0).node()
        for geom in geom_node.getGeoms():
            #geom.decompose()
            vdata = geom.getVertexData()
            vertex = GeomVertexReader(vdata, 'vertex')
            for prim in geom.getPrimitives():
                num_primitives=prim.getNumPrimitives()
                for p in range(num_primitives):
                    #print ('primitive {} of {}'.format(p, num_primitives))
                    s = prim.getPrimitiveStart(p)
                    e = prim.getPrimitiveEnd(p)
                    triangle={'vertex_id':[], 'vertex_pos':[]}
                    for i in range(s, e):
                        vi = prim.getVertex(i)
                        vertex.setRow(vi)
                        v =tuple([round(i, 4) for i in vertex.getData3f() ])
                        triangle['vertex_pos'].append(v)
                        triangle['vertex_id'].append(vi)
                        vert_dict[vi].add(len(self.triangles))#len(self.triangles) is the triangle id
                        dup[v].add(vi)
                    self.triangles.append(triangle)
        for pos, ids in dup.items():
            if len(ids)>1:
                ids=list((ids))
                union=vert_dict[ids[0]]|vert_dict[ids[1]]
                vert_dict[ids[0]]=union
                vert_dict[ids[1]]=union
        #get centers and neighbors
        for i, triangle in enumerate(self.triangles):
            #print ('triangle ', i ,' of ', len(self.triangles) )
            triangle['center']=self.get_center(triangle['vertex_pos'])
            triangle['neighbors']=self.get_neighbors3(triangle['vertex_id'], vert_dict, i)
        #construct the dict
        edges={}
        cost={}
        positions={}
        for i, triangle in enumerate(self.triangles):
            #print ('neighbor ', i)
            edges[i]=triangle['neighbors']
            cost[i]={}
            start=triangle['center']
            positions[i]=start
            for neighbor in triangle['neighbors']:
                cost[i][neighbor]=self.distance(start, self.triangles[neighbor]['center'])
        lookup={self.round_vec3_to_tuple(value):key for (key, value) in positions.items()}
        end_time = timer()
        print ("nav graph made in: ",(end_time - start_time))
        return {'neighbors':edges, 'cost':cost, 'pos':positions, 'lookup':lookup}

    def distance(self, start, end):
        #start and end should be Vec3,
        #converting tuples/lists to Vec3 here wil slow down pathfinding 10-30x
        v=end-start
        # we use the distane to find nearest nodes
        # lengthSquared() should be faster and good enough
        #return v.length()
        return v.lengthSquared()

    def get_center(self, vertex):
        v=Vec3((vertex[0][0]+vertex[1][0]+vertex[2][0])/3.0, (vertex[0][1]+vertex[1][1]+vertex[2][1])/3.0, (vertex[0][2]+vertex[1][2]+vertex[2][2])/3.0)
        return v

    def get_neighbors(self, vertex, vert_dict, triangle_id):
        common=set()
        for vert_id in vertex:
            common=common | vert_dict[vert_id]
        common=common-{triangle_id}
        return list(common)


    def get_neighbors3(self, vertex, vert_dict, triangle_id):
        #returns only 3 neighbors per triangle
        common=set()
        for pair in itertools.combinations(vertex, 2):
            common=common | vert_dict[pair[0]] & vert_dict[pair[1]]
        common=common-{triangle_id}
        return list(common)

    def heuristic(self, graph, a, b):
        #a better heuristic could be used, but it will probably slow things down :(
        return self.distance(graph['pos'][a], graph['pos'][b])

    def a_star_search(self, graph, start, goal, heuristic, max_move=8000):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            max_move-=1
            if max_move<0:
                print( 'path to long')
                return None

            if current == goal:
                break

            for next in graph['neighbors'][current]:
                new_cost = cost_so_far[current] + graph['cost'][current][next]
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + heuristic(graph['pos'][goal],graph['pos'][next])
                    frontier.put(next, priority)
                    came_from[next] = current
        current = goal
        path = [graph['pos'][current]]
        while current != start:
            try:
                current = came_from[current]
            except:
                print( 'no path')
                return None
            path.append(graph['pos'][current])
        path.reverse()
        return path

d=Demo()
base.run()
