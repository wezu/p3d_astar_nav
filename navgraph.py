from __future__ import print_function
from panda3d.core import *
from direct.showutil.Rope import Rope
from functools import wraps
import itertools
import heapq
import sys
from collections import defaultdict
if sys.version_info >= (3, 3):
    from time import perf_counter as timer
else:
    if sys.platform == "win32":
        from time import clock as timer
    else:
        from time import time as timer

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

class NavGraph:
    def __init__(self, mesh, smooth=0.5, edge_neighbors_only=True, max_moves=8000, debug=False, draw_graph=False):
        self.debug=debug
        self.smooth_factor=smooth
        self.max_moves=max_moves
        #load the mesh
        self.make_nav_graph(mesh, edge_neighbors_only, draw_graph)

    def debug_timer(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            if args[0].debug:
                start = timer()
                r = func(*args, **kwargs)
                end = timer()
                print('DEBUG: {}.{}() time: {}'.format(func.__module__, func.__name__, end-start))
            else:
                r = func(*args, **kwargs)
            return r
        return wrapper

    def test_performance(self, start_node=610, end_node=977):
        start_time = timer()
        for _ in range(1000):
            self._a_star_search(start_node, end_node, self._distance)
        end_time = timer()
        print ("Performance:", end_time-start_time)

    def draw_connections(self):
        try:
            self.visual.removeNode()
        except:
            pass
        l=LineSegs()
        l.setColor(1,0,0,1)
        l.setThickness(2)
        for start_node, ends in self.graph['neighbors'].items():
            start_pos=self.graph['pos'][start_node]
            for end in ends:
                end_pos=self.graph['pos'][end]
                l.moveTo(start_pos)
                l.drawTo(end_pos)
        self.visual=render.attachNewNode(l.create())

    def _round_vec3_to_tuple(self, vec):
        return tuple([round(x*4.0)/4.0 for x in vec])

    def _find_nearest_node(self, pos):
        pos=self._round_vec3_to_tuple(pos)
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

    def _smooth_path(self, path, smooth_factor=0.5):
        if len(path)<4 or smooth_factor <0.01:
            return path
        r=Rope()
        verts=[(None, point) for point in path]
        r.setup(order=4, verts=verts, knots = None)
        #r.ropeNode.setThickness(2.0)
        #r.reparentTo(render)
        #r.setColor(1,0,1, 1)
        #r.setZ(0.5)
        return r.getPoints(int(len(path)*smooth_factor))

    @debug_timer
    def find_path(self, start, end):
        '''Returns a path (list of points) from start to end,
        start and end must be Vec3/Point3/VBase3, or 3 element tuple/list'''
        start_node=self._find_nearest_node(start)
        end_node=self._find_nearest_node(end)
        path=self._a_star_search(start_node, end_node, self._distance, self.max_moves)
        if path:
            path=[start]+path
            path.append(end)
        else:
            return None
        return self._smooth_path(path, self.smooth_factor)

    def find_first_geom(self, mesh):
        for child in mesh.getChildren():
            node=child.node()
            if node.isGeomNode():
                return node.getGeom(0)

    @debug_timer
    def make_nav_graph(self, mesh, edge_neighbors_only=True, draw_graph=False):
        '''Creates a navigation graph from a 3D mesh,
        A node is created for each triangle in the mesh,
        nodes are connected either by shared edges (edge_neighbors_only=True),
        or by shared vertex (edge_neighbors_only=False).
        '''
        _get_center=self._get_center
        _get_neighbors=self._get_neighbors
        _distance=self._distance
        _round_vec3_to_tuple=self._round_vec3_to_tuple

        #make a list of the triangles
        #get the id of each vert in each triangle and
        #get the position of each vert
        triangles=[]
        vert_dict=defaultdict(set)
        #only works ok with one geom so flatten the mesh befor comming here
        geom=self.find_first_geom(mesh)
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
                    v =[round(i, 4) for i in vertex.getData3f() ]
                    vertex_id=tuple([round(i*4.0)/4.0 for i in v])
                    triangle['vertex_pos'].append(v)
                    triangle['vertex_id'].append(vertex_id)
                    vert_dict[vertex_id].add(len(triangles))#len(self.triangles) is the triangle id
                triangles.append(triangle)

        #get centers and neighbors
        for i, triangle in enumerate(triangles):
            #print ('triangle ', i ,' of ', len(self.triangles) )
            triangle['center']=_get_center(triangle['vertex_pos'])
            triangle['neighbors']=_get_neighbors(triangle['vertex_id'], vert_dict, i, edge_neighbors_only)
        #construct the dict
        edges={}
        cost={}
        positions={}
        for i, triangle in enumerate(triangles):
            #print ('neighbor ', i)
            edges[i]=triangle['neighbors']
            cost[i]={}
            start=triangle['center']
            positions[i]=start
            for neighbor in triangle['neighbors']:
                cost[i][neighbor]=_distance(start, triangles[neighbor]['center'])
        lookup={_round_vec3_to_tuple(value):key for (key, value) in positions.items()}
        self.graph= {'neighbors':edges, 'cost':cost, 'pos':positions, 'lookup':lookup}
        if draw_graph:
            self.draw_connections()

    def _distance(self, start, end):
        #start and end should be Vec3,
        #converting tuples/lists to Vec3 here wil slow down pathfinding 10-30x
        v=end-start
        # we use the distane to find nearest nodes
        # lengthSquared() should be faster and good enough
        #return v.length()
        return v.lengthSquared()

    def _get_center(self, vertex):
        v=Vec3((vertex[0][0]+vertex[1][0]+vertex[2][0])/3.0, (vertex[0][1]+vertex[1][1]+vertex[2][1])/3.0, (vertex[0][2]+vertex[1][2]+vertex[2][2])/3.0)
        return v

    def _get_neighbors(self, vertex, vert_dict, triangle_id, edge_only=True):
        common=set()
        if edge_only:
            for pair in itertools.combinations(vertex, 2):
                common=common | vert_dict[pair[0]] & vert_dict[pair[1]]
        else:
            for vert_id in vertex:
                common=common | vert_dict[vert_id]
        common=common-{triangle_id}
        return list(common)

    def _a_star_search(self, start, goal, heuristic, max_move=8000):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        neighbors=self.graph['neighbors']
        cost=self.graph['cost']
        pos=self.graph['pos']

        while not frontier.empty():
            current = frontier.get()

            if max_move is not None:
                max_move-=1
                if max_move<0:
                    return None

            if current == goal:
                break

            for next in neighbors[current]:
                new_cost = cost_so_far[current] + cost[current][next]
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + heuristic(pos[goal], pos[next])
                    frontier.put(next, priority)
                    came_from[next] = current
        current = goal
        path = [pos[current]]
        while current != start:
            try:
                current = came_from[current]
            except:
                return None
            path.append(pos[current])
        path.reverse()
        return path
