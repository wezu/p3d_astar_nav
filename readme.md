A little silly path finding and nav mesh making script for Panda3D.


The pathfinding works on any 2d or 3d mesh made from triangles (the script should decompose other primitives, but that's not tested).


A pathfinding graph is made by putting a node in the center of each triangle and then connecting it with adjacent nodes just the way the original triangles are connected in the mesh. The graph is just a python dict with more nested dicts inside, so it can be serialized and written to disk using json, xml, yaml or any other such tools.


The pathfinding uses a A-star algorithm (from http://www.redblobgames.com/pathfinding/) and returns a list of Point3 locations that needs to be traversed in other to get from one point to another.


In this implementation the list provided by the pathfinding algorithm is feed into a Rope object that constructs a smooth, uniform path (nurbs curve).


Finally, in this demo there's a PathFallower that simply moves a node along the given path. In a 'real-world' situation it's advised to use some sort of collision detection to keep the node on the surface of your mesh since the path is constructed from the centers of triangles and additionally smoothed out. Depending on the mesh, it it also possible that the plotted path will cut corners.
