from panda3d.core import *
from direct.interval.IntervalGlobal import *
from direct.showbase.PythonUtil import fitSrcAngle2Dest

def _distance(start, end):
    v=end-start
    return v.length()

class PathFollower:
    def __init__(self, node, move_speed=4.0, turn_speed=300.0, min_distance=0.5, draw_line=False):
        self.vis_node=node
        self.node=NodePath('Pathfollower')
        self.move_speed=move_speed
        self.turn_speed=turn_speed
        self.min_distance=min_distance
        self.seq=Sequence()
        self.vis=None
        self.draw_line=draw_line
        self.task=taskMgr.add(self._update, 'update_task')

    def _update(self, task):
        dt=globalClock.getDt()
        move_speed=dt*self.move_speed
        origHpr = self.vis_node.get_hpr()
        newHpr=origHpr
        self.vis_node.look_at(self.node)
        targetHpr = self.vis_node.get_hpr()
        origHpr = Vec3(fitSrcAngle2Dest(origHpr[0], targetHpr[0]),
                         fitSrcAngle2Dest(origHpr[1], targetHpr[1]),
                         fitSrcAngle2Dest(origHpr[2], targetHpr[2]))
        delta = max(abs(targetHpr[0] - origHpr[0]),
                    abs(targetHpr[1] - origHpr[1]),
                    abs(targetHpr[2] - origHpr[2]))
        if delta != 0:
            t = min(dt * self.turn_speed/delta, 1.0)
            newHpr = origHpr + (targetHpr - origHpr) * t
        self.vis_node.set_hpr(newHpr)

        pad=0.0
        if self.seq.isPlaying():
            pad=self.min_distance
        dist=self.vis_node.get_distance(self.node)

        if dist > move_speed +pad:
            move_speed*= dist/2.0
            self.vis_node.set_y(self.vis_node,move_speed)

        return task.cont

    def follow_path(self, path):
        if self.draw_line:
            self.draw_path(path)
        self.stop()
        self.set_path(path)
        self.start()

    def set_path(self, path):
        self.seq=Sequence()
        prev_point=None
        blend='easeIn'
        for point in path:
            if prev_point:
                d=_distance(prev_point, point)
                duration=_distance(prev_point, point)/self.move_speed
                self.seq.append(LerpPosInterval(self.node, duration, point, prev_point, blendType=blend))
                blend='noBlend'
            prev_point=point

    def draw_path(self,path):
        if self.vis:
            self.vis.removeNode()
        l=LineSegs()
        l.setColor(1,0,0,1)
        l.setThickness(2)
        l.moveTo(path[0])
        for point in path:
            l.drawTo(point)
        self.vis=render.attachNewNode(l.create())
        self.vis.setZ(0.5)

    def start(self):
        self.seq.start()

    def pause(self):
        if self.seq.isPlaying():
            self.seq.pause()
        else:
            self.seq.resume()

    def stop(self):
        pos=self.node.get_pos(render)
        self.seq.finish()
        self.node.set_pos(render, pos)

    @property
    def active(self):
        return self.seq.isPlaying()
