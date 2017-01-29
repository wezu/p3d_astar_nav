from panda3d.core import *
from direct.showbase.PythonUtil import fitSrcAngle2Dest

from math import atan2,degrees

def getAngle(p1, p2, vertical=False):
    xDiff = p2.getX() - p1.getX()
    yDiff = p2.getY() - p1.getY()
    zDiff = p2.getZ() - p1.getZ()
    if vertical:
        return degrees(atan2(zDiff, yDiff))
    return degrees(atan2(yDiff, xDiff))


class Pathfollower():
    def __init__(self, node, actor=None, scale=1.0, move_speed=5.0, turn_speed=160.0, min_distance=0.5, align=Vec3(0.5, 0.5, 0.0)):
        self.node=node
        self.move_speed=move_speed
        self.turn_speed=turn_speed
        self.min_distance=min_distance
        self.align=align
        self.path=None
        self.next_target=None
        self.active=False
        self.actor=actor
        self.scale=scale
        self.tilt=0.0
        self.task=taskMgr.add(self._update, 'update_task')

    def cleanup(self):
        taskMgr.remove(self.task)
        self.actor=None
        self.node=None

    def stop(self):
        self.active=False
        if self.actor:
            if(self.actor.getCurrentAnim()!="idle"):
                self.actor.loop("idle")

    def start(self):
        self.active=True

    def followPath(self, path):
        self.path=path
        self._nextTarget()
        if self.next_target==None:
            self.stop()
        self.start()

    def _nextTarget(self):
        if self.path:
            t=self.path.pop(0)
            self.next_target=(Vec3(t)*self.scale)+self.align
        else:
            self.next_target=None

    def _update(self, task):
        if self.next_target and self.active:
            dt=globalClock.getDt()
            #check if we are close enough to get a new target
            if self.next_target.almostEqual(self.node.getPos(),self.min_distance):
            #if self.next_target.almostEqual(self.node.getPos()/self.scale,self.min_distance):
                self._nextTarget()
                if self.next_target==None:
                    self.active=False
                    if self.actor:
                        if(self.actor.getCurrentAnim()!="idle"):
                            self.actor.enableBlend()
                            self.actor.loop("idle")
                            self.actor.loop("idle_l")
                            self.actor.loop("idle_r")
                            L=0.0
                            R=0.0
                            C=1.0
                            if self.tilt>0.0:
                                R=self.tilt*1.8
                                L=0.0
                                C=1.0-R
                            if self.tilt<0.0:
                                R=0.0
                                L=self.tilt*-1.8
                                C=1.0-L
                            #print "L, R, C", L, R, C
                            self.actor.setControlEffect('walk', 0.0)
                            self.actor.setControlEffect('idle', C)
                            self.actor.setControlEffect('idle_l', L)
                            self.actor.setControlEffect('idle_r', R)
                return task.cont
            #rotate to the target if needed
            temp=NodePath('temp')
            temp.setPos(self.node.getPos())
            temp.lookAt(self.next_target)
            targetHpr=temp.getHpr()
            temp.removeNode()
            #self.node.setHpr(target_hpr)

            move_speed=self.move_speed

            origHpr = self.node.getHpr()
            # Make the rotation go the shortest way around.
            origHpr = Vec3(fitSrcAngle2Dest(origHpr[0], targetHpr[0]),
                             fitSrcAngle2Dest(origHpr[1], targetHpr[1]),
                             fitSrcAngle2Dest(origHpr[2], targetHpr[2]))

            # How far do we have to go from here?
            delta = max(abs(targetHpr[0] - origHpr[0]),
                        abs(targetHpr[1] - origHpr[1]),
                        abs(targetHpr[2] - origHpr[2]))
            if delta != 0:
                t = min(dt * self.turn_speed/delta, 1.0)
                newHpr = origHpr + (targetHpr - origHpr) * t
                self.node.setHpr(newHpr)
                #slow down on slopes/bends
                move_speed=self.move_speed/(max(delta/30.0, 1.0))

            #move forward
            #print move_speed
            self.node.setY(self.node, dt*move_speed)
            if self.actor:
                if(self.actor.getCurrentAnim()=="idle"):
                    self.actor.stop()
                    self.actor.disableBlend()
                if(self.actor.getCurrentAnim()!="walk"):
                        self.actor.loop("walk")
        return task.cont

