from panda3d.core import *
from direct.showbase.PythonUtil import fitSrcAngle2Dest

from math import atan2,degrees

def getAngle(p1, p2): 
    xDiff = p2.getX() - p1.getX()
    yDiff = p2.getY() - p1.getY()
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
            self.next_target=(Vec3(t[0], t[1], 0.0)*self.scale)+self.align   
        else:
            self.next_target=None    
        
    def _update(self, task):        
        if self.next_target and self.active:         
            dt=globalClock.getDt()
            #check if we are close enough to get a new target
            t=Vec2(self.next_target[0],self.next_target[1])
            p=Vec2(self.node.getPos()[0],self.node.getPos()[1])
            #print t, p
            if t.almostEqual(p,self.min_distance):                
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
            target_h=90.0+getAngle(self.node.getPos(), self.next_target)            
            h = self.node.getH(render)
            
            # Make the rotation go the shortest way around.
            h = fitSrcAngle2Dest(h, target_h)
            # How far do we have to go from here?
            delta = abs(target_h - h)
            if delta != 0:  
                # Figure out how far we should rotate in this frame, based on the
                # distance to go, and the speed we should move each frame.               
                t = min(dt * self.turn_speed/delta, 1.0)
                new_h =h + (target_h - h) * t
                self.node.setH(new_h)
            #move forward
            self.node.setY(self.node, -dt*self.move_speed)
            if self.actor:
                if(self.actor.getCurrentAnim()=="idle"):
                    self.actor.stop()
                    self.actor.disableBlend()
                if(self.actor.getCurrentAnim()!="walk"):
                        self.actor.loop("walk")                                                       
        return task.cont    
