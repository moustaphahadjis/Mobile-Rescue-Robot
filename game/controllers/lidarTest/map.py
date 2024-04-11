import numpy as np
import math
class Map:
    def __init__(self,robot, timestep):
        self.display = robot.getDevice('display')
        

        self.l1 = robot.getDevice('laser1')
        self.l2 = robot.getDevice('laser2')
        self.l3 = robot.getDevice('laser3')
        self.l4 = robot.getDevice('laser4')
        self.l5 = robot.getDevice('laser5')
        self.l6 = robot.getDevice('laser6')
        self.l7 = robot.getDevice('laser7')

        self.l1.enable(timestep)
        self.l2.enable(timestep)
        self.l3.enable(timestep)
        self.l4.enable(timestep)
        self.l5.enable(timestep)
        self.l6.enable(timestep)
        self.l7.enable(timestep)

        self.lasers = [self.l1, self.l2, self.l3, self.l4,self.l5,self.l6,self.l7]

        self.displayRes = 1000
        self.scale = 0.5

        self.world =  np.zeros((self.displayRes, self.displayRes,1))
        self.distThr = 0.1

        self.initDisplay()

    def initDisplay(self):
       for i in range(self.displayRes):
          for j in range(self.displayRes):
            self.display.setColor(0xffffff)
            self.display.drawPixel(i,j)
    def detectWall(self, currentPos, orientation):
        for i in range(len(self.lasers)):
            angle = math.radians(i*30 + orientation)
            dist = self.lasers[i].getValue()

            y = dist* math.sin(angle) 
            x = dist* math.cos(angle) 
            #print(f'[{x, y}]')
            self.setWall(x*self.displayRes*self.scale + currentPos[0], -y*self.displayRes*self.scale + currentPos[1] )
            
            
    def setTrack(self, x, y):
     if x<self.displayRes and y <self.displayRes:
        #print(x)
        if self.world[int(x)][int(y)][0] == 0:
            self.world[int(x)][int(y)][0] = 1
            self.display.setColor(0x000000)
            self.display.drawPixel(x,y)
    
    def setWall(self, x,y):
     if x<self.displayRes and y <self.displayRes:
        #print(x)
        if self.world[int(x)][int(y)][0] == 0:
            self.world[int(x)][int(y)][0] = 2
            self.display.setColor(0xff0000)
            self.display.drawPixel(x,y)

    def isFrontFree(self):
        if self.l4.getValue() > self.distThr:
            return True
        else:
            return False