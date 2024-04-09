import numpy as np
import math
class Map:
    def __init__(self,robot, timestep):
        self.display = robot.getDevice('display')
        

        self.l1 = robot.getDevice('laser1')
        self.l2 = robot.getDevice('laser2')
        self.l3 = robot.getDevice('laser3')

        self.l1.enable(timestep)
        self.l2.enable(timestep)
        self.l3.enable(timestep)

        self.lasers = [self.l1, self.l2, self.l3]

        self.displayRes = 1000
        self.scale = 0.5

        self.world =  np.zeros((self.displayRes, self.displayRes,1))

    def detectWall(self, currentPos, orientation):
        for i in range(len(self.lasers)):
            angle = math.radians(i*90 + orientation)
            dist = self.lasers[i].getValue()

            y = dist* math.sin(angle) 
            x = dist* math.cos(angle) 
            #print(f'[{x, y}]')
            self.setWall(x*self.displayRes*self.scale + currentPos[0], -y*self.displayRes*self.scale + currentPos[1] )
            
            
    def setTrack(self, x, y):
        #print(x)
        if self.world[int(x)][int(y)][0] == 0:
            self.world[int(x)][int(y)][0] = 1
            self.display.setColor(0x93A8B3)
            self.display.drawPixel(x,y)
    
    def setWall(self, x,y):
        #print(x)
        if self.world[int(x)][int(y)][0] == 0:
            self.world[int(x)][int(y)][0] = 2
            self.display.setColor(0x48D1B8)
            self.display.drawPixel(x,y)
