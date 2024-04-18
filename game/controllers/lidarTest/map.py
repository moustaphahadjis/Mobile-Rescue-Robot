import numpy as np
import math
class Map:
    def __init__(self,robot, timestep, initGPS):
        self.display = robot.getDevice('display')
        

        self.l1 = robot.getDevice('laser1')
        self.l2 = robot.getDevice('laser2')
        self.l3 = robot.getDevice('laser3')
        self.l4 = robot.getDevice('laser4')
        self.l5 = robot.getDevice('laser5')
        self.l6 = robot.getDevice('laser6')
        self.l7 = robot.getDevice('laser7')
        self.l8 = robot.getDevice('laser8')
        self.l9 = robot.getDevice('laser9')

        self.l1.enable(timestep)
        self.l2.enable(timestep)
        self.l3.enable(timestep)
        self.l4.enable(timestep)
        self.l5.enable(timestep)
        self.l6.enable(timestep)
        self.l7.enable(timestep)
        self.l8.enable(timestep)
        self.l9.enable(timestep)

        self.lasers = [self.l1, self.l2, self.l3, self.l4,self.l5,self.l6,self.l7, self.l8, self.l9]
        self.angles =[0,30,60,75,105,120,150,180,90]
        self.displayRes = 1000
        self.scale = 0.5

        self.world =  np.zeros((self.displayRes, self.displayRes,1))
        self.distThr = 0.1
        self.curr = (0,0)
        self.pos = [self.displayRes/2, self.displayRes/2]
        self.initGPS = initGPS

        #self.initDisplay()

    def initDisplay(self):
       for i in range(self.displayRes):
          for j in range(self.displayRes):
            self.display.setColor(0xffffff)
            self.display.drawPixel(i,j)

    def detectVictimLoc(self, currentPos, orientation):
            
            angle = math.radians(270 + orientation +90)
            dist = self.lasers[3].getValue()

            y = dist* math.sin(angle) 
            x = dist* math.cos(angle) 
            #print(f'[{x, y}]')
            return x*self.displayRes*self.scale + currentPos[0],-y*self.displayRes*self.scale + currentPos[1]
            
    
    def detectWall(self, currentPos, orientation):
        for i in range(len(self.lasers)):
            angle = math.radians(self.angles[i] + orientation +90)
            dist = self.lasers[i].getValue()

            y = dist* math.sin(angle) 
            x = dist* math.cos(angle) 
            #print(f'[{x, y}]')
            self.setWall(x*self.displayRes*self.scale + currentPos[0], -y*self.displayRes*self.scale + currentPos[1] )
    def mapping(self,orientation, gps):
       currentPos = self.drawTrack(gps)
       self.detectWall(currentPos,orientation)
            
    def setTrack(self, x, y):
     self.curr = (int(x),int(y))
     if x<self.displayRes and y <self.displayRes:
        #print(x)
        if self.world[int(x)][int(y)][0] == 0:
            self.world[int(x)][int(y)][0] = 1
            self.display.setColor(0x00ff00)
            self.display.drawPixel(x,y)
    
    def setWall(self, x,y):
     if x<self.displayRes and y <self.displayRes:
        #print(x)
        if self.world[int(x)][int(y)][0] == 0 or self.world[int(x)][int(y)][0] == 5:
            self.world[int(x)][int(y)][0] = 2
            self.display.setColor(0xff0000)
            self.display.drawPixel(x,y)

    def setPath(self, path):
       for i in range (len(path)):
          self.world[path[i][0]][path[i][1]] = 5
          self.display.setColor(0xffffff)
          self.display.drawPixel(path[i][0],path[i][1])
    def removePath(self, path):
       for i in range (len(path)):
          self.world[path[i][0]][path[i][1]] = 0
          self.display.setColor(0x000000)
          self.display.drawPixel(path[i][0],path[i][1])


    def securePath(self, path):
        res = True
        for i in range(len(path)):
            if self.world[path[i][0]][path[i][1]] !=5:
                res = False
                break
        
        return res
             

    def drawTrack(self, gps):
        print(gps)
        x0 = -gps[0] + self.initGPS[0]
        y0 = -gps[2] + self.initGPS[2]
        print(x0)

        x = x0*self.displayRes*self.scale + self.pos[0]
        y = y0*self.displayRes*self.scale + self.pos[1]
        self.setTrack(x,y)
        return (x,y)
       

    
       
