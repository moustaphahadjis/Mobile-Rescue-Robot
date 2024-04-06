import numpy as np

class Map:
    def __init__(self,robot, size):
        self.display = robot.getDevice('display')
        self.world =  np.zeros((size,size,1))

    def setTrack(self, x, y):
        print(x)
        self.world[int(x)][int(y)][0] = 1
        self.display.setColor(0xff0000)
        self.display.drawPixel(x,y)
    
    def setWall(self, x,y):
        print(x)
        self.world[int(x)][int(y)][0] = 2
        self.display.setColor(0x0000ff)
        self.display.drawPixel(x,y)
