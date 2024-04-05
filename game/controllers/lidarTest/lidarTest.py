"""lidarTest controller."""


from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()

while robot.step(timestep) != -1:
    print((lidar.getRangeImage()))
    #aaa = lidar.getPointCloud()
    #print(aaa[1])
    print('----------------------------------------')
    pass

# Enter here exit cleanup code.
