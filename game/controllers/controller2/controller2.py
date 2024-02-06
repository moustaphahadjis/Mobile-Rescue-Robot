from controller import Robot

timeStep = 32            # Set the time step for the simulation
max_velocity = 6.28      # Set a maximum velocity time constant

robot = Robot()

wheel_left = robot.getDevice("left wheel motor")   # Create an object to control the left wheel
wheel_right = robot.getDevice("right wheel motor") # Create an object to control the right wheel

leftSensors = []      # Create an empty list to store the left sensor values 
rightSensors = []     # Create an empty list to store the right sensor values
frontSensors = []     # Create an empty list to store the front sensor values

camera = robot.getDevice("camera")
camera.enable(timeStep)

frontSensors.append(robot.getDevice("ps7"))
frontSensors[0].enable(timeStep)
frontSensors.append(robot.getDevice("ps0"))
frontSensors[1].enable(timeStep)

rightSensors.append(robot.getDevice("ps1"))
rightSensors[0].enable(timeStep)
rightSensors.append(robot.getDevice("ps2"))
rightSensors[1].enable(timeStep)

leftSensors.append(robot.getDevice("ps5"))
leftSensors[0].enable(timeStep)
leftSensors.append(robot.getDevice("ps6"))
leftSensors[1].enable(timeStep)

#        [left wheel speed, right wheel speed]
speeds = [max_velocity,max_velocity]

wheel_left.setPosition(float("inf"))
wheel_right.setPosition(float("inf"))


def turn_right():
    #set left wheel speed
    speeds[0] = 0.6 * max_velocity
    #set right wheel speed
    speeds[1] = -0.2 * max_velocity

def turn_left():
    #set left wheel speed
    speeds[0] = -0.2 * max_velocity
    #set right wheel speed
    speeds[1] = 0.6 * max_velocity

def spin():
    #set left wheel speed
    speeds[0] = 0.6 * max_velocity
    #set right wheel speed
    speeds[1] = -0.6 * max_velocity
    
    
while robot.step(timeStep) != -1:
    speeds[0] = max_velocity
    speeds[1] = max_velocity

    # Check left and right sensors to avoid walls,
    for i in range(2):                             # Loop through the two sensors for each of the left and right
        #for sensors on the left, either
        if leftSensors[i].getValue() > 0.05:
            turn_right()                           # We see a wall on the left, so turn right away from the wall
        #for sensors on the right, either
        elif rightSensors[i].getValue() > 0.05:
            turn_left()
    
    #for both front sensors
    if frontSensors[0].getValue() > 80 and frontSensors[1].getValue() > 0.05:
        spin()

    wheel_left.setVelocity(speeds[0])              # Send the speed values we have chosen to the robot
    wheel_right.setVelocity(speeds[1])