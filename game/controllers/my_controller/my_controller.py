from controller import Robot

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28
    
    # Initialize motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))  # Set to velocity control
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    # Initialize distance sensors
    sensors = []
    for i in range(8):
        sensor_name = 'ps' + str(i)
        sensor = robot.getDevice(sensor_name)
        sensor.enable(timestep)
        sensors.append(sensor)
    
    # Trémaux algorithm variables
    visited = {}  # Dictionary to store visited positions with visit count
    
    while robot.step(timestep) != -1:
        sensor_values = [sensor.getValue() for sensor in sensors]
        front_free = sensor_values[0] < 100 and sensor_values[7] < 100  # Check if front path is clear
        
        # Determine robot position (simplified for example)
        position = (robot.getPosition()[0], robot.getPosition()[1])
        if position not in visited:
            visited[position] = 0
        visited[position] += 1
        
        if visited[position] == 1:
            if front_free:
                # Move forward if front is clear and the path is less visited
                left_motor.setVelocity(max_speed)
                right_motor.setVelocity(max_speed)
            else:
                # Turn randomly
                left_motor.setVelocity(max_speed)
                right_motor.setVelocity(-max_speed)
        else:
            # Backtrack or turn around if this path is highly visited
            left_motor.setVelocity(-max_speed)
            right_motor.setVelocity(max_speed)
            
run_robot(Robot())
