"""test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor
from matplotlib import pyplot as plt
import numpy as np
from scipy import signal


TIME_STEP = 32
MAX_SPEED = 6.28 
# create the Robot instance.
robot = Supervisor()
    
#retriving and enabling all devices
leftMotor = robot.getDevice('wheel_left_joint')
rightMotor = robot.getDevice('wheel_right_joint')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)  
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(TIME_STEP)
lidar.enablePointCloud()
display = robot.getDevice('display')
compass = robot.getDevice('compass')
compass.enable(TIME_STEP)
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

# field of view for lidar divice
FOV = 4.19


angles = np.linspace(FOV/2, -FOV/2, num=667)

# function to convert world coordinates to pixle location
def world2map(xw,yw):
    # px = (xw - 0.1715) * 300/5.87
    # py = (0.0996 - yw) * 300/4.7
    px = (xw + 3) * 300/5.87
    py = (1.8 - yw) * 300/5.8
    
    return [px,py]
    
# grey scale mapp data structure
occ_map = np.zeros((300,300))

# marker for robot following path
marker = robot.getFromDef("marker").getField("translation")
# marker.setSFVec3f([0,0,0.2])

# chosen waypoints for the marker
WP = [(0.68, -1.48), (0.32,-2.85), (-0.88, -3.2), 
      (-1.73, -3.08), (-1.72, -1.46), 
      (-1.74, 0.1), (-0.05, 0.5)]


index = 0 # index for marker location
reached_end = False # bool for when robot finishes first circle


# get the time step of the current world.

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    # setting marker location
    marker.setSFVec3f([*WP[index], 0])

    # retriving world coordinates, orientation, and 
    # euclidian distance to marker
    xw = gps.getValues()[0]
    yw = gps.getValues()[1]
    theta=np.arctan2(compass.getValues()[0],compass.getValues()[1])
    rho = np.sqrt( (xw - WP[index][0]) ** 2 + (yw - WP[index][1]) ** 2)
    
    # calculating heading error
    alpha = np.arctan2( (WP[index][1]-yw), (WP[index][0]-xw) ) - theta
    if (alpha > np.pi):
        alpha -= 2 * np.pi
    if (alpha < -np.pi):
        alpha += 2 * np.pi
    
    # reseting marker location when robot gets
    # close enough to current location
    if (rho < 0.3):
        if (index == 6 or reached_end):
            index -= 1
            reached_end = True
        else:
            index += 1
    
    # setting red pixles for trajectory path
    [px, py] = world2map(xw,yw)
    if 0 <= px < 300 and 0 <= py < 300:
        display.setColor(0xFF0000)
        display.drawPixel(px,py)
    
    # retriving lidar measurements
    ranges = np.array(lidar.getRangeImage())
    ranges[ranges == np.inf] = 100
    


    # w_T_r  = np.array([[np.cos(theta), -np.sin(theta), xw * 0.202],
                      # [np.sin(theta), np.cos(theta), yw],
                      # [0,0,1]])
                      
    # accounting for lidar offset from robot center
    LIDAR_X = 0.202
    lidar_world_x = xw + np.cos(theta)*LIDAR_X
    lidar_world_y = yw + np.sin(theta)*LIDAR_X
    
    w_T_r = np.array([[np.cos(theta), -np.sin(theta), lidar_world_x],
                      [np.sin(theta),  np.cos(theta), lidar_world_y],
                  [0,              0,             1]])
                      
    X_i = np.array([ranges * np.cos(angles), ranges * np.sin(angles), np.ones((667,))])
    D = w_T_r @ X_i
    
    
    # setting grey scale pixles for lidar readings in world frame
    for d in D.transpose():
        px1, py1 = world2map(d[0], d[1])
    
        px1 = int(round(px1))
        py1 = int(round(py1))
    
        if 0 <= px1 < 300 and 0 <= py1 < 300:
            occ_map[py1, px1] = min(1.0, occ_map[py1, px1] + 0.01)
    
            v = int(occ_map[py1, px1] * 255)
    
            color = v * 256**2 + v * 256 + v
    
            display.setColor(color)
            display.drawPixel(px1, py1)
         
    
    # print(g)
    # print(modes[mode])
    print(f"xw: {xw}")
    print(f"yw: {yw}")
    print(f"px: {px}")
    print(f"py: {py}")
    # print(f"alpha: {alpha * (180/np.pi)}")
    # print(f"theta: {theta * (180/np.pi)}")
    # print(f"rho: {rho}")
    # print(f"px: {px}")
    # print(f"py: {py}")
    # print(f"error: {error}")
    # print(f"distance: {distance}")
    # print(f"rotation: {rotation * (180/np.pi)}")
    # print(len(ranges))
    # print(ranges[89])
    
    

    # leftMotor.setVelocity(phildot)
    # rightMotor.setVelocity(phirdot)
    
    #setting motor speeds with chosen gain values
    p1 = 5
    p2 = 3
    leftSpeed = max(min(- alpha*p1 + rho*p2,6.28),-6.28)
    rightSpeed = max(min(alpha*p1 + rho*p2,6.28),-6.28)
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    # when robot has finished its pathing exit loop
    if index == -2:
        break
        
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
print('finally got it to exit')


# reducing noise
occ_map[occ_map < 0.05] = 0

# plotting cmap
kernel = np.ones((34,34))
cmap = signal.convolve2d(occ_map,kernel,mode='same')
plt.figure(0)
plt.imshow(cmap)
plt.title("C-Map")

#plotting cspace
cspace = cmap > 0.9
plt.figure(1)
plt.imshow(cspace)
plt.title("C-Space")


plt.show()
