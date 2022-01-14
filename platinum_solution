#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math


#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math



# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

# Initialize the motors and sensors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
direction_motor = Motor(Port.A)
side_sensor = UltrasonicSensor(Port.S1)
back_sensor = UltrasonicSensor(Port.S2)
color_sensor = ColorSensor(Port.S4)

robot = DriveBase(left_motor,right_motor,160,114)

depth = 0

dist_park = 0
a = 0 

val_col = color_sensor.reflection() #initialisation of the first value of the clor sensor to adjust the position of the robot 

while dist_park < 470 :  # while the distance of parking is below 47cm we continue the loop
    
    robot.drive(100,0)  # the robot will drive until we call the function robot.stop()

    
    while side_sensor.distance() < 180 : #and side_sensor.distance() > 10 : #when the side sensor detects an obstacle (<18cm) we stay in the loop to allow the robot to continue to drive  
        True
        

        if color_sensor.reflection() > val_col + 15  : # if the intial value of the sensor si above this value + 15 the robot is not going straight so we need to adjust it 

            direction_motor.run_angle(70, 7, then=Stop.HOLD, wait=True) # we set the direction motor to go to the left  by putting an angle of 7°
            direction_motor.run_angle(70, -7, then=Stop.HOLD, wait=True) # wwe set the direction motor to go back at the initial angle 
            
      
            
        if color_sensor.reflection() < val_col - 15 :
            direction_motor.run_angle(70, -7, then=Stop.HOLD, wait=True) # we set the direction motor to go to the left  by putting an angle of 7°
            direction_motor.run_angle(70, 7, then=Stop.HOLD, wait=True) # wwe set the direction motor to go back at the initial angle

        
        
    direction_motor.reset_angle(0) # we reset the direction motor to 0°

    robot.reset() # we reset the robot to reset all parameters like distance 

    dist_park = robot.distance() # we initiate the distance the robot will travel

    robot.drive(100,0)

    # while the robot doesnt detects any obstacle it will stay in the loop 
    # that will allow us to calculate the distance  between 2 obstacles
    while side_sensor.distance() > 180 : 
        
        
        # Robot adjustement with color sensor 
        if color_sensor.reflection() >= val_col + 15  :

            direction_motor.run_angle(70, 7, then=Stop.HOLD, wait=True)
            direction_motor.run_angle(70, -7, then=Stop.HOLD, wait=True)
            
            
        if color_sensor.reflection() <= val_col - 15 :

            direction_motor.run_angle(70, -7, then=Stop.HOLD, wait=True)
            direction_motor.run_angle(70, 7, then=Stop.HOLD, wait=True)

        # depth calculate just once ( doesnt work ) get false value 
        if depth == 0 :
            depth = side_sensor.distance()
        
      
        True

    dist_park = robot.distance() + 50 # distance calculate when the robot detects an obstacle (50 error adjustment)
 

    if dist_park >= 350 and dist_park <= 450 and depth >= 450 : # if the distance of park is below 45 cm and above 35 cm we will park in a perpendicular way 
        
        
        a=1 # put the variable at 1 that allow the program to go in the parallel method
        break # we go out of the loop

    print ()
robot.stop() 
robot.straight(100)


# if the distance calculate between two obstacles is above 47 cm so a=0 and we park parallely
if a == 0 :

    #---------------------Calculation for right distance and right position of the car---------------------------------------------#

    Rmin = 23.36    # the turning Radius of the circle done by the car using the steering angle 64 degrees

    sensor_value = side_sensor.distance(True) *0.1 # side sensor value 

    yi = sensor_value + 8.75 +  2 # initial lateral position; 8.75 = width/2 ; 2 = distance between sensor and wheel

    xi = 43 + 2.7 # initial longitudinal position aligned wheels + p(radius of wheel)

    yc_2 = yi - Rmin #  lateral position of the imaginary circle done with the turning radius

    yc_1 = Rmin # lateral position of the second imaginary circle done with the turning radius

    yt = (yc_1+yc_2)/2 # lateral position of the counter-steer point

    xc_1 = 4 # longitudinal position of the imaginary circle after the counter-steer

    xt = xc_1 + math.sqrt(pow((Rmin),2)-pow(abs(yt - yc_1),2)) # longitudinal position of the counter-steer point

    xs = 2*xt - xc_1 # longitudinal position of when the car need to steer 

    teta = math.acos((yc_1-yc_2)/(2*Rmin)) # angle between the (xs,ys) point and the (xt,yt) point

    steering_dis = 2*Rmin*teta # distance ²&of the steering 
    print( "xs",xs, " xi = ", xi )
    #-------------------------------------------------------------------------------------------------------------------------------#

    # adjust the position of the car with the initial position
    # calculation of the distance between the initial position and the steering point 
    if xs > xi :
        robot.stop()
        robot.settings(70)
        robot.straight(abs(xs-xi)*10 + +60 )
        robot.stop()
    elif xs < xi :
        robot.stop()
        robot.settings(-70)
        robot.straight(abs(xs-xi)*10 + 60 )
        robot.stop()
    elif xs == xi :
        robot.stop()


    direction_motor.run_angle(60, 64, then=Stop.HOLD, wait=True)# The direction motor is turning to the right angle 
    robot.stop()


    robot.reset()
    distance = robot.distance()

    #while the distance travel by the robot is above the steering distance we continue the loop 
    # we put a minus because the robot go backward
    while  distance >=  -((steering_dis-15)*10) :

        while back_sensor.distance() <= 60 : # if the back sensor detect an obstacle with a distance below 6 cm it will stop until there is no more obstacle 

            right_motor.stop()
            left_motor.stop()

            
        right_motor.run(-100) 
        left_motor.run(-90)

        distance = robot.distance()# calculate the distance treveled everytime 
        
   
    robot.stop()


    direction_motor.run_angle(60, -128, then=Stop.HOLD, wait=True)# the car is counter steering

    robot.reset()
    distance = robot.distance()
    robot.stop()

    while  distance >=  -((steering_dis-15)*10)  :

        while back_sensor.distance() <= 60 :
            right_motor.stop()
            left_motor.stop()
            ev3.screen.clear()
            ev3.screen.draw_text(40, 50, "STOP " +  str(side_sensor.distance()) )
        
        right_motor.run(-90)
        left_motor.run(-100)
        distance = robot.distance()



    robot.stop()
    direction_motor.run_angle(40, 64, then=Stop.HOLD, wait=True) # the direction motor is coming back at the initial position 
    robot.stop()
    direction_motor.run_angle(40, 15, then=Stop.HOLD, wait=True) # the direction motor is coming back at the initial position 
    robot.stop()
    robot.settings(70)
    robot.straight(20)
    direction_motor.run_angle(40, -15, then=Stop.HOLD, wait=True) # the direction motor is coming back at the initial position 
    direction_motor.reset_angle(0)
    
#-------------------------------------Perpendicular parking--------------------------------------------------------------------------#
if a==1 : 

 #---------------------Calculation for right distance and right position of the car---------------------------------------------#

    Rmin = 23.36    # the turning Radius of the circle done by the car using the steering angle 64 degrees

    sensor_value = side_sensor.distance(True) *0.1 # side sensor value 

    s= Rmin - sensor_value - 2 - 8.75 

    OD = math.sqrt(pow((Rmin-8.75),2)-pow(s,2))


    yi = sensor_value + 8.75 +  2 # initial lateral position; 8.75 = width/2 ; 2 = distance between sensor and wheel

    xi = 35 + 2.7 # initial longitudinal position aligned wheels + p(radius of wheel)

    yc_2 = yi - Rmin #  lateral position of the imaginary circle done with the turning radius


    yt = 0 # lateral position of the counter-steer point

    xc_1 = 16 # longitudinal position of the imaginary circle after the counter-steer

    xt = 14.5

    xs = OD + (35-xt) # longitudinal position of when the car need to steer 

    teta = 1.5708 # angle between the (xs,ys) point and the (xt,yt) point

    steering_dis = 2*Rmin*teta # distance ²&of the steering 
    print( "xs",xs, " xi = ", xi , "steering = ", steering_dis )
    #-------------------------------------------------------------------------------------------------------------------------------#

    # adjust the position of the car with the initial posi*tion
    # calculation of the distance between the initial position and the steering point 
    if xs > xi :
        robot.stop()
        robot.settings(70)
        robot.straight(abs(xs-xi)*10 - 20 )
        robot.stop()
    elif xs < xi :
        robot.stop()
        robot.settings(-70)
        robot.straight(abs(xs-xi)*10 -20  )
        robot.stop()
    elif xs == xi :
        robot.stop()


    direction_motor.run_angle(60, 64, then=Stop.HOLD, wait=True)# The direction motor is turning to the right angle 
    robot.stop()


    robot.reset()
    distance = robot.distance()
    print ( distance )

    print ("steering = ",(steering_dis-27)*10)


    while  distance >=  -((steering_dis-38)*10) :

        right_motor.run(-100)
        left_motor.run(-90)
        distance = robot.distance()
        print ("distance = ",distance, "steering = ",-(steering_dis-15)*10)

    robot.stop()
    direction_motor.run_angle(40, -64, then=Stop.HOLD, wait=True) # the direction motor is coming back at the initial position 
    robot.settings(-70)
    robot.straight(200)
    robot.stop()
