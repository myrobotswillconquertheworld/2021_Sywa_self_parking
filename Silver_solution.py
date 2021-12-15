#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math
from fonction_lego import park_method


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

robot = DriveBase(left_motor,right_motor,160,114)



dist_park = 0


while dist_park < 470 :  # tant que la distance de stationnement n'est pas au minimum égal à 47 cm le robot continue la boucle 
    robot.drive(100,0   )

   
    
    while side_sensor.distance() < 180 : # lorsque le robot detecte un obstacles a moin,s de 18cm sur le coté il  reste dans la boucle pour continuer d'avancer  
        True
        ev3.screen.clear()
        ev3.screen.draw_text(40, 50, "SIIIDE " +  str(side_sensor.distance()) )

        if side_sensor.distance() >= 75 and side_sensor.distance() <= 180  :  #si la valezur du capteur est compris entre 7.5 et 18 cm la voiture tourne cers la gauche 
            direction_motor.run_angle(70, 7, then=Stop.HOLD, wait=True)
            #wait(800)
            direction_motor.run_angle(70, -7, then=Stop.HOLD, wait=True)
            ev3.screen.clear()
            ev3.screen.draw_text(40, 50, "GAUCHE " +  str(side_sensor.distance()) )

          
            
        if side_sensor.distance() <= 65 : # si la valeur est inférieur à 6.5cm on tourne un peu vers la droite
            direction_motor.run_angle(70, -7, then=Stop.HOLD, wait=True)
            #wait(800)  
            direction_motor.run_angle(70, 7, then=Stop.HOLD, wait=True)
            ev3.screen.clear()
            ev3.screen.draw_text(40, 50, "DROITE " + str(side_sensor.distance()) )

           

    
    direction_motor.reset_angle(0) # on reset l'angle du moteur pour aller touyt droit 


    robot.reset() # on appelle la fonction resest pour réinitialiser la valeur de la distance
    dist_park = robot.distance() 


    while side_sensor.distance() > 180 : # tant que le capteur ne detecte pas d'obstacle il reste dans la boucle afin de calculer la distance
        True
    dist_park = robot.distance() + 50 # on récuoere la distance calculé
    ev3.screen.clear()
    ev3.screen.draw_text(40, 50, dist_park)
    

    print ()
robot.stop() # on arrete la voiture
robot.straight(100) # on aligne la voiture avec la place de parking 

 #---------------------Calculation for right distance and right position of the car---------------------------------------------#

Rmin = 23.36    # the turning Radius of the circle done by the car using the steering angle 64 degrees

sensor_value = side_sensor.distance(True) *0.1 # side sensor value 

yi = sensor_value + 8.75 +  2 # initial lateral position ; 8.75 = width/2 ; 2 = distance between sensor and wheel

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

# adjust the position of the car with the initial posi*tion
# calculation of the distance between the initial position and the steering point 
if xs > xi :
    robot.stop()
    robot.settings(70)
    robot.straight(abs(xs-xi)*10 )
    robot.stop()
elif xs < xi :
    robot.stop()
    robot.settings(-70)
    robot.straight(abs(xs-xi)*10 )
    robot.stop()
elif xs == xi :
    robot.stop()


direction_motor.run_angle(60, 64, then=Stop.HOLD, wait=True)# The direction motor is turning to the right angle 
robot.stop()


robot.reset()
distance = robot.distance()
print ( distance )

while  distance >=  -((steering_dis-15)*10) :

    right_motor.run(-100)
    left_motor.run(-90)
    distance = robot.distance()
    print ("distance = ",distance, "steering = ",-(steering_dis-15)*10)


robot.stop()


direction_motor.run_angle(60, -128, then=Stop.HOLD, wait=True)# the car is counter steering

robot.reset()
distance = robot.distance()
robot.stop()

while  distance >=  -((steering_dis-15)*10)  :
    right_motor.run(-90)
    left_motor.run(-100)
    distance = robot.distance()



robot.stop()
direction_motor.run_angle(40, 64, then=Stop.HOLD, wait=True) # the direction motor is coming back at the initial position 
robot.stop()
direction_motor.run_angle(40, 15, then=Stop.HOLD, wait=True) # the direction motor is coming back at the initial position 
robot.stop()
robot.settings(70)
robot.straight(75)
direction_motor.run_angle(40, -15, then=Stop.HOLD, wait=True) # the direction motor is coming back at the initial position 
direction_motor.reset_angle(0)
