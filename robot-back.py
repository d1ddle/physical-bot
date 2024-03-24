from datetime import datetime, timedelta
from sr.robot3 import *
import math, time

## !! IMPORTANT !!                          #For Hz robot EACH WHEEL HAS 22.7cm CIRCUMFERENCE#
#DEFINE CONSTANTS HERE
MIN_DIST = 0.3                              #minimum distance a sensor should consider a collision.
COLLISIONOFFSET = 0.6                       #collision offset for arms when holding box moving towards marker target (spaceship)

#We have 5 types of object.
BOUNDARY = -1
ASTEROID = -2
EGG = -3
SPACESHIP = -4
MYSPACESHIP = -5

MARKER_FOV_THRESHOLD = 0.03

#Arena boundary	                0 - 27	
#Asteroid marker	        150-199	
#Egg Marker	                110	
#Spaceship Port Marker	        120-123	
#Spaceship Starboard Marker	125-128	

class MyRobot(Robot):
    
    def __init__(self):                     #Initialise Robot class
        super().__init__()
        self.status = "class initialised"
        self.mySpaceShip = 120 + self.zone
        self.log(("My Spaceship is ",self.mySpaceShip))
        self.clearCollision()               #assume starting position of robot is not in a collision state.    
        self.marker_target = ""             #current target - Asteroid, Spaceship or EGG!.
        self.update()                       #gets markers, sensor readings etc.
        self.visited_markers = []           #array of asteroids collected.
        self.stop_motors = False            #permit robot to move.
        
        
    #Gripper Functions
    def gripperMove(self,value):            #worker function for gripper
        self.servo_board.servos[0].position = value
        self.servo_board.servos[1].position = value
        self.log(("gripper moved to position",value))
        self.restInterupt(0.5)
        self.servo_board.servos[0].position = None
        self.servo_board.servos[1].position = None
        self.restInterupt(0.5)
        self.servo_board.servos[1].position = None
    def gripperOpen(self):                  # open gripper fully
        self.gripperMove(-1)
        self.log("gripper fully open")
    def gripperDump(self):                  # dump asteroid in spaceship
        self.log("gripper dump")
        self.moveArm(1)                     # arm up
        self.restInterupt(1)                # rests for 1 second
        self.startRobot()                   # starts the robot
        self.moveRobot(1, 2)              # move forward 0.3 speed, 1 second
        self.stopRobot()                    # stops the robot
        self.gripperOpen()                  # pray
        self.visited_markers.append(self.marker_target) #mark box as territory
    def gripperClose(self):                 # close gripper fully
        self.gripperMove(1)
        self.log("gripper fully closed")
        
    #Arm Functions
    def moveArm(self, decimal):             #move the arm up/down on % 
        # inputs between 0 and 1, 0 = down, 1 = up
        self.log(("Raising arm ",decimal))
        self.servo_board.servos[2].position = decimal
        self.restInterupt(0.1)

    def isBoundary(self, marker_id):
        if marker_id >= 0 and marker_id <= 27:
            return True
        else:
            return False

    #Wait, Utility & Interupt functions.
    def getTarget(self, targettype):
        #updates the class.marker_tartget to the closest marker of desired type 
        #BOUNDARY = 0 #ASTEROID = 1 #EGG = 2 #SPACESHIP = 3
        self.marker_target = ""
        for temp_marker in self.markers: #find closed marker of desired type
            if targettype >= 0 and temp_marker.id == targettype:    #reused targettype as explicit index
                                                                    #by making object types negative
                self.marker_target = temp_marker    #find specific marker_id
                break
            elif targettype == MYSPACESHIP and temp_marker.id == self.mySpaceShip:
                self.marker_target = temp_marker    #we found our spaceship
                self.log("Found mySpaceShip")
                break
            elif targettype == SPACESHIP and temp_marker.id >= 120 and temp_marker.id <= 123:
                self.marker_target = temp_marker    #we found a spaceship
                break
            elif targettype == ASTEROID and temp_marker.id >= 150 and temp_marker.id <=199:
                self.marker_target = temp_marker    #we found asteroid
                break
            elif targettype == EGG and temp_marker.id == 110: 
                self.marker_target = temp_marker    #we found egg!!!!!!!
                break
            elif targettype == BOUNDARY and temp_marker.id >= 0 and temp_marker.id <= 27:
                self.marker_target = temp_marker
                break
        self.log(("Found target from camera ",temp_marker.id))        
        return (self.marker_target != "")
    
    def restInterupt(self, sleep):
        # do stuff in here to check for errors.
        self.log(("Sleep:restInterupt (seconds) ",sleep))
        self.sleep(sleep)
    
    def clearCollision(self):               #reset collision detection settings
        self.front_collision = False
        self.back_collision = False
        self.rear_collision = False
        self.right_collision = False
        self.left_collision = False
        self.gripper_collision_left = False
        self.gripper_collision_right = False
    def frontCollision(self):
        return (self.front_collision or self.gripper_collision_left or self.gripper_collision_right)
    
    def updateMarkers(self):
        self.markers = self.camera.see()    #ensure we take scene to evaluate next nearest asteroid, spaceship or egg!
        self.log("Markers updated.")
    def update(self):
        self.restInterupt(0.5)
        #gather sensor data from Ultrasound
        #if the robot is NOT moving AND the ARM is down then we can call 'see' and get a new scene. 
        #Hz robot will need these remapped due to different Ultrasound sensor array.
        #Ultra Sound Sensors for simulated robot as below.
        #A0 Front Left INPUT
        #A1 Front Right INPUT
        #A2 Left INPUT
        #A3 Right INPUT
        #A4 Front INPUT
        #A5 Back INPUT

        collision_offset = 0
        if self.marker_target != "" and self.marker_target.id == self.mySpaceShip or self.marker_target != "" and self.isBoundary(self.marker_target.id):
            collision_offset = COLLISIONOFFSET
        
        self.gripper_sensor_left_distance = round(self.arduino.pins[A0].analog_read(),2)
        self.gripper_sensor_right_distance = round(self.arduino.pins[A1].analog_read(),2)
        self.front_sensor_distance = round(self.arduino.pins[A4].analog_read(),2)
        self.back_sensor_distance = round(self.arduino.pins[A5].analog_read(),2)
        self.right_sensor_distance = round(self.arduino.pins[A3].analog_read(),2)
        self.left_sensor_distance = round(self.arduino.pins[A2].analog_read(),2)
        #detect here if a sensor has less than MIN_DIST, set class variable accordingly.
        if self.gripper_sensor_left_distance < MIN_DIST + collision_offset:
            self.gripper_collision_left = True
        if self.gripper_sensor_right_distance < MIN_DIST + collision_offset:
            self.gripper_collision_right = True
        if self.front_sensor_distance < MIN_DIST + collision_offset:
            self.front_collision = True
        if self.back_sensor_distance < MIN_DIST + collision_offset:
            self.back_collision = True
        if self.right_sensor_distance < MIN_DIST + collision_offset:
            self.right_collision = True
        if self.left_sensor_distance  < MIN_DIST + collision_offset:
            self.left_collision = True
        if self.front_collision or self.back_collision or self.left_collision or self.right_collision or self.gripper_collision_left or self.gripper_collision_right : 
            self.log("Collision detected.")
            self.stopRobot()                   
        if self.motor_board.motors[0].power == 0 and self.motor_board.motors[1].power == 0:
            self.restInterupt(0.1)
            self.updateMarkers()
            self.log(("Updated Sensors, Camera active, Measure", self.front_sensor_distance, self.gripper_sensor_right_distance, self.gripper_sensor_left_distance))
        else:
            self.log(("Updated Sensors, Camera Inactive, Measure", self.front_sensor_distance, self.back_sensor_distance, self.markers[0].position.distance/1000))

    def delay(self, angle):
        ## determines relationship between angle of turn and duration needed at 0.1 power
        time = abs(((angle*100)/6.429)/1000)
        self.log(("TIME: ", time))
        if time > 0:
            self.restInterupt(time)
            self.motor_board.motors[0].power = 0
            self.motor_board.motors[1].power = 0
            self.log(("Slept for", time))

    def log(self,message):                  #log info to debug console
        now = datetime.now()                #current date and time
        date_time = now.strftime("%m%d%Y %H:%M:%S")
        print(date_time,"|",message)	
        
    #Robot mobility functions    
    def stopRobot(self):                    #used to force robot to cease movement from wheels.          
        self.stop_motors = True
        self.motor_board.motors[0].power = 0
        self.motor_board.motors[1].power = 0

    def startRobot(self):
        self.stop_motors = False
    
    def rotateRobot(self, angle, delay):
        #Accept positive or negative angle in degrees
        if angle < 0:                       #left
            self.log(("Rotate Left", angle))
            self.motor_board.motors[0].power = -0.1
            self.motor_board.motors[1].power = 0.1
        elif angle > 0:                     #right
            self.log(("Rotate Right", angle))
            self.motor_board.motors[0].power = 0.1
            self.motor_board.motors[1].power = -0.1    
        else:
            print("Angle is zero, wyd?")
        robot.restInterupt(delay)
        self.motor_board.motors[0].power = 0
        self.motor_board.motors[1].power = 0

    def moveRobot(self, speed, duration):   #Move at specified speed and for duration
        if not self.stop_motors:
            starttime = datetime.now()      #current date and time
            endtime = starttime + timedelta(seconds=duration)
            now_time = datetime.now()
            while now_time < endtime and not self.stop_motors:
                self.update()
                self.motor_board.motors[0].power = speed
                self.motor_board.motors[1].power = speed
                self.log(("moveRobot speed:", speed," duration ", duration))
                self.restInterupt(0.1)
                now_time = datetime.now()
            self.stopRobot()


robot = MyRobot()
done = False

def findObject(Object):
    delay = 0.1
    robot.stopRobot()
    robot.update() #measure distance, we need to ensure markers scanned
    robot.clearCollision()
    if robot.getTarget(Object):
        while not robot.frontCollision():
            # case 1: box is in front (any orientation, but position = 0)
            if abs(robot.marker_target.position.horizontal_angle) < MARKER_FOV_THRESHOLD: # do until robot is aligned # and front_sensor < (3/4 * d)
                robot.startRobot()
                robot.moveRobot(0.3, (robot.marker_target.position.distance/1000)/0.3)
            # case 2: box is not directly infront
            else:  
                robot.getTarget(Object)
                robot.rotateRobot(robot.marker_target.position.horizontal_angle, delay)
                delay = delay/1.1
            robot.stopRobot()
            robot.update()
        return True
    else:
        robot.rotateRobot(math.pi/6, 0.5)
        return findObject(Object)


if findObject(ASTEROID):
    robot.gripperClose()
    robot.moveArm(0.02)
    robot.clearCollision()
    print("ONE")
    if findObject(2):
        print("TWO")
        robot.clearCollision()
        if findObject(MYSPACESHIP):
            robot.gripperDump()
    else:
        print("THREE")
