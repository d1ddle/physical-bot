from datetime import datetime, timedelta
from sr.robot3 import *
import math, time, serial, serial.tools.list_ports, sys

## Each wheel has 22.7cm Circumference ##
## Robot max speed at 60rpm is 180mm/s
MIN_DIST = 10
COLLISION_OFFSET = 5
SPACESHIP_OFFSET = 3
BOUNDARY_OFFSET = 6

#check if turned 360 when finding object; if object spaceship move back, if object ateroid look for egg; else look for spaceship & take spaceship/asteroid

#We have 5 types of object.
#Arena boundary	                0 - 27	
#Asteroid marker	        150-199	
#Egg Marker	                110	
#Spaceship Port Marker	        120-123	
#Spaceship Starboard Marker	125-128
BOUNDARY = -1
ASTEROID = -2
EGG = -3
SPACESHIP = -4
MYSPACESHIP = -5

ARMVACBOARD = "SR0XK1E"
WHEELBOARD = "SR0UF7"

FIRST_MARKER_FOV_THRESHOLD = 0.02     #0.03 #Threshold for alignment to marker in radians
SECOND_MARKER_FOV_THRESHOLD = 0.010
TURN_CONSTANT = 0.01325         #0.0155, #0.01111, #0.0132
SMOOTH_TURN_CONSTANT = 0.023
TURN_SPEED = 1                  #0.1 / 0.3
SCAN_ANGLE = 40                 #previously 35 deg
SENSOR_ERROR_THRESHOLD = 3
PIVOT_ARM_OFFSET = 37           #must be in mm!! origianally 37
CAMERA_ARM_OFFSET = 77          #must be in mm!!
PIVOT_CAMERA_OFFSET = 40        #must be in mm!!
GROUND_CAMERA_HEIGHT = 300      #height from centre of box on ground to camera
MINIMUM_CAMERA_RANGE = 750     #orig 1000, 450
ARM_MOVE_TIME = 1               #calibrate the time taken for arm to roughly move completely up/down
VERBOSELOG = False
LOGGING = False
BAUD_RATE = 921600
ARDUINO_SN = '75230313833351619111'
ARM_CAMERA_OFFSET_MM = 80

#music note frequs
CS4 = 277
FS4 = 370
GS4 = 415
B4 = 494
AS4 = 466
CS5 = 554
DS4 = 311
FS5 = 740
A4 = 440
A5 = 880
E5 = 659
D5 = 587
D4 = 294
ES5 = 622
F5 = 698

class MyRobot(Robot):
    def __init__(self):
        self.tune = False
        self.mii_tune = False
        self.speed = 0.3
        self.ground_distance = 0
        self.camera_fail = 0
        self.clearCollision()
        self.marker_target = ""
        self.stop_motors = True
        self.undocking = False
        self.arrived = False
        self.markers = []
        self.visited_markers = []
        
        self.armIsUp = False
        self.armIsDown = False
        self.front_sensor_distance = 600.0  #init sensor values
        self.back_sensor_distance = 600.0
        self.right_sensor_distance = 600.0
        self.left_sensor_distance = 600.0
        self.gripper_sensor_distance = 600.0
        self.top_endstop = False
        self.bottom_endstop = False
        self.left_wheel = 0.0
        self.right_wheel = 0.0
        self.arduino_activated = True
        self.packet_fail = 0
        
        if self.arduino_activated:
            self.ser = self.find_arduino(ARDUINO_SN)
        super().__init__(wait_for_start = True, trace_logging=True, debug=False, ignored_arduinos=[ARDUINO_SN], raw_ports=[(ARDUINO_SN, BAUD_RATE)])
        
        self.voltage = self.power_board.battery_sensor.voltage
        self.current = self.power_board.battery_sensor.current
        self.log(("VOLTAGE: ", self.voltage, "CURRENT: ", self.current))

        if self.voltage < 10.9:
            self.log(("BATTERY NEARLY FLAT", self.voltage, "V"))
            self.power_board.piezo.buzz(Note.B6, 0.5, blocking=True)
            self.power_board.piezo.buzz(Note.D6, 0.5)
            sys.exit()
        if self.current < 0.3 or self.current > 0.7:
            self.log(("INCORRECT BATTERY CURRENT!! ABORT!!!"))
            self.power_board.piezo.buzz(Note.B6, 0.5, blocking=True)
            self.power_board.piezo.buzz(Note.D6, 0.5)
            sys.exit()
        
        self.mySpaceShip = 120 + self.zone
        self.log(("My Spaceship is ",self.mySpaceShip))
        #self.armInit()
        self.readArduino("X")               #reset wheels
        self.readArduino("U")               #sensor ultimatum init
        self.update(True)                       #gets markers, sensor readings etc.

        if self.tune:
            if self.mii_tune:
                self.power_board.piezo.buzz(FS4, 0.4, blocking=True)
                self.power_board.piezo.buzz(A4, 0.2, blocking=True)
                self.power_board.piezo.buzz(CS5, 0.4, blocking=True)
                self.power_board.piezo.buzz(A4, 0.4, blocking=True)
                self.power_board.piezo.buzz(FS4, 0.2, blocking=True)
                self.power_board.piezo.buzz(D4, 0.15, blocking=True)
                self.power_board.piezo.buzz(D4, 0.15, blocking=True)
                self.power_board.piezo.buzz(D4, 0.2, blocking=True)
            else:
                for i in range(0,4):
                    if i == 0 or i == 1:
                        self.power_board.piezo.buzz(CS4, 0.2, blocking=True)
                    elif i == 2 or i == 3:
                        self.power_board.piezo.buzz(DS4, 0.2, blocking=True)
##                    elif i == 4 or i == 5:
##                        self.power_board.piezo.buzz(FS4, 0.2, blocking=True)
                    self.power_board.piezo.buzz(CS5, 0.2, blocking=True)
                    self.power_board.piezo.buzz(GS4, 0.2, blocking=True)
                    self.power_board.piezo.buzz(FS4, 0.2, blocking=True)
                    self.power_board.piezo.buzz(FS5, 0.2, blocking=True)
                    self.power_board.piezo.buzz(GS4, 0.2, blocking=True)
                    self.power_board.piezo.buzz(F5, 0.2, blocking=True)
                    self.power_board.piezo.buzz(GS4, 0.2, blocking=True)

    def find_arduino(self, serial_number):
        for pinfo in serial.tools.list_ports.comports():
            if pinfo.serial_number == serial_number:
                return serial.Serial(pinfo.device)
            self.log(("Tried Connection; ", pinfo.serial_number))
        raise IOError("Could not find an arduino - is it plugged in?")

    #arm & vacuum gripper functions
    def armInit(self):                      #checks arm endstops function
        self.armDown()
        self.armUp()
##        self.power_board.piezo.buzz(Note.B6, 0.5, blocking=True)
##        self.power_board.piezo.buzz(Note.D6, 0.5)
        
    def movearmUp(self, decimal):         #move arm percentage up assuming start pos is at top
        self.motor_boards[ARMVACBOARD].motors[1].power = -0.2
        self.restInterupt(decimal * ARM_MOVE_TIME_UP)
        self.motor_boards[ARMVACBOARD].motors[1].power = 0
        
    def movearmHalfPos(self, decimal):         #move arm percentage up assuming start pos is at top
        self.motor_boards[ARMVACBOARD].motors[1].power = 1.0
        self.restInterupt(decimal * 0.35)
        self.motor_boards[ARMVACBOARD].motors[1].power = 0

    def armUp(self):               #move arm up until endstop
        self.motor_boards[ARMVACBOARD].motors[1].power = 1
        #self.top_endstop = self.arduino.command("T")
        while self.top_endstop == False:
            #self.top_endstop = self.arduino.command("T")
            self.readArduino("T")
            print(self.data, self.top_endstop)
            self.sleep(0.2)
        self.motor_boards[ARMVACBOARD].motors[1].power = 0
        self.power_board.piezo.buzz(Note.D6, 0.5)
        self.log("ARM AT TOP")
        self.armIsUp = True
        self.armIsDown = False
        self.bottom_endstop = False

    def armDown(self):                        #move arm down until endstop
        self.motor_boards[ARMVACBOARD].motors[1].power = -0.5
        #self.bottom_endstop = self.arduino.command("B")
        while self.bottom_endstop == False:
            #self.bottom_endstop = self.arduino.command("B")
            self.readArduino("B")
            self.sleep(0.2)
        self.motor_boards[ARMVACBOARD].motors[1].power = 0
        self.power_board.piezo.buzz(Note.C6, 0.5)
        self.log("ARM AT BOTTOM")
        self.armIsUp = False
        self.armIsDown = True
        self.top_endstop = False

    def pickBox(self):                                     #grab a box
        self.motor_boards[ARMVACBOARD].motors[0].power = 1      #set vacuum on, must come before armUp
        self.armDown()
        self.motor_boards[ARMVACBOARD].motors[1].power = 0.2    #continue to add rotational pressure
        self.restInterupt(1.7)

    def letGoBox(self):
        self.motor_boards[ARMVACBOARD].motors[0].power = 1
        self.armUp()
        self.motor_boards[ARMVACBOARD].motors[0].power = 0        #let go

    def dropBox(self):                  # dump asteroid in spaceship
        self.stopRobot()
        self.log("gripper dump")
        self.letGoBox()                  # pray
        self.restInterupt(10)            # WAIIIIIITTTT !!!
        self.undocking = True
        self.clearCollision()
        self.startRobot()
        self.clearCollision()
        self.moveRobot(-1, 2)           # move away from spaceship after drop
        self.visited_markers.append(self.marker_target) #mark box as territory
        self.marker_target = ""
        self.undocking = False
        self.startRobot()
        self.rotateRobot(-90)
        self.armUp()

    #utility functions
    def calculateSpeed(self, distance):             #calculate speed for moving faster over longer distances
        if distance > 2.5:
            #self.log("Speed set to 1 WARP factor 9")
            self.speed = 0.8
        elif distance > 1 and distance < 2.5:
            #self.log("Speed set to WARP factor 1")
            self.speed = 0.5
        else:
            #self.log("Speed set to full Impulse...")
            self.speed = 0.3

    def isBoundary(self, marker_id):                #checks if marker is a boundary
        if marker_id >= 0 and marker_id <= 27:
            return True
        else:
            return False

    def getTarget(self, targettype):        #requires an update beforehand!!
        self.log(("getTarget start"))
        #updates the class.marker_target to the closest marker of desired type 
        #BOUNDARY = 0 #ASTEROID = 1 #EGG = 2 #SPACESHIP = 3
        self.calculateSpeed(self.front_sensor_distance)      #forward fast!

        result = False
        self.marker_target = ""
        for temp_marker in self.markers: #find closed marker of desired type
            if not temp_marker in self.visited_markers:
                if targettype >= 0 and temp_marker.id == targettype:    #reused targettype as explicit index
                    result = True                                       #by making object types negative
                    self.marker_target = temp_marker    #find specific marker_id
                    break
                elif targettype == MYSPACESHIP and temp_marker.id == self.mySpaceShip:
                    result = True
                    self.marker_target = temp_marker    #we found our spaceship
                    self.log("Found mySpaceShip")
                    break
                elif targettype == SPACESHIP and temp_marker.id >= 120 and temp_marker.id <= 123:
                    result = True
                    self.marker_target = temp_marker    #we found a spaceship
                    break
                elif targettype == ASTEROID and temp_marker.id >= 150 and temp_marker.id <=199:
                    result = True
                    self.marker_target = temp_marker    #we found asteroid
                    break
                elif targettype == EGG and temp_marker.id == 110: 
                    result = True
                    self.marker_target = temp_marker    #we found egg!!!!!!!
                    break
                elif targettype == BOUNDARY and temp_marker.id >= 0 and temp_marker.id <= 27:
                    result = True
                    self.marker_target = temp_marker
                    break
                else:
                    self.log(("Target ", targettype, " not equal to temp_marker.id ", temp_marker.id))
        
        if result == True:
            self.log(("Found target from camera ",temp_marker.id))
            try:
                self.ground_distance = math.sqrt((self.marker_target.position.distance**2) - (GROUND_CAMERA_HEIGHT**2))
            except AttributeError:
                self.log(("getTarget requires an update(True) call beforehand"))
        else:
            self.log(("NOT found from camera where TARGETTYPE Value = ",targettype))
        return result #(self.marker_target != "")

    def restInterupt(self, sleep):
        # do stuff in here to check for errors and read motor encoders.
        self.log(("Sleep:restInterupt (seconds) ",sleep))
        self.sleep(sleep)
        #self.calculateSpeed(self.front_sensor_distance)      #forward fast!

    def log(self,message):                  #log info to debug console
        if LOGGING:
            now = datetime.now()                #current date and time
            date_time = now.strftime("%m%d%Y %H:%M:%S")
            x = ""
            for elem in message:
                x += str(elem)
            print(date_time,"|",x)

    def log2(self, *messages):
        print(datetime.now().strftime("%m%d%Y %H:%M:%S"),"|", [x for x in messages])
    
    def clearCollision(self):               #reset collision detection settings
        self.front_collision = False
        self.back_collision = False
        self.rear_collision = False
        self.right_collision = False
        self.left_collision = False
        self.log("Collisions cleared.")

    def frontCollision(self):
        return (self.front_collision or self.left_collision or self.right_collision)

    def readArduino(self, cmd):
        if self.arduino_activated:
            # (lD148.61,zD28.93,fD105.21,rD52.11,gD:0.0,B0,T0,l:0,r:0)
            self.raw_serial_devices[ARDUINO_SN].write(cmd.encode())
            if not cmd == "X":
                ##issue where len(READARD) !> 0 so next if never fires.
                self.data = self.raw_serial_devices[ARDUINO_SN].read_until(b")").decode().strip()
                self.log((self.data))
                if len(self.data) > 0:
                    self.packet_fail = 0
                    if self.data[:1] == "(" and self.data[-1:] == ")":  #whole packet?
                        self.data = self.data[1:-1]
                        self.data = self.data.split(",")
                        #self.log(self.data)
                        i = 0
                        
                        for myString in self.data:
                            match myString[:2]:
                                case 'lD':
                                    self.left_sensor_distance = float(myString.replace("lD", ""))
                                    if VERBOSELOG: self.log(("Left Sensor; ", self.left_sensor_distance))
                                case 'zD':
                                    self.back_sensor_distance = float(myString.replace("zD", ""))
                                    if VERBOSELOG: self.log(("Back Sensor; ", self.back_sensor_distance))
                                case 'fD':
                                    self.front_sensor_distance = float(myString.replace("fD", ""))
                                    if VERBOSELOG: self.log(("Front Sensor; ", self.front_sensor_distance))
                                case 'rD':
                                    self.right_sensor_distance = float(myString.replace("rD", ""))
                                    if VERBOSELOG: self.log(("Right Sensor; ", self.right_sensor_distance))
                                case 'gD':
                                    self.gripper_sensor_distance = float(myString.replace("gD", ""))
                                    if VERBOSELOG: self.log(("Gripper Sensor; ", self.gripper_sensor_distance))
                                case 'B:':
                                    self.bottom_endstop = (int(myString.replace("B:", "")) == 1)
                                    if VERBOSELOG: self.log(("endstop B; ", self.bottom_endstop))
                                case 'T:':
                                    self.top_endstop = (int(myString.replace("T:", "")) == 1)
                                    if VERBOSELOG: self.log(("endstop T; ", self.top_endstop))
                                case 'l:':
                                    self.left_wheel = float(myString.replace("l:", ""))
                                    if VERBOSELOG: self.log(("leftWheel; ", self.left_wheel))
                                case 'r:':
                                    self.right_wheel = float(myString.replace("r:", ""))
                                    if VERBOSELOG: self.log(("rightWheel; ", self.right_wheel))
                    else:
                        self.packet_fail += 1
                        if self.packet_fail == 5:
                            #self.log(("Incomplete packet! Raised packet_fail; Retrying:"))
                            self.readArduino(cmd)
                        else:
                            self.log(("Packet drop/fail more than 3 times; continuing"))
                            self.packet_fail = 0
                else:
                    self.packet_fail += 1
                    if self.packet_fail == 5:
                        #self.log(("len(packet) is 0! Raised packet_fail; Retrying:"))
                        self.readArduino(cmd)
                    else:
                        self.log(("Packet drop/fail more than 3 times; continuing"))
                        self.packet_fail = 0
    ##            #self.top_endstop is named the same so is already updated. I kept the sensors separate to be able to distinguish where errors may be occuring.
    ##            #the endstops are only used once so I thought that was the smartest choice.
    ##            #left wheel and right wheel haven't existed up until now so they are the new varibles.
            else:
                self.log(("WHEELS RESET"))
        else:
            self.log(("Arduino INACTIVE; readArduino Failed"))

    def updateMarkers(self):
        self.restInterupt(0.4)
        self.markers = self.camera.see()    #ensure we take scene to evaluate next nearest asteroid, spaceship or egg!
        self.log2("Markers updated.")

    def update(self,photo):
        self.log2("!! UPDATE !!")
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
        if self.marker_target != "":
            if self.marker_target.id == self.mySpaceShip:
                collision_offset = COLLISION_OFFSET + SPACESHIP_OFFSET
            if self.isBoundary(self.marker_target.id):
                collision_offset = COLLISION_OFFSET + BOUNDARY_OFFSET
            if self.armIsUp:
                collision_offset = 0 #Centimetres!
            if self.undocking:
                 collision_offset = -70 #Centimetres!

        self.clearCollision() # re-detect for collisions...
        self.readArduino("C")
        
        #detect here if a sensor has less than MIN_DIST, set class variable accordingly.
        self.log(("MIN_DIST, COLLISION_OFFSET, collision_offset", MIN_DIST, COLLISION_OFFSET, collision_offset))
        if self.front_sensor_distance <= MIN_DIST + collision_offset:
            self.front_collision = True
        if self.back_sensor_distance <= MIN_DIST + collision_offset:
            self.back_collision = True
        if self.right_sensor_distance <= MIN_DIST + collision_offset:
            self.right_collision = True
        if self.left_sensor_distance  <= MIN_DIST + collision_offset:
            self.left_collision = True
            
        if self.front_collision: 
            self.log2(("Front Collision detected, Halted;", self.front_sensor_distance))
            self.stopRobot()
            self.power_board.piezo.buzz(Note.B6, 0.2)
            self.power_board.piezo.buzz(Note.B6, 0.2)
        if self.left_collision:
            self.log2(("Left Collision detected, Halted;", self.left_sensor_distance))
            self.stopRobot()
            self.power_board.piezo.buzz(Note.B6, 0.2)
            self.power_board.piezo.buzz(Note.B6, 0.2)
        if self.right_collision:
            self.log2(("Right Collision detected, Halted;", self.right_sensor_distance))
            self.stopRobot()
            self.power_board.piezo.buzz(Note.B6, 0.2)
            self.power_board.piezo.buzz(Note.B6, 0.2)
        if self.back_collision:
            self.log2(("Back Collision detected, Halted;", self.back_sensor_distance))
            self.stopRobot()
            self.power_board.piezo.buzz(Note.B6, 0.2)
            self.power_board.piezo.buzz(Note.B6, 0.2)
            
        if self.stop_motors == True and photo:
            self.updateMarkers()
            if len(self.markers) <= 0:
                self.markers = []
                if self.front_collision and not self.back_collision:
                    self.log2(("FRONT IS AGAINST A WALL! MOVE BACK. DON'T MOVE FAST; YOU CAN'T SEE."))
                    self.moveRobot(-0.2, 0.5)
                    
                elif self.back_collision and not self.front_collision:
                    self.log2(("BACK IS AGAINST A WALL! CAMERA COVERED. MOVE FORWARD. SCAN WILL ACTIVATE AS NORMAL."))
                    self.moveRobot(0.2, 0.5)
            else:
                #self.restInterupt(0.1)
                self.log2(("Updated Sensors, Camera active, Measure F,B,MRKR_DIST", self.front_sensor_distance, self.back_sensor_distance, self.markers[0].position.distance/1000))
        else:
            if len(self.markers) > 0:
                self.log2(("Updated Sensors, Camera Inactive, Measure F,B,MRKR_DIST", self.front_sensor_distance, self.back_sensor_distance, self.markers[0].position.distance/1000))
            else:
                self.log(("Updated Sensors, Camera Inactive, Measure F,B", self.front_sensor_distance, self.back_sensor_distance))
        self.log2("!! UPDATE FIN !!")
        
    #Robot mobility functions    
    def stopRobot(self):                    #used to force robot to cease movement from wheels.          
        self.stop_motors = True
        self.motor_boards[WHEELBOARD].motors[0].power = 0
        self.motor_boards[WHEELBOARD].motors[1].power = 0

    def startRobot(self):
        self.stop_motors = False

    def rotateRobot(self, angle):
        if not self.stop_motors:
            # Accept positive or negative angle in degrees
            # 1.395 @ 0.3 = 90 degrees
            # 5.58 @ 0.3 = 360 degrees
            # 1 degree = 0.0155
            # speed = distance / time
            # to turn 1 degree angle will be 1
            # so the delay is 0.0155
            # delay is angle * 0.0155
            delay = abs(angle) * TURN_CONSTANT
            #TURN_CONSTANT = 0.0155
            #TURN_SPEED = 0.1
            if angle < 0:                       #left
                self.log(("Rotate Left", angle))
                self.motor_boards[WHEELBOARD].motors[0].power = -TURN_SPEED
                self.motor_boards[WHEELBOARD].motors[1].power = TURN_SPEED
            elif angle > 0:                     #right
                self.log(("Rotate Right", angle))
                self.motor_boards[WHEELBOARD].motors[0].power = TURN_SPEED
                self.motor_boards[WHEELBOARD].motors[1].power = -TURN_SPEED
            else:
                print("Angle is zero, wyd?")
            robot.restInterupt(delay)
            self.motor_boards[WHEELBOARD].motors[0].power = 0
            self.motor_boards[WHEELBOARD].motors[1].power = 0
        else:
            self.log("rotateRobot failed; stop_motors is True")

    def smoothRotateRobot(self, angle):
        arr = [10, 5, 2.5, 1.25, 1, 1.25, 2.5, 5, 10]
        if not self.stop_motors:
            for divisor in arr:
                divadjust = 1/divisor
                delay = (abs(angle) * SMOOTH_TURN_CONSTANT) / divadjust
                self.log((delay))
                speed = TURN_SPEED/divisor
                #TURN_CONSTANT = 0.0155
                #TURN_SPEED = 0.1
                if angle < 0:                       #left
                    self.log(("Rotate Left", angle))
                    self.motor_boards[WHEELBOARD].motors[0].power = -speed
                    self.motor_boards[WHEELBOARD].motors[1].power = speed
                elif angle > 0:                     #right
                    self.log(("Rotate Right", angle))
                    self.motor_boards[WHEELBOARD].motors[0].power = speed
                    self.motor_boards[WHEELBOARD].motors[1].power = -speed
                else:
                    print("Angle is zero, wyd?")
                robot.restInterupt(delay)
            self.motor_boards[WHEELBOARD].motors[0].power = 0
            self.motor_boards[WHEELBOARD].motors[1].power = 0
        else:
            self.log("rotateRobot failed; stop_motors is True")
            

    def moveRobot(self, speed, duration):   #Move at specified speed and for duration
        if not self.stop_motors:
            starttime = datetime.now()      #current date and time
            endtime = starttime + timedelta(seconds=duration)
            now_time = datetime.now()
            while now_time < endtime and not robot.frontCollision():
                self.log(("moveRobot speed:", speed," duration ", duration))
                self.motor_boards[WHEELBOARD].motors[0].power = speed
                self.motor_boards[WHEELBOARD].motors[1].power = speed
                self.update(True)
                now_time = datetime.now()
            self.stopRobot()
            self.log("Stop Moving")
        else:
            self.log("moveRobot failed; stop_motors is True")

    def moveRobotDistanceForward(self, distance):   #distance in mm
        duration = distance/180
        self.log(("moveRobot speed: 1, duration ", duration))
        if not self.stop_motors:
            starttime = datetime.now()      #current date and time
            endtime = starttime + timedelta(seconds=duration)
            now_time = datetime.now()
            while now_time < endtime and not robot.frontCollision():
                self.motor_boards[WHEELBOARD].motors[0].power = 1
                self.motor_boards[WHEELBOARD].motors[1].power = 1
                self.update(True)
                now_time = datetime.now()
##            self.log("Stop Moving")
##            self.stopRobot()
        else:
            self.log("moveRobot failed; stop_motors is True")

    def moveTowardsBox(self, Object, decimal, box_offset):
        if self.ground_distance < 450:  # only see top marker of box; force 'final move'
            decimal = 1
            box_offset = 100
        self.log(("Ground Distance:", self.ground_distance))
        self.update(True)
        self.getTarget(Object)
        self.startRobot()
        self.moveRobotDistanceForward((self.ground_distance*decimal)+box_offset)
        self.stopRobot()
        self.ground_distance = 0

    def calculateAngleFromArmOffset(self):
        a = (self.ground_distance*math.sin(math.radians(90)-self.marker_target.position.horizontal_angle)) / (math.sin(math.radians(90)))
        self.log2("A", a)
        self.wait_start()
        c = math.sqrt( math.pow(self.ground_distance, 2) - math.pow(a, 2) )
        self.log2("C", c)
        self.wait_start()
        F = math.atan(a/(c+ARM_CAMERA_OFFSET_MM))
        self.log2("F", F)
        self.log2("HORIZ_ANG", math.degrees(self.marker_target.position.horizontal_angle))
        self.wait_start()
        theta = self.marker_target.position.horizontal_angle + ( (math.radians(90) - self.marker_target.position.horizontal_angle - F) )
        self.log2("theta", theta)
        return -math.degrees(theta)

    def correctAngle(self, Object, marker_fov):
        if self.getTarget(Object):
            if not abs(self.marker_target.position.horizontal_angle) < marker_fov:
                self.update(True)
                self.getTarget(Object)
                self.startRobot()
                if self.marker_target != "":
                    self.rotateRobot(self.calculateAngleFromArmOffset())
                    self.stopRobot()
                    return True
                else:
                    self.log2("PANIC! BOX LOST; find new one.")
                    return False
                    ## PANIC! BOX LOST; find new one.
                        
    def findObject(self, Object):
        self.log2("findObject", Object)
        self.arrived = False
        self.continue_flag = True
        self.stopRobot()
        self.update(True)
        if self.getTarget(Object):
            while not self.frontCollision() and not self.arrived:
                if self.correctAngle(Object, FIRST_MARKER_FOV_THRESHOLD):
                    if Object != ASTEROID and Object != EGG:
                        if Object == MYSPACESHIP or Object == self.mySpaceShip:
                            self.armUp()
                        self.moveTowardsBox(Object, 1, 0) # was Object, 1, 30
                    else:
                        self.moveTowardsBox(Object, 0.6, 0)
                        if self.correctAngle(Object, SECOND_MARKER_FOV_THRESHOLD):
                            self.moveTowardsBox(Object, 1, 150)
                            self.log2(("findObject finished; moveTowardsBox ended"))
                            self.arrived = True
                        else:
                            return self.findObject(Object)
                else:
                    return self.findObject(Object)
                if self.frontCollision():
                    self.log2(("findObject(", Object, ") aborted; frontCollision is True"))
                    return self.findObject(Object)
                else:
                    return True
        else:
            self.arrived = False
            self.startRobot()
            self.rotateRobot(SCAN_ANGLE)
            self.stopRobot()
            self.update(True)
            return self.findObject(Object)

robot = MyRobot()
done = False

##while not done:
##    robot.findObject(ASTEROID)
##    robot.pickBox()
##    robot.movearmHalfPos(0.4)
##    robot.clearCollision()
##    robot.findObject((robot.zone*7) + 2)                   #We've decided to look for ID 2. Its a border marker near out spaceship.
##    robot.clearCollision()
##    # NEED TO ADD CASE WHERE if mySpaceShip detected, don't bother getTarget-ing again or checking alignment twice. Just go straight for marker and dump box.
##    # width of box is so wide it is hard to miss haha.
##    robot.findObject(MYSPACESHIP)
##    robot.clearCollision()
##    robot.dropBox()
##    done = True
    #if len(robot.visited_markers) == 5: # stop after 5 boxes dropped into spaceship
        #done = True

##    print("\n\nNEW LOOP\n\n")


# Working & Unworking tests

##while not done:               # NOT WORKING !!!
##    robot.servo_board.servos[0].position = 0.2
##    robot.restInterrupt(1)
##    robot.servo_board.servos[1].position = 0.2
##    robot.restInterupt(1)
##    robot.servo_board.servos[0].position = -0.5
##    robot.servo_board.servos[1].position = -0.5
##    print("hehe")
##    robot.wait_start()
##    print("woah")

##while not done:         # MovearmhalfPos test
##    robot.motor_boards[ARMVACBOARD].motors[0].power = 1.0
##    robot.armDown()
##    robot.movearmHalfPos(1.0)
##    robot.update(True)
##    robot.restInterupt(1.7)
##    robot.motor_boards[ARMVACBOARD].motors[0].power = 0.0
##    done=True

##while not done:         # MINIMUM MARKER RANGE calibration
##    robot.update(True)
##    robot.getTarget(ASTEROID)
##    robot.log2("Ground Distance:", robot.ground_distance)

##while not done:           #calculateAngle test
##    robot.update(True)
##    robot.getTarget(ASTEROID)
##
##    x = robot.calculateRotationAngle()
##    robot.log(("CALCULATE_ROTATION_ANGLE:", x))
##    robot.startRobot()
##    robot.rotateRobot(x)
##    robot.restInterupt(0.2)
##    robot.moveRobotDistanceForward(robot.ground_distance-20)
##    
##    robot.restInterupt(0.2)
##    robot.rotateRobot(-x)
##    robot.restInterupt(5)
##    done=True

##while not done:     #encoder calibration
##    robot.stop_motors = False
##    robot.moveRobot(1, 1)   #this moves 180mm
##    robot.moveRobot(1, 2)   #this moves ~360mm
##    robot.moveRobot(1, 3)   #this moves ~540mm
##    robot.moveRobotDistanceForward(400)
##    robot.readArduino("W")
##    done=True

##while not done:     #turnconstant tests
##    robot.stop_motors = False
##    robot.smoothRotateRobot(90)
##    robot.restInterupt(3)
##    robot.motor_boards[WHEELBOARD].motors[0].power = -1
##    robot.motor_boards[WHEELBOARD].motors[1].power = 1

##while not done:     #SHAKE test
##    robot.rotateRobot(1)
##    robot.rotateRobot(2)
##    robot.rotateRobot(4)
##    robot.rotateRobot(8)
##    robot.rotateRobot(10)
##    robot.rotateRobot(10)
##    robot.rotateRobot(8)
##    robot.rotateRobot(4)
##    robot.rotateRobot(2)
##    robot.rotateRobot(1)

##while not done:     #arm , vacuum check
##    robot.armInit()
##    robot.log2("PICKBOX")
##    robot.pickBox()
##    robot.log2("ARMUP")
##    robot.armUp()
##    robot.log2("letGoBox")
##    robot.letGoBox()
##    robot.wait_start()
    
##
##while not done:     #sensors check
##    robot.update(True)
##    robot.clearCollision()
##    robot.wait_for_start()

##while not done:     #motor check
##    robot.stop_motors = False
##    robot.restInterupt(1)
##    robot.motor_boards[WHEELBOARD].motors[0].power = 1
##    robot.motor_boards[WHEELBOARD].motors[1].power = 1
##    robot.restInterupt(1)
##    robot.motor_boards[WHEELBOARD].motors[0].power = 0
##    robot.motor_boards[WHEELBOARD].motors[1].power = 0
##    robot.restInterupt(1)
##    robot.motor_boards[WHEELBOARD].motors[0].power = -1
##    robot.motor_boards[WHEELBOARD].motors[1].power = -1
##    robot.restInterupt(1)
##    robot.motor_boards[WHEELBOARD].motors[0].power = 0
##    robot.motor_boards[WHEELBOARD].motors[1].power = 0
##    robot.restInterupt(1)
##    robot.stop_motors = False
##    robot.rotateRobot(90)
##    robot.restInterupt(1)
##    robot.stop_motors = False
##    robot.rotateRobot(-90)
##    robot.wait_for_start()

##while not done:     #vacuum check
##    robot.motor_boards[ARMVACBOARD].motors[0].power = 1
##    robot.restInterupt(1)
##    robot.motor_boards[ARMVACBOARD].motors[0].power = 0
##    robot.restInterupt(1)
##    robot.wait_for_start()

done = True
