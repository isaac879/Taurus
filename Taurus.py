#!/usr/bin/python

#=================================================================================
#Python script for project Taurus PROJ324
#MEng Robotics
#Plymouth University
#Isaac Chasteau
#=================================================================================

from Adafruit_PWM_Servo_Driver import PWM
import time
import math
import sys
import numbers
import piMusic
import MCP3008_ADC
import PS3Controller

#=================================================================================

# Initialise the PWM device using the default address
pwm1 = PWM(0x40) #Servo driver board 1, legs 1, 3 and 5
pwm2 = PWM(0x41) #Servo driver board 2, legs 2, 4 and 6
pwm3 = PWM(0x42) #Servo driver board 3, Body, Tail and Eyes

#Sets the PWM frequency for each servo driver board
pwm1.setPWMFreq(60) #Set frequency to 60Hz
pwm2.setPWMFreq(60) #Set frequency to 60Hz
pwm3.setPWMFreq(60) #Set frequency to 60Hz
#=================================================================================

#Maps a number from one range to another
def map_number(input_variable, min_input, max_input, min_output, max_output):
    return (input_variable - min_input) * (max_output - min_output)/(max_input - min_input) + min_output

#=================================================================================

def Exit_Error(Error_Message):
    print "ERROR: %s" % Error_Message
    sys.exit(1)

#=================================================================================

class Leg:
    L1 = 64.25 #Length of link 1 (S1 to S2) in mm
    L2 = 64.25 #Length of link 2 (S2 to S3) in mm
    L3 = 135.6 #Length of link 3 (S3 to end effector) in mm

    #X, Y, Z Offset for the servo mounting position relitive to the defined origin
    X_Offset = 0.0
    Y_Offset = 0.0
    Z_Offset = 0.0

    #Base X, Y, Z coordinates for the end effector
    X_base_position = 0.0
    Y_base_position = 0.0
    Z_base_position = -80.0

    #Desired X, Y, Z coordinates for the end effector
    X_Coordinate = 0.0
    Y_Coordinate = 0.0
    Z_Coordinate = -80.0

    #Previous X, Y, Z coordinates
    Last_X = X_base_position
    Last_Y = Y_base_position
    Last_Z = Z_base_position

    previous_X = X_base_position
    previous_Y = Y_base_position
    previous_Z = Z_base_position
    
    #Kinematic limit flag
    limit_Flag = 0
    
    #Joint angles
    Hip_Joint = 0
    Femur_Joint = 0
    Tibia_Joint = 0
    
    #Servo pulse counts
    servoMin_S1 = 150
    servoMax_S1 = 550
    servoMin_S2 = 150
    servoMax_S2 = 550
    servoMin_S3 = 150
    servoMax_S3 = 550

    #Side the leg is mounted on
    Leg_Side = "Unknown"

    #Step number the leg is on through its current gait stride profile
    step_number = 0

    #Leg inverse kinematics
    def inverse_kinematics(self):
        Last_X_Coordinate = self.X_Coordinate
        Last_Y_Coordinate = self.Y_Coordinate
        Last_Z_Coordinate = self.Z_Coordinate
        
        #Relate the X, Y, Z coordinates of the end effector to the origin
        Leg_X_Coordinate = self.X_Coordinate - self.X_Offset
        Leg_Y_Coordinate = self.Y_Coordinate - self.Y_Offset
        Leg_Z_Coordinate = self.Z_Coordinate - self.Z_Offset

        if self.Leg_Side == "Right":
            #Calculating angles in radians
            theta = math.pi + math.atan2(Leg_Y_Coordinate, Leg_X_Coordinate)
            DFOXY = math.sqrt(Leg_X_Coordinate**2 + Leg_Y_Coordinate**2)
            XY_Component = DFOXY - self.L1
            extension = math.sqrt(XY_Component**2 + Leg_Z_Coordinate**2)
            if extension > self.L2 + self.L3 or extension < 79.35317504 or (Leg_Z_Coordinate > 0 and Leg_Y_Coordinate > 0):# or phi > math.pi or phi < 0:#out of limits
                print "Out of limits"
                self.X_Coordinate = self.Last_X
                self.Y_Coordinate = self.Last_Y
                self.Z_Coordinate = self.Last_Z
                self.limit_Flag = 1
            else:  
                psy = (math.acos((self.L2**2 + self.L3**2 - extension**2)/(2 * self.L2 * self.L3))) - 0.374251421# + 0.4111467424 - 0.7853981634
                phip1 = math.atan2(XY_Component, Leg_Z_Coordinate)
                delta = (math.acos((self.L3**2 + extension**2 - self.L2**2)/(2 * self.L3 * extension)))
                phip2 = (math.acos((self.L2**2 + extension**2 - self.L3**2)/(2 * self.L2 * extension)))
                phi = phip1 - phip2
                if phi < 0 or phi > math.pi:
                    print "Out of limits"
                    self.X_Coordinate = self.Last_X
                    self.Y_Coordinate = self.Last_Y
                    self.Z_Coordinate = self.Last_Z
                    self.limit_Flag = 1
                else:
                    self.Last_X = self.X_Coordinate
                    self.Last_Y = self.Y_Coordinate
                    self.Last_Z = self.Z_Coordinate

                    self.Hip_Joint = int(round(map_number(theta, 0, math.pi, self.servoMin_S1, self.servoMax_S1)))
                    self.Femur_Joint = int(round(map_number(phi, 0, math.pi, self.servoMin_S2, self.servoMax_S2)))
                    self.Tibia_Joint = int(round(map_number(psy, 0, math.pi, self.servoMin_S3, self.servoMax_S3)))
                    self.limit_Flag = 0

        elif self.Leg_Side == "Left":
            theta = math.atan2(Leg_Y_Coordinate, Leg_X_Coordinate)
            DFOXY = math.sqrt(Leg_X_Coordinate**2 + Leg_Y_Coordinate**2)
            XY_Component = DFOXY - self.L1
            extension = math.sqrt(XY_Component**2 + Leg_Z_Coordinate**2)
            
            if extension > self.L2 + self.L3 or extension < 79.35317504 or (Leg_Z_Coordinate > 0 and Leg_Y_Coordinate < 0) or theta < 1.2444 or theta > 2.234:# or phi > math.pi or phi < 0:#out of limits
                print "Out of limits"
                self.X_Coordinate = self.Last_X
                self.Y_Coordinate = self.Last_Y
                self.Z_Coordinate = self.Last_Z
                self.limit_Flag = 1
            else:
                psy = (math.acos((self.L2**2 + self.L3**2 - extension**2)/(2 * self.L2 * self.L3))) - 0.374251421# + 0.4111467424 - 0.7853981634# * 180/math.pi + 23.5569731 - 45
                phip1 = math.atan2(XY_Component, Leg_Z_Coordinate)
                delta = (math.acos((self.L3**2 + extension**2 - self.L2**2)/(2 * self.L3 * extension)))
                phip2 = (math.acos((self.L2**2 + extension**2 - self.L3**2)/(2 * self.L2 * extension)))
                phi = phip1 - phip2
                if phi < 0 or phi > math.pi:
                    print "Out of limits"
                    self.X_Coordinate = self.Last_X
                    self.Y_Coordinate = self.Last_Y
                    self.Z_Coordinate = self.Last_Z
                    self.limit_Flag = 1
                else:
                    self.Last_X = self.X_Coordinate
                    self.Last_Y = self.Y_Coordinate
                    self.Last_Z = self.Z_Coordinate

                    self.Hip_Joint = int(round(map_number(theta, 0, math.pi, self.servoMin_S1, self.servoMax_S1)))
                    self.Femur_Joint = int(round(map_number(phi, 0, math.pi, self.servoMin_S2, self.servoMax_S2)))
                    self.Tibia_Joint = int(round(map_number(psy, 0, math.pi, self.servoMin_S3, self.servoMax_S3)))
                    self.limit_Flag = 0       
        else:
            print "Leg_Side needs to be either 'Left' or 'Right'. It is currently set to %s" %self.Leg_Side

#=================================================================================

class Claw:
    L1 = 69.0 #Length of link 1 (S1 to S2) in mm
    L2 = 57.0 #Length of link 2 (S2 to S3) in mm
    L3 = 0.0 #Length of link 3 (S3 to end effector) in mm

    #X, Y, Z Offset for the servo mounting position relitive to the defined origin
    X_Offset = 175.0
    Y_Offset = 0.0
    Z_Offset = 10.0

    #Base X, Y, Z coordinates for the end effector
    X_base_position = 60.0
    Y_base_position = 0.0
    Z_base_position = 20

    #Desired X, Y, Z coordinates for the end effector
    X_Coordinate = 60.0
    Y_Coordinate = 0.0
    Z_Coordinate = 20.0

    #Joint angles
    Shoulder_XZ = 90.0
    Shoulder_XY = 45.0
    Elbow = 90.0
    Wrist = 0.0
    Pincer = 50

    #Pulse counts for the Pincer to be open and closed
    PincerOpen = 0
    PincerClosed = 0

    #Servo pulse counts
    servoMin_S1 = 150
    servoMax_S1 = 550
    servoMin_S2 = 150
    servoMax_S2 = 550
    servoMin_S3 = 150
    servoMax_S3 = 550
    servoMin_S4 = 150
    servoMax_S4 = 550
    servoMin_S5 = 150
    servoMax_S5 = 550

    #Side the claw is mounted on
    Claw_Side = "Unknown"

    #Claw inverse kinematics
    def inverse_kinematics(self):
        #Relate the X, Y, Z coordinates of the end effector to the origin
        Claw_X_Coordinate = self.X_Coordinate - self.X_Offset
        Claw_Y_Coordinate = self.Y_Coordinate - self.Y_Offset
        Claw_Z_Coordinate = self.Z_Coordinate - self.Z_Offset

        if self.Claw_Side == "Right":
            Beta = self.Wrist
            
            #Calculating angles in radians
            Theta = math.atan2(Claw_Z_Coordinate, Claw_X_Coordinate)
            Extension = math.sqrt(Claw_X_Coordinate**2 + Claw_Y_Coordinate**2 + Claw_Z_Coordinate**2)
            Delta = math.acos((self.L1**2 + self.L2**2 - Extension**2)/(2 * self.L1 * self.L2))
            Omega = math.acos((Extension**2 + self.L1**2 - self.L2**2)/(2 * Extension * self.L1))
            Sigma = math.asin(Claw_Y_Coordinate/Extension)
            Lambda2 =  Sigma - Omega

            self.Shoulder_XZ = int(round(map_number(1.570796327 - Theta, 0, math.pi, self.servoMin_S1, self.servoMax_S1)))
            testing = int(round(map_number(Lambda2*180/math.pi, -135, 45, 180, 0)))
            self.Shoulder_XY = int(round(map_number(testing + 25, 0, 180, self.servoMin_S2, self.servoMax_S2)))
            self.Elbow = int(round(map_number(Delta + 0.3490658504, 0, math.pi, self.servoMin_S3, self.servoMax_S3)))

        elif self.Claw_Side == "Left":
            Beta = self.Wrist

            #Calculating angles in Radians
            Theta = math.atan2(Claw_Z_Coordinate, Claw_X_Coordinate)
            Extension = math.sqrt(Claw_X_Coordinate**2 + Claw_Y_Coordinate**2 + Claw_Z_Coordinate**2)
            Delta = math.acos((self.L1**2 + self.L2**2 - Extension**2)/(2 * self.L1 * self.L2))
            Omega = math.acos((Extension**2 + self.L1**2 - self.L2**2)/(2 * Extension * self.L1))
            Sigma = math.asin(Claw_Y_Coordinate/Extension)
            Lambda2 =  Sigma + Omega

            self.Shoulder_XZ = int(round(map_number(1.570796327 - Theta, 0, math.pi, self.servoMin_S1, self.servoMax_S1)))
            testing = int(round(map_number(Lambda2*180/math.pi, -45, 135, 0, 180)))#use rads
            self.Shoulder_XY = int(round(map_number(testing + 25, 0, 180, self.servoMin_S2, self.servoMax_S2)))
            self.Elbow = int(round(map_number(Delta + 0.3490658504, 0, math.pi, self.servoMax_S3, self.servoMin_S3)))

        else:
            print "Claw_Side needs to be either 'Left' or 'Right'. It is currently set to %s" %self.Claw_Side

    def pincer_position(self, position):#0 = closed, 100 = Open

        if position >= 0 and position <= 100:
            pincerPosition = int(round(map_number(position, 0, 100, self.PincerClosed, self.PincerOpen)))

            if self.Claw_Side == "Left":
                pwm1.setPWM(15, 0, pincerPosition)#Servo 5 Left Claw
            elif self.Claw_Side == "Right":
                pwm2.setPWM(0, 0, pincerPosition)#Servo 5 Right Claw
            else:
                Exit_Error("Claw Side is not Right or Left")
        else:
            Exit_Error("Pincer position value out of range")

#=================================================================================

class Body:
    roll = 0.0#Body roll about the X
    pitch = 0.0#Body pitch about the Y
    yaw = 0.0#Body yaw about the Z

    previous_roll = 0
    previous_pitch = 0
    previous_yaw = 0
    
    Legs_1_2_X_position = 95.0#Base X distance for legs 1 and 2 from the origin
    Legs_3_4_X_position = 0.0#Base X distance for legs 3 and 4 from the origin
    Legs_5_6_X_position = -95.0#Base X distance for legs 5 and 6 from the origin
    camber = 340.0#Base distance between feet on oposite sides
    height = -100.0#Base height of the origin
    Camera_Angle = 0.0#Angle of the camera relavive to the X axis in the XZ plain
    camera_variable = 40
    pitchToggle = 0

    #Link lengths
    L1 = 64.25 #Length of link 1 in mm
    L2 = 64.25 #Length of link 2 in mm
    L3 = 64.25 #Length of link 3 in mm
    L4 = 64.25 #Length of link 4 in mm
    L5 = 64.25 #Length of link 5 in mm
    LTail_Base = 151.362 #Length of link LTail_Base in mm
    
    #Tail base offset from the origin
    tail_X_Offset = - 182 #X offset
    tail_Y_Offset = 0 #Y offset
    tail_Z_Offset = 1.25 #Z offset
    
    #Servo pulse counts at the base of the tail
    servoMin_S29 = 170
    servoMax_S29 = 525
    servoMin_S30 = 160
    servoMax_S30 = 518

    #Tail coordinates
    X_Tail_Coordinate = -240
    Z_Tail_Coordinate = 190
    synch = 0.60#Variable used to ensure both are always mapped to the same value
    
    Last_X = X_Tail_Coordinate
    Last_Z = Z_Tail_Coordinate
    Last_synch = synch

    #RPY initial leg values
    Leg1_X = 0
    Leg1_Y = 0
    Leg1_Z = 0

    Leg2_X = 0
    Leg2_Y = 0
    Leg2_Z = 0

    Leg3_X = 0
    Leg3_Y = 0
    Leg3_Z = 0

    Leg4_X = 0
    Leg4_Y = 0
    Leg4_Z = 0

    Leg5_X = 0
    Leg5_Y = 0
    Leg5_Z = 0

    Leg6_X = 0
    Leg6_Y = 0
    Leg6_Z = 0
    
    #Servos pulse counts in the tail
    #Segment 1
    servoMin_S24 = 155
    servoMax_S24 = 628
    #Segment 2
    servoMin_S23 = 135
    servoMax_S23 = 620
    #Segment 3
    servoMin_S22 = 136
    servoMax_S22 = 610
    #Segment 4
    servoMin_S20 = 165
    servoMax_S20 = 650
    #Segment 5
    servoMin_S19 = 142
    servoMax_S19 = 632

    #Tail servo default positions
    Tail_base_S29 = int(round(map_number(synch, -90, 45, servoMax_S29, servoMin_S29)))
    Tail_base_S30 = int(round(map_number(synch, -90, 45, servoMax_S30, servoMin_S30)))

    Tail_Segment_1 = int(round(map_number(45, -90, 90, servoMin_S24, servoMax_S24)))
    Tail_Segment_2 = int(round(map_number(35, -90, 90, servoMin_S23, servoMax_S23)))
    Tail_Segment_3 = int(round(map_number(35, -90, 90, servoMin_S22, servoMax_S22)))
    Tail_Segment_4 = int(round(map_number(35, -90, 90, servoMin_S20, servoMax_S20)))
    Tail_Segment_5 = int(round(map_number(camera_variable, -90, 90, servoMin_S19, servoMax_S19)))

    #Calculates the movement to give the specified body pose
    def kinematics(self, camera_Angle):
        print "Body Kinematics"

    def set_tail_positions(self, TailBase, Segment1, Segment2, Segment3, Segment4, Segment5):
        if all((i >= -90 and i <= 90) for i in(Segment1, Segment2, Segment3, Segment4, Segment5)) and TailBase >= -90 and TailBase <= 45:
            synch = TailBase#Variable used to ensure both are always mapped to the same value
            self.Tail_base_S29 = int(round(map_number(synch, -90, 45, self.servoMax_S29, self.servoMin_S29)))
            self.Tail_base_S30 = int(round(map_number(synch, -90, 45, self.servoMax_S30, self.servoMin_S30)))
            self.Tail_Segment_1 = int(round(map_number(Segment1, -90, 90, self.servoMin_S24, self.servoMax_S24)))
            self.Tail_Segment_2 = int(round(map_number(Segment2, -90, 90, self.servoMin_S23, self.servoMax_S23)))
            self.Tail_Segment_3 = int(round(map_number(Segment3, -90, 90, self.servoMin_S22, self.servoMax_S22)))
            self.Tail_Segment_4 = int(round(map_number(Segment4, -90, 90, self.servoMin_S20, self.servoMax_S20)))
            self.Tail_Segment_5 = int(round(map_number(Segment5, -90, 90, self.servoMin_S19, self.servoMax_S19)))
            self.Camera_Angle = 180 - synch - Segment1 - Segment2 - Segment3 - Segment4 - Segment5
        else:
             Exit_Error("Invalid Tail Servo Position")#Exits the script and prints an error message
    
    def tail_inverse_kinematics(self):
       
        synch = self.synch #need to calculate synch so it will be dynamic #Variable used to ensure both are always mapped to the same value
        
        X_tail_base_offset = math.cos(synch) * self.LTail_Base
        Z_tail_base_offset = math.sin(synch) * self.LTail_Base

        X_target = self.X_Tail_Coordinate - self.tail_X_Offset + X_tail_base_offset
        Z_target = self.Z_Tail_Coordinate - self.tail_Z_Offset - Z_tail_base_offset

        extension1 = math.sqrt(X_target**2 + Z_target**2)
        extension2 = map_number(extension1, 0, self.L1 + self.L2 + self.L3 + self.L4, math.sqrt(self.L1**2 + self.L2**2), self.L1 + self.L2)
        extension3 = map_number(extension1, 0, self.L1 + self.L2 + self.L3 + self.L4,  math.sqrt(self.L3**2 + self.L4**2), self.L3 + self.L4)

        if extension1 >= self.L1 + self.L2 + self.L3 + self.L4 or extension2 >= self.L1 + self.L2 or extension3 >= self.L3 + self.L4:
            print "Tail out of kinematic limits"
            self.X_Tail_Coordinate = self.Last_X 
            self.Z_Tail_Coordinate = self.Last_Z
            self.synch = self.Last_synch
            return 0
        
        sigma = math.acos((self.L3**2 + self.L4**2 - extension2**2)/(2 * self.L3 * self.L4))
        rho = math.acos((self.L1**2 + self.L2**2 - extension2**2)/(2 * self.L1 * self.L2))

        Lambda = math.acos((extension2**2 + extension3**2 - extension1**2)/(2 * extension2 * extension3))
        l = Lambda*180/math.pi
        
        eta = math.acos((extension2**2 + self.L2**2 - self.L1**2)/(2 * extension2 * self.L2))
        theta = math.acos((self.L3**2 + extension3**2 - self.L4**2)/(2 * self.L3 * extension3))
        
        mu = math.atan2(Z_target, -X_target)
        nu = math.acos((self.L1**2 + extension2**2 - self.L2**2)/(2 * self.L1 * extension2))
        xi = math.acos((extension2**2 + extension1**2 - extension3**2)/(2 * extension2 * extension1))
        
        alpha = mu - synch - xi - nu
        beta = 3.141592654 - sigma
        gamma = 3.141592654 - eta - Lambda - theta
        delta = 3.141592654 - rho
        zeta = self.Camera_Angle
        
        if self.synch > -1.570796327 + 0.1 and self.synch < 0.7853981634 - 0.1 and extension1 <= self.L1 + self.L2 + self.L3 + self.L4 -4 and extension1 >= 110 and (self.Z_Tail_Coordinate > 130 or (self.Z_Tail_Coordinate >= 0 and self.X_Tail_Coordinate < self.tail_X_Offset)) and alpha <= 1.570796327:#With in the limits

            self.Last_X = self.X_Tail_Coordinate
            self.Last_Z = self.Z_Tail_Coordinate
            self.Last_synch = self.synch
            
            self.Tail_base_S29 = int(round(map_number(synch, -1.570796327, 0.7853981634, self.servoMax_S29, self.servoMin_S29)))
            self.Tail_base_S30 = int(round(map_number(synch, -1.570796327, 0.7853981634, self.servoMax_S30, self.servoMin_S30)))
            self.Tail_Segment_1 = int(round(map_number(alpha, -1.570796327, 1.570796327, self.servoMin_S24, self.servoMax_S24)))
            self.Tail_Segment_2 = int(round(map_number(beta, -1.570796327, 1.570796327, self.servoMin_S23, self.servoMax_S23)))
            self.Tail_Segment_3 = int(round(map_number(gamma, -1.570796327, 1.570796327, self.servoMin_S22, self.servoMax_S22)))
            self.Tail_Segment_4 = int(round(map_number(delta, -1.570796327, 1.570796327, self.servoMin_S20, self.servoMax_S20)))
            self.Tail_Segment_5 = int(round(map_number(3.141592654 - synch - alpha - beta - gamma - delta - zeta, -1.570796327, 1.570796327, self.servoMin_S19, self.servoMax_S19)))          
            
        else:
            print "Tail out of kinematic limits"
            self.X_Tail_Coordinate = self.Last_X 
            self.Z_Tail_Coordinate = self.Last_Z
            self.synch = self.Last_synch
        
    def set_camber(self, legCamber):
        self.camber = legCamber
        rightCamber = - legCamber / 2
        leftCamber = legCamber / 2
        Leg1.Y_base_position = leftCamber#Sets the base Y coordinate for the leg
        Leg3.Y_base_position = leftCamber#Sets the base Y coordinate for the leg
        Leg5.Y_base_position = leftCamber#Sets the base Y coordinate for the leg
        Leg2.Y_base_position = rightCamber#Sets the base Y coordinate for the leg
        Leg4.Y_base_position = rightCamber#Sets the base Y coordinate for the leg
        Leg6.Y_base_position = rightCamber#Sets the base Y coordinate for the leg

    def set_height(self, legHeight):
        Leg1.Z_base_position = legHeight#Sets the base Z coordinate for the leg
        Leg2.Z_base_position = legHeight#Sets the base Z coordinate for the leg
        Leg3.Z_base_position = legHeight#Sets the base Z coordinate for the leg
        Leg4.Z_base_position = legHeight#Sets the base Z coordinate for the leg
        Leg5.Z_base_position = legHeight#Sets the base Z coordinate for the leg
        Leg6.Z_base_position = legHeight#Sets the base Z coordinate for the leg
        self.height = legHeight#sets the body height

    def set_leg_spacing(self, Xpos_12, Xpos_34, Xpos_56):
        Leg1.X_base_position = Xpos_12#Sets the base X coordinate for the leg
        Leg2.X_base_position = Xpos_12#Sets the base X coordinate for the leg
        Leg3.X_base_position = Xpos_34#Sets the base Z coordinate for the leg
        Leg4.X_base_position = Xpos_34#Sets the base Z coordinate for the leg
        Leg5.X_base_position = Xpos_56#Sets the base Z coordinate for the leg
        Leg6.X_base_position = Xpos_56#Sets the base Z coordinate for the leg

    def PS3_Camera_Angle(self):
        if self.pitchToggle == 0:
            if PS3Controller.ps3.axis24 > 0.15:
                PS3Controller.ps3.axis24 = 0.15
        elif PS3Controller.ps3.axis24 < -0.15:
            PS3Controller.ps3.axis24 = -0.15

        step = map_number(PS3Controller.ps3.axis24, -0.15, 0.15, -3, 3)

        if step > 0 and self.camera_variable > (-90 + step):

            self.camera_variable = self.camera_variable - step
            self.set_tail_positions(40, 45, 30, 30, 30, self.camera_variable)

        if step < 0 and self.camera_variable < (90 + step):
            self.camera_variable = self.camera_variable - step
            self.set_tail_positions(40, 45, 30, 30, 30, self.camera_variable)

    def sting(self):
        #moves servos to make a sting look
        print "sting"

    def leg_translation_to_base_position(self, inverse_Speed):#Only to be used after translations to recenter the body
        
        X_change_step = (Leg1.X_base_position - Leg1.X_Coordinate)/inverse_Speed
        Y_change_step = (Leg1.Y_base_position - Leg1.Y_Coordinate)/inverse_Speed
        Z_change_step = (Leg1.Z_base_position - Leg1.Z_Coordinate)/inverse_Speed

        for i in range(0, inverse_Speed, 1):#for loop moves the leg to the X, Y base positions
            Leg1.X_Coordinate = Leg1.X_Coordinate + X_change_step
            Leg1.Y_Coordinate = Leg1.Y_Coordinate + Y_change_step
            Leg1.Z_Coordinate = Leg1.Z_Coordinate + Z_change_step

            Leg2.X_Coordinate = Leg2.X_Coordinate + X_change_step
            Leg2.Y_Coordinate = Leg2.Y_Coordinate + Y_change_step
            Leg2.Z_Coordinate = Leg2.Z_Coordinate + Z_change_step

            Leg3.X_Coordinate = Leg3.X_Coordinate + X_change_step
            Leg3.Y_Coordinate = Leg3.Y_Coordinate + Y_change_step
            Leg3.Z_Coordinate = Leg3.Z_Coordinate + Z_change_step

            Leg4.X_Coordinate = Leg4.X_Coordinate + X_change_step
            Leg4.Y_Coordinate = Leg4.Y_Coordinate + Y_change_step
            Leg4.Z_Coordinate = Leg4.Z_Coordinate + Z_change_step

            Leg5.X_Coordinate = Leg5.X_Coordinate + X_change_step
            Leg5.Y_Coordinate = Leg5.Y_Coordinate + Y_change_step
            Leg5.Z_Coordinate = Leg5.Z_Coordinate + Z_change_step

            Leg6.X_Coordinate = Leg6.X_Coordinate + X_change_step
            Leg6.Y_Coordinate = Leg6.Y_Coordinate + Y_change_step
            Leg6.Z_Coordinate = Leg6.Z_Coordinate + Z_change_step
            
            leg_Inverse_Kinamatics()#Calculates the inverse kinematics for the leg
            moveLegs() 

        Leg1.X_Coordinate = Leg1.X_base_position
        Leg1.Y_Coordinate = Leg1.Y_base_position
        Leg1.Z_Coordinate = Leg1.Z_base_position

        Leg2.X_Coordinate = Leg2.X_base_position
        Leg2.Y_Coordinate = Leg2.Y_base_position
        Leg2.Z_Coordinate = Leg2.Z_base_position

        Leg3.X_Coordinate = Leg3.X_base_position
        Leg3.Y_Coordinate = Leg3.Y_base_position
        Leg3.Z_Coordinate = Leg3.Z_base_position

        Leg4.X_Coordinate = Leg4.X_base_position
        Leg4.Y_Coordinate = Leg4.Y_base_position
        Leg4.Z_Coordinate = Leg4.Z_base_position

        Leg5.X_Coordinate = Leg5.X_base_position
        Leg5.Y_Coordinate = Leg5.Y_base_position
        Leg5.Z_Coordinate = Leg5.Z_base_position

        Leg6.X_Coordinate = Leg6.X_base_position
        Leg6.Y_Coordinate = Leg6.Y_base_position
        Leg6.Z_Coordinate = Leg6.Z_base_position
            
        leg_Inverse_Kinamatics()#Calculates the inverse kinematics for the leg
        moveLegs()
            
        Gait.reset_step_numbers()
        
    def linear_leg_to_base_position(self, inverse_Speed):
        
        X_change_step1 = (Leg1.X_base_position - Leg1.X_Coordinate)/inverse_Speed
        Y_change_step1 = (Leg1.Y_base_position - Leg1.Y_Coordinate)/inverse_Speed
        Z_change_step1 = (Leg1.Z_base_position - Leg1.Z_Coordinate)/inverse_Speed

        X_change_step2 = (Leg2.X_base_position - Leg2.X_Coordinate)/inverse_Speed
        Y_change_step2 = (Leg2.Y_base_position - Leg2.Y_Coordinate)/inverse_Speed
        Z_change_step2 = (Leg2.Z_base_position - Leg2.Z_Coordinate)/inverse_Speed

        X_change_step3 = (Leg3.X_base_position - Leg3.X_Coordinate)/inverse_Speed
        Y_change_step3 = (Leg3.Y_base_position - Leg3.Y_Coordinate)/inverse_Speed
        Z_change_step3 = (Leg3.Z_base_position - Leg3.Z_Coordinate)/inverse_Speed

        X_change_step4 = (Leg4.X_base_position - Leg4.X_Coordinate)/inverse_Speed
        Y_change_step4 = (Leg4.Y_base_position - Leg4.Y_Coordinate)/inverse_Speed
        Z_change_step4 = (Leg4.Z_base_position - Leg4.Z_Coordinate)/inverse_Speed

        X_change_step5 = (Leg5.X_base_position - Leg5.X_Coordinate)/inverse_Speed
        Y_change_step5 = (Leg5.Y_base_position - Leg5.Y_Coordinate)/inverse_Speed
        Z_change_step5 = (Leg5.Z_base_position - Leg5.Z_Coordinate)/inverse_Speed

        X_change_step6 = (Leg6.X_base_position - Leg6.X_Coordinate)/inverse_Speed
        Y_change_step6 = (Leg6.Y_base_position - Leg6.Y_Coordinate)/inverse_Speed
        Z_change_step6 = (Leg6.Z_base_position - Leg6.Z_Coordinate)/inverse_Speed

        for i in range(0, inverse_Speed, 1):#for loop moves the leg to the X, Y base positions
            Leg1.X_Coordinate = Leg1.X_Coordinate + X_change_step1
            Leg1.Y_Coordinate = Leg1.Y_Coordinate + Y_change_step1
            Leg1.Z_Coordinate = Leg1.Z_Coordinate + Z_change_step1

            Leg2.X_Coordinate = Leg2.X_Coordinate + X_change_step2
            Leg2.Y_Coordinate = Leg2.Y_Coordinate + Y_change_step2
            Leg2.Z_Coordinate = Leg2.Z_Coordinate + Z_change_step2

            Leg3.X_Coordinate = Leg3.X_Coordinate + X_change_step3
            Leg3.Y_Coordinate = Leg3.Y_Coordinate + Y_change_step3
            Leg3.Z_Coordinate = Leg3.Z_Coordinate + Z_change_step3

            Leg4.X_Coordinate = Leg4.X_Coordinate + X_change_step4
            Leg4.Y_Coordinate = Leg4.Y_Coordinate + Y_change_step4
            Leg4.Z_Coordinate = Leg4.Z_Coordinate + Z_change_step4

            Leg5.X_Coordinate = Leg5.X_Coordinate + X_change_step5
            Leg5.Y_Coordinate = Leg5.Y_Coordinate + Y_change_step5
            Leg5.Z_Coordinate = Leg5.Z_Coordinate + Z_change_step5

            Leg6.X_Coordinate = Leg6.X_Coordinate + X_change_step6
            Leg6.Y_Coordinate = Leg6.Y_Coordinate + Y_change_step6
            Leg6.Z_Coordinate = Leg6.Z_Coordinate + Z_change_step6
            
            leg_Inverse_Kinamatics()#Calculates the inverse kinematics for the leg
            moveLegs() 

        Leg1.X_Coordinate = Leg1.X_base_position
        Leg1.Y_Coordinate = Leg1.Y_base_position
        Leg1.Z_Coordinate = Leg1.Z_base_position

        Leg2.X_Coordinate = Leg2.X_base_position
        Leg2.Y_Coordinate = Leg2.Y_base_position
        Leg2.Z_Coordinate = Leg2.Z_base_position

        Leg3.X_Coordinate = Leg3.X_base_position
        Leg3.Y_Coordinate = Leg3.Y_base_position
        Leg3.Z_Coordinate = Leg3.Z_base_position

        Leg4.X_Coordinate = Leg4.X_base_position
        Leg4.Y_Coordinate = Leg4.Y_base_position
        Leg4.Z_Coordinate = Leg4.Z_base_position

        Leg5.X_Coordinate = Leg5.X_base_position
        Leg5.Y_Coordinate = Leg5.Y_base_position
        Leg5.Z_Coordinate = Leg5.Z_base_position

        Leg6.X_Coordinate = Leg6.X_base_position
        Leg6.Y_Coordinate = Leg6.Y_base_position
        Leg6.Z_Coordinate = Leg6.Z_base_position
            
        leg_Inverse_Kinamatics()#Calculates the inverse kinematics for the leg
        moveLegs()
            
        Gait.reset_step_numbers()
        
    def calculate_leg_roll(self, Leg, Y, Z):
        ex = math.sqrt(Y ** 2 + Z ** 2)
        psy = math.atan2(Y, Z)
        phi = psy - self.roll
        Y_roll_change = math.sin(phi) * ex - Leg.Y_Coordinate
        Z_roll_change = math.cos(phi) * ex - Leg.Z_Coordinate
        Leg.Y_Coordinate = Leg.Y_Coordinate + Y_roll_change
        Leg.Z_Coordinate = Leg.Z_Coordinate + Z_roll_change
        
    def calculate_roll(self):
        if self.roll == 0 and self.pitch == 0 and self.yaw == 0:
            self.Leg1_Y = Leg1.Y_Coordinate
            self.Leg1_Z = Leg1.Z_Coordinate
            self.Leg2_Y = Leg2.Y_Coordinate
            self.Leg2_Z = Leg2.Z_Coordinate
            self.Leg3_Y = Leg3.Y_Coordinate
            self.Leg3_Z = Leg3.Z_Coordinate
            self.Leg4_Y = Leg4.Y_Coordinate
            self.Leg4_Z = Leg4.Z_Coordinate
            self.Leg5_Y = Leg5.Y_Coordinate
            self.Leg5_Z = Leg5.Z_Coordinate
            self.Leg6_Y = Leg6.Y_Coordinate
            self.Leg6_Z = Leg6.Z_Coordinate

            self.calculate_leg_roll(Leg1, Leg1.Y_Coordinate, Leg1.Z_Coordinate)
            self.calculate_leg_roll(Leg2, Leg2.Y_Coordinate, Leg2.Z_Coordinate)
            self.calculate_leg_roll(Leg3, Leg3.Y_Coordinate, Leg3.Z_Coordinate)
            self.calculate_leg_roll(Leg4, Leg4.Y_Coordinate, Leg4.Z_Coordinate)
            self.calculate_leg_roll(Leg5, Leg5.Y_Coordinate, Leg5.Z_Coordinate)
            self.calculate_leg_roll(Leg6, Leg6.Y_Coordinate, Leg6.Z_Coordinate)
        else:
            self.calculate_leg_roll(Leg1, self.Leg1_Y, self.Leg1_Z)
            self.calculate_leg_roll(Leg2, self.Leg2_Y, self.Leg2_Z)
            self.calculate_leg_roll(Leg3, self.Leg3_Y, self.Leg3_Z)
            self.calculate_leg_roll(Leg4, self.Leg4_Y, self.Leg4_Z)
            self.calculate_leg_roll(Leg5, self.Leg5_Y, self.Leg5_Z)
            self.calculate_leg_roll(Leg6, self.Leg6_Y, self.Leg6_Z)     
        
    def calculate_leg_pitch(self, Leg, X, Z):
        ex = math.sqrt(X ** 2 + Z ** 2)
        psy = math.atan2(Z, X)
        phi = psy - self.pitch

        X_pitch_change = math.cos(phi) * ex - Leg.X_Coordinate
        Z_pitch_change = math.sin(phi) * ex - Leg.Z_Coordinate
        Leg.X_Coordinate = Leg.X_Coordinate + X_pitch_change
        Leg.Z_Coordinate = Leg.Z_Coordinate + Z_pitch_change

    def calculate_pitch(self):
        if self.roll == 0 and self.pitch == 0 and self.yaw == 0:
            self.Leg1_X = Leg1.X_Coordinate
            self.Leg1_Z = Leg1.Z_Coordinate
            self.Leg2_X = Leg2.X_Coordinate
            self.Leg2_Z = Leg2.Z_Coordinate
            self.Leg3_X = Leg3.X_Coordinate
            self.Leg3_Z = Leg3.Z_Coordinate
            self.Leg4_X = Leg4.X_Coordinate
            self.Leg4_Z = Leg4.Z_Coordinate
            self.Leg5_X = Leg5.X_Coordinate
            self.Leg5_Z = Leg5.Z_Coordinate
            self.Leg6_X = Leg6.X_Coordinate
            self.Leg6_Z = Leg6.Z_Coordinate

            self.calculate_leg_pitch(Leg1, Leg1.X_Coordinate, Leg1.Z_Coordinate)
            self.calculate_leg_pitch(Leg2, Leg2.X_Coordinate, Leg2.Z_Coordinate)
            self.calculate_leg_pitch(Leg3, Leg3.X_Coordinate, Leg3.Z_Coordinate)
            self.calculate_leg_pitch(Leg4, Leg4.X_Coordinate, Leg4.Z_Coordinate)
            self.calculate_leg_pitch(Leg5, Leg5.X_Coordinate, Leg5.Z_Coordinate)
            self.calculate_leg_pitch(Leg6, Leg6.X_Coordinate, Leg6.Z_Coordinate)
        else:
            self.calculate_leg_pitch(Leg1, self.Leg1_X, self.Leg1_Z)
            self.calculate_leg_pitch(Leg2, self.Leg2_X, self.Leg2_Z)
            self.calculate_leg_pitch(Leg3, self.Leg3_X, self.Leg3_Z)
            self.calculate_leg_pitch(Leg4, self.Leg4_X, self.Leg4_Z)
            self.calculate_leg_pitch(Leg5, self.Leg5_X, self.Leg5_Z)
            self.calculate_leg_pitch(Leg6, self.Leg6_X, self.Leg6_Z)
            
    def calculate_leg_yaw(self, Leg, X, Y):
        ex = math.sqrt(X ** 2 + Y ** 2)
        psy = math.atan2(Y, X)
        phi = psy - self.yaw
        Leg.X_Coordinate = math.cos(phi) * ex
        Leg.Y_Coordinate = math.sin(phi) * ex
        
    def calculate_yaw(self):
        if self.roll == 0 and self.pitch == 0 and self.yaw == 0:
            self.Leg1_X = Leg1.X_Coordinate
            self.Leg1_Y = Leg1.Y_Coordinate
            self.Leg2_X = Leg2.X_Coordinate
            self.Leg2_Y = Leg2.Y_Coordinate
            self.Leg3_X = Leg3.X_Coordinate
            self.Leg3_Y = Leg3.Y_Coordinate
            self.Leg4_X = Leg4.X_Coordinate
            self.Leg4_Y = Leg4.Y_Coordinate
            self.Leg5_X = Leg5.X_Coordinate
            self.Leg5_Y = Leg5.Y_Coordinate
            self.Leg6_X = Leg6.X_Coordinate
            self.Leg6_Y = Leg6.Y_Coordinate

            self.calculate_leg_yaw(Leg1, Leg1.X_Coordinate, Leg1.Y_Coordinate)
            self.calculate_leg_yaw(Leg2, Leg2.X_Coordinate, Leg2.Y_Coordinate)
            self.calculate_leg_yaw(Leg3, Leg3.X_Coordinate, Leg3.Y_Coordinate)
            self.calculate_leg_yaw(Leg4, Leg4.X_Coordinate, Leg4.Y_Coordinate)
            self.calculate_leg_yaw(Leg5, Leg5.X_Coordinate, Leg5.Y_Coordinate)
            self.calculate_leg_yaw(Leg6, Leg6.X_Coordinate, Leg6.Y_Coordinate)
        else:
            self.calculate_leg_yaw(Leg1, self.Leg1_X, self.Leg1_Y)
            self.calculate_leg_yaw(Leg2, self.Leg2_X, self.Leg2_Y)
            self.calculate_leg_yaw(Leg3, self.Leg3_X, self.Leg3_Y)
            self.calculate_leg_yaw(Leg4, self.Leg4_X, self.Leg4_Y)
            self.calculate_leg_yaw(Leg5, self.Leg5_X, self.Leg5_Y)
            self.calculate_leg_yaw(Leg6, self.Leg6_X, self.Leg6_Y)
            
    def calculate_leg_RPY(self, Leg, X, Y, Z):
        ex_roll = math.sqrt(Y ** 2 + Z ** 2)
        psy_roll = math.atan2(Y, Z)
        phi_roll = psy_roll - self.roll
        Y_roll_change = math.sin(phi_roll) * ex_roll - Y
        Z_roll_change = math.cos(phi_roll) * ex_roll - Z
        
        ex_pitch = math.sqrt(X ** 2 + Z ** 2)
        psy_pitch = math.atan2(Z, X)
        phi_pitch = psy_pitch - self.pitch
        X_pitch_change = math.cos(phi_pitch) * ex_pitch - X
        Z_pitch_change = math.sin(phi_pitch) * ex_pitch - Z

        ex_yaw = math.sqrt(X ** 2 + Y ** 2)
        psy_yaw = math.atan2(Y, X)
        phi_yaw = psy_yaw - self.yaw
        X_yaw_change = math.cos(phi_yaw) * ex_yaw - X
        Y_yaw_change = math.sin(phi_yaw) * ex_yaw - Y
        
        Leg.X_Coordinate = X + X_pitch_change + X_yaw_change
        Leg.Y_Coordinate = Y + Y_roll_change + Y_yaw_change
        Leg.Z_Coordinate = Z + Z_roll_change + Z_pitch_change

    def calculate_RPY(self):
        if self.roll == 0 and self.pitch == 0 and self.yaw == 0:
            self.Leg1_X = Leg1.X_Coordinate
            self.Leg1_Y = Leg1.Y_Coordinate
            self.Leg1_Z = Leg1.Z_Coordinate
            
            self.Leg2_X = Leg2.X_Coordinate
            self.Leg2_Y = Leg2.Y_Coordinate
            self.Leg2_Z = Leg2.Z_Coordinate
            
            self.Leg3_X = Leg3.X_Coordinate
            self.Leg3_Y = Leg3.Y_Coordinate
            self.Leg3_Z = Leg3.Z_Coordinate
            
            self.Leg4_X = Leg4.X_Coordinate
            self.Leg4_Y = Leg4.Y_Coordinate
            self.Leg4_Z = Leg4.Z_Coordinate
            
            self.Leg5_X = Leg5.X_Coordinate
            self.Leg5_Y = Leg5.Y_Coordinate
            self.Leg5_Z = Leg5.Z_Coordinate
            
            self.Leg6_X = Leg6.X_Coordinate
            self.Leg6_Y = Leg6.Y_Coordinate
            self.Leg6_Z = Leg6.Z_Coordinate

            self.calculate_leg_RPY(Leg1, Leg1.X_Coordinate, Leg1.Y_Coordinate, Leg1.Z_Coordinate)
            self.calculate_leg_RPY(Leg2, Leg2.X_Coordinate, Leg2.Y_Coordinate, Leg2.Z_Coordinate)
            self.calculate_leg_RPY(Leg3, Leg3.X_Coordinate, Leg3.Y_Coordinate, Leg3.Z_Coordinate)
            self.calculate_leg_RPY(Leg4, Leg4.X_Coordinate, Leg4.Y_Coordinate, Leg4.Z_Coordinate)
            self.calculate_leg_RPY(Leg5, Leg5.X_Coordinate, Leg5.Y_Coordinate, Leg5.Z_Coordinate)
            self.calculate_leg_RPY(Leg6, Leg6.X_Coordinate, Leg6.Y_Coordinate, Leg6.Z_Coordinate)
        else:
            self.calculate_leg_RPY(Leg1, self.Leg1_X, self.Leg1_Y, self.Leg1_Z)
            self.calculate_leg_RPY(Leg2, self.Leg2_X, self.Leg2_Y, self.Leg2_Z)
            self.calculate_leg_RPY(Leg3, self.Leg3_X, self.Leg3_Y, self.Leg3_Z)
            self.calculate_leg_RPY(Leg4, self.Leg4_X, self.Leg4_Y, self.Leg4_Z)
            self.calculate_leg_RPY(Leg5, self.Leg5_X, self.Leg5_Y, self.Leg5_Z)
            self.calculate_leg_RPY(Leg6, self.Leg6_X, self.Leg6_Y, self.Leg6_Z)
            
#=================================================================================

class Gaits:
    number_of_steps = 30#Number of steps in the gait cycle (inversly proportional to speed)
    step_number = 0#Current step of the gait cycle
    stride_length = 45.0#Maximum length of each step taken
    stride_height = 30.0#Maximum height of each step taken
    stride_profile = "Rectangular"#Trapize, Semicircular or Rectangular
    gaitType = "tripod"
    ICC = [0, 10000, "CCW"] #instantaneous centre of curvature [x, y "dir"]
    max_ICC_distance = 0#The distance the ICC is from the furthest leg
    max_base_ICC_distance = 0
    one_gait_cycle_flag = 0

    lifting_steps = 0
    lower_steps = 0
    support_steps = 0
    swing_steps = 0

    X_dist1 = 0
    Y_dist1 = 0
    X_dist2 = 0
    Y_dist2 = 0
    X_dist3 = 0
    Y_dist3 = 0
    X_dist4 = 0
    Y_dist4 = 0
    X_dist5 = 0
    Y_dist5 = 0
    X_dist6 = 0
    Y_dist6 = 0
    leg_ICC_distance1 = 0.0
    leg_ICC_distance2 = 0.0
    leg_ICC_distance3 = 0.0
    leg_ICC_distance4 = 0.0
    leg_ICC_distance5 = 0.0
    leg_ICC_distance6 = 0.0

    leg_base_ICC_distance1 = 0.0
    leg_base_ICC_distance2 = 0.0
    leg_base_ICC_distance3 = 0.0
    leg_base_ICC_distance4 = 0.0
    leg_base_ICC_distance5 = 0.0
    leg_base_ICC_distance6 = 0.0

    def stop(self):
        self.stride_length = 0.0#Maximum length of each step taken
        self.stride_height = 0.0#Maximum height of each step taken

    def ripple_123456_step_numbers(self):
        #Correctly scales the step number for each leg
        Leg6var = self.step_number + self.number_of_steps * 0/6
        Leg5var = self.step_number + self.number_of_steps * 1/6
        Leg4var = self.step_number + self.number_of_steps * 2/6
        Leg3var = self.step_number + self.number_of_steps * 3/6
        Leg2var = self.step_number + self.number_of_steps * 4/6
        Leg1var = self.step_number + self.number_of_steps * 5/6

        #Loops the step numbers
        if Leg1var >= self.number_of_steps:
            Leg1var = Leg1var - self.number_of_steps
        if Leg2var >= self.number_of_steps:
            Leg2var = Leg2var - self.number_of_steps
        if Leg3var >= self.number_of_steps:
            Leg3var = Leg3var - self.number_of_steps
        if Leg4var >= self.number_of_steps:
            Leg4var = Leg4var - self.number_of_steps
        if Leg5var >= self.number_of_steps:
            Leg5var = Leg5var - self.number_of_steps
        if Leg6var >= self.number_of_steps:
            Leg6var = Leg6var - self.number_of_steps

        if all((i < self.number_of_steps and i >= 0) for i in(Leg1var, Leg2var, Leg3var, Leg4var, Leg5var, Leg6var)):#Check all step number are in the correct range
             Leg1.step_number = Leg1var
             Leg2.step_number = Leg2var
             Leg3.step_number = Leg3var
             Leg4.step_number = Leg4var
             Leg5.step_number = Leg5var
             Leg6.step_number = Leg6var
        else:
            Exit_Error("Leg step_number out of accepted range")

        self.step_number = self.step_number + 1#Incements the step number

        if self.step_number >= self.number_of_steps:#Resets step number
            self.step_number = 0

    def tripod_Cycle_Steps(self, numberOfSteps):
        #Number of steps in each phase of the tripod gait
        self.lifting_steps = ((numberOfSteps/2)/(self.stride_length + 2 * self.stride_height)) * self.stride_height
        #print "lifting_steps = %f" % self.lifting_steps
        self.lower_steps = ((numberOfSteps/2)/(self.stride_length + 2 * self.stride_height)) * self.stride_height
        #print "lower_steps = %f" % self.lower_steps
        self.support_steps = numberOfSteps/2
        #print "support_steps = %f" % self.support_steps
        self.swing_steps = ((numberOfSteps/2)/(self.stride_length + 2 * self.stride_height)) * self.stride_length
        #print "swing_steps = %f" % self.swing_steps
        self.number_of_steps = numberOfSteps

    def tripod_step_numbers(self):
        if self.ICC[2] == "Stop":
            return 0
        #Correctly scales the step number for each leg
        Leg1var = self.step_number + self.number_of_steps * 0/6
        Leg2var = self.step_number + self.number_of_steps * 3/6
        Leg3var = self.step_number + self.number_of_steps * 3/6
        Leg4var = self.step_number + self.number_of_steps * 0/6
        Leg5var = self.step_number + self.number_of_steps * 0/6
        Leg6var = self.step_number + self.number_of_steps * 3/6

        #Loops the step numbers
        if Leg1var >= self.number_of_steps:
            Leg1var = Leg1var - self.number_of_steps
        if Leg2var >= self.number_of_steps:
            Leg2var = Leg2var - self.number_of_steps
        if Leg3var >= self.number_of_steps:
            Leg3var = Leg3var - self.number_of_steps
        if Leg4var >= self.number_of_steps:
            Leg4var = Leg4var - self.number_of_steps
        if Leg5var >= self.number_of_steps:
            Leg5var = Leg5var - self.number_of_steps
        if Leg6var >= self.number_of_steps:
            Leg6var = Leg6var - self.number_of_steps

        if all((i < self.number_of_steps and i >= 0) for i in(Leg1var, Leg2var, Leg3var, Leg4var, Leg5var, Leg6var)):#Check all step number are in the correct range
             Leg1.step_number = Leg1var
             Leg2.step_number = Leg2var
             Leg3.step_number = Leg3var
             Leg4.step_number = Leg4var
             Leg5.step_number = Leg5var
             Leg6.step_number = Leg6var
        else:
            Exit_Error("Leg step_number out of accepted range")

        self.step_number = self.step_number + 1#Incements the step number

        if self.step_number >= self.number_of_steps:#Resets step number
            self.step_number = 0

    def ripple_Cycle_Steps(self, numberOfSteps):
        #Number of steps in each phase of the ripple_123456 gait
        self.lifting_steps = ((numberOfSteps/6)/(self.stride_length + 2 * self.stride_height)) * self.stride_height
        #print "lifting_steps = %f" % self.lifting_steps
        self.lower_steps = ((numberOfSteps/6)/(self.stride_length + 2 * self.stride_height)) * self.stride_height
        #print "lower_steps = %f" % self.lower_steps
        self.support_steps = numberOfSteps*5/6
        #print "support_steps = %f" % self.support_steps
        self.swing_steps = ((numberOfSteps/6)/(self.stride_length + 2 * self.stride_height)) * self.stride_length
        #print "swing_steps = %f" % self.swing_steps
        self.number_of_steps = numberOfSteps

    def ripple_654321_step_numbers(self):
        #Correctly scales the step number for each leg
        Leg1var = self.step_number + self.number_of_steps * 0/6
        Leg2var = self.step_number + self.number_of_steps * 1/6
        Leg3var = self.step_number + self.number_of_steps * 2/6
        Leg4var = self.step_number + self.number_of_steps * 3/6
        Leg5var = self.step_number + self.number_of_steps * 4/6
        Leg6var = self.step_number + self.number_of_steps * 5/6

        #Loops the step numbers
        if Leg1var >= self.number_of_steps:
            Leg1var = Leg1var - self.number_of_steps
        if Leg2var >= self.number_of_steps:
            Leg2var = Leg2var - self.number_of_steps
        if Leg3var >= self.number_of_steps:
            Leg3var = Leg3var - self.number_of_steps
        if Leg4var >= self.number_of_steps:
            Leg4var = Leg4var - self.number_of_steps
        if Leg5var >= self.number_of_steps:
            Leg5var = Leg5var - self.number_of_steps
        if Leg6var >= self.number_of_steps:
            Leg6var = Leg6var - self.number_of_steps

        if all((i < self.number_of_steps and i >= 0) for i in(Leg1var, Leg2var, Leg3var, Leg4var, Leg5var, Leg6var)):#Check all step number are in the correct range
             Leg1.step_number = Leg1var
             Leg2.step_number = Leg2var
             Leg3.step_number = Leg3var
             Leg4.step_number = Leg4var
             Leg5.step_number = Leg5var
             Leg6.step_number = Leg6var
        else:
            Exit_Error("Leg step_number out of accepted range")

        self.step_number = self.step_number + 1#Incements the step number

        if self.step_number >= self.number_of_steps:#Resets step number
            self.step_number = 0
            
    def ripple_135246_step_numbers(self):
        #Correctly scales the step number for each leg
        Leg1var = self.step_number + self.number_of_steps * 5/6
        Leg2var = self.step_number + self.number_of_steps * 2/6
        Leg3var = self.step_number + self.number_of_steps * 4/6
        Leg4var = self.step_number + self.number_of_steps * 1/6
        Leg5var = self.step_number + self.number_of_steps * 3/6
        Leg6var = self.step_number + self.number_of_steps * 0/6

        #Loops the step numbers
        if Leg1var >= self.number_of_steps:
            Leg1var = Leg1var - self.number_of_steps
        if Leg2var >= self.number_of_steps:
            Leg2var = Leg2var - self.number_of_steps
        if Leg3var >= self.number_of_steps:
            Leg3var = Leg3var - self.number_of_steps
        if Leg4var >= self.number_of_steps:
            Leg4var = Leg4var - self.number_of_steps
        if Leg5var >= self.number_of_steps:
            Leg5var = Leg5var - self.number_of_steps
        if Leg6var >= self.number_of_steps:
            Leg6var = Leg6var - self.number_of_steps

        if all((i < self.number_of_steps and i >= 0) for i in(Leg1var, Leg2var, Leg3var, Leg4var, Leg5var, Leg6var)):#Check all step number are in the correct range
             Leg1.step_number = Leg1var
             Leg2.step_number = Leg2var
             Leg3.step_number = Leg3var
             Leg4.step_number = Leg4var
             Leg5.step_number = Leg5var
             Leg6.step_number = Leg6var
        else:
            Exit_Error("Leg step_number out of accepted range")

        self.step_number = self.step_number + 1#Incements the step number

        if self.step_number >= self.number_of_steps:#Resets step number
            self.step_number = 0
            
    def double_ripple_Cycle_Steps(self, numberOfSteps):
        #Number of steps in each phase of the ripple_123456 gait
        self.lifting_steps = ((numberOfSteps/3)/(self.stride_length + 2 * self.stride_height)) * self.stride_height
        #print "lifting_steps = %f" % self.lifting_steps
        self.lower_steps = ((numberOfSteps/3)/(self.stride_length + 2 * self.stride_height)) * self.stride_height
        #print "lower_steps = %f" % self.lower_steps
        self.support_steps = numberOfSteps*2/3
        #print "support_steps = %f" % self.support_steps
        self.swing_steps = ((numberOfSteps/3)/(self.stride_length + 2 * self.stride_height)) * self.stride_length
        #print "swing_steps = %f" % self.swing_steps
        self.number_of_steps = numberOfSteps
        
    def double_ripple_14_36_52(self):#sets the step numbers for each leg
        if self.ICC[2] == "Stop":
            return 0
        
        #Correctly scales the step number for each leg
        Leg1var = self.step_number + self.number_of_steps * 2/3
        Leg2var = self.step_number + self.number_of_steps * 0/3
        Leg3var = self.step_number + self.number_of_steps * 1/3
        Leg4var = self.step_number + self.number_of_steps * 2/3
        Leg5var = self.step_number + self.number_of_steps * 0/3
        Leg6var = self.step_number + self.number_of_steps * 1/3

        #Loops the step numbers
        if Leg1var >= self.number_of_steps:
            Leg1var = Leg1var - self.number_of_steps
        if Leg2var >= self.number_of_steps:
            Leg2var = Leg2var - self.number_of_steps
        if Leg3var >= self.number_of_steps:
            Leg3var = Leg3var - self.number_of_steps
        if Leg4var >= self.number_of_steps:
            Leg4var = Leg4var - self.number_of_steps
        if Leg5var >= self.number_of_steps:
            Leg5var = Leg5var - self.number_of_steps
        if Leg6var >= self.number_of_steps:
            Leg6var = Leg6var - self.number_of_steps

        if all((i < self.number_of_steps and i >= 0) for i in(Leg1var, Leg2var, Leg3var, Leg4var, Leg5var, Leg6var)):#Check all step number are in the correct range
             Leg1.step_number = Leg1var
             Leg2.step_number = Leg2var
             Leg3.step_number = Leg3var
             Leg4.step_number = Leg4var
             Leg5.step_number = Leg5var
             Leg6.step_number = Leg6var
        else:
            Exit_Error("Leg step_number out of accepted range")

        self.step_number = self.step_number + 1#Incements the step number

        if self.step_number >= self.number_of_steps:#Resets step number
            self.step_number = 0
            
    def reset_step_numbers(self):
        Leg1.step_number = 0
        Leg2.step_number = 0
        Leg3.step_number = 0
        Leg4.step_number = 0
        Leg5.step_number = 0
        Leg6.step_number = 0
        self.step_number = 0
        
    def reset_Gait_variables(self):
        self.max_ICC_distance = 0.0#The distance the ICC is from the furthest leg

    def calculate_ICC(self):#Reads in from the controller and calculates the ICC
        #print "Calculate the ICC"
        #analog read
        self.ICC[0] = 0 #analog read with some maths
        self.ICC[1] = 100000 #analog read with some maths
        self.ICC[2] = "CCW"

        #Checks the ICC parameters are valid
        if isinstance(self.ICC[0], numbers.Number) == False or isinstance(self.ICC[1], numbers.Number) == False or self.ICC[2] not in("CW", "CCW", "Stop"):
            Exit_Error("Invalid ICC parameters")#Exits the script and prints an error message

    def calculate_leg_ICC_distances(self):#Split this down to 3 function for each pair of legs ready for multi threading
        #print "Calculate the leg to ICC distances"
        self.max_ICC_distance = 0.0

        self.X_dist1 = Leg1.X_Coordinate - self.ICC[0]#X Distance from ICC to the leg end effector
        self.Y_dist1 = Leg1.Y_Coordinate - self.ICC[1]#Y Distance from ICC to the leg end effector
        self.leg_ICC_distance1 = math.sqrt(self.X_dist1**2 + self.Y_dist1**2)#Distance from ICC to the leg end effector

        if self.leg_ICC_distance1 > self.max_ICC_distance:
            self.max_ICC_distance = self.leg_ICC_distance1

        self.X_dist2 = Leg2.X_Coordinate - self.ICC[0]#X Distance from ICC to the leg end effector
        self.Y_dist2 = Leg2.Y_Coordinate - self.ICC[1]#Y Distance from ICC to the leg end effector
        self.leg_ICC_distance2 = math.sqrt(self.X_dist2**2 + self.Y_dist2**2)#Distance from ICC to the leg end effector

        if self.leg_ICC_distance2 > self.max_ICC_distance:
            self.max_ICC_distance = self.leg_ICC_distance2

        self.X_dist3 = Leg3.X_Coordinate - self.ICC[0]#X Distance from ICC to the leg end effector
        self.Y_dist3 = Leg3.Y_Coordinate - self.ICC[1]#Y Distance from ICC to the leg end effector
        self.leg_ICC_distance3 = math.sqrt(self.X_dist3**2 + self.Y_dist3**2)#Distance from ICC to the leg end effector

        if self.leg_ICC_distance3 > self.max_ICC_distance:
            self.max_ICC_distance = self.leg_ICC_distance3

        self.X_dist4 = Leg4.X_Coordinate - self.ICC[0]#X Distance from ICC to the leg end effector
        self.Y_dist4 = Leg4.Y_Coordinate - self.ICC[1]#Y Distance from ICC to the leg end effector
        self.leg_ICC_distance4 = math.sqrt(self.X_dist4**2 + self.Y_dist4**2)#Distance from ICC to the leg end effector

        if self.leg_ICC_distance4 > self.max_ICC_distance:
            self.max_ICC_distance = self.leg_ICC_distance4

        self.X_dist5 = Leg5.X_Coordinate - self.ICC[0]#X Distance from ICC to the leg end effector
        self.Y_dist5 = Leg5.Y_Coordinate - self.ICC[1]#Y Distance from ICC to the leg end effector
        self.leg_ICC_distance5 = math.sqrt(self.X_dist5**2 + self.Y_dist5**2)#Distance from ICC to the leg end effector

        if self.leg_ICC_distance5 > self.max_ICC_distance:
            self.max_ICC_distance = self.leg_ICC_distance5

        self.X_dist6 = Leg6.X_Coordinate - self.ICC[0]#X Distance from ICC to the leg end effector
        self.Y_dist6 = Leg6.Y_Coordinate - self.ICC[1]#Y Distance from ICC to the leg end effector
        self.leg_ICC_distance6 = math.sqrt(self.X_dist6**2 + self.Y_dist6**2)#Distance from ICC to the leg end effector

        if self.leg_ICC_distance6 > self.max_ICC_distance:
            self.max_ICC_distance = self.leg_ICC_distance6

    def calculate_base_leg_ICC_distances(self):#Split this down to 3 function for each pair of legs ready for multi threading
        #print "Calculate the leg to ICC distances"
        self.max_base_ICC_distance = 0.0

        self.leg_base_ICC_distance1 = math.sqrt((Leg1.X_base_position - self.ICC[0])**2 + (Leg1.Y_base_position - self.ICC[1])**2)#Distance from ICC to the leg end effector

        if self.leg_base_ICC_distance1 > self.max_base_ICC_distance:
            self.max_base_ICC_distance = self.leg_base_ICC_distance1

        self.leg_base_ICC_distance2 = math.sqrt((Leg2.X_base_position - self.ICC[0])**2 + (Leg2.Y_base_position - self.ICC[1])**2)#Distance from ICC to the leg end effector

        if self.leg_base_ICC_distance2 > self.max_base_ICC_distance:
            self.max_base_ICC_distance = self.leg_base_ICC_distance2

        self.leg_base_ICC_distance3 = math.sqrt((Leg3.X_base_position - self.ICC[0])**2 + (Leg3.Y_base_position - self.ICC[1])**2)#Distance from ICC to the leg end effector

        if self.leg_base_ICC_distance3 > self.max_base_ICC_distance:
            self.max_base_ICC_distance = self.leg_base_ICC_distance3

        self.leg_base_ICC_distance4 = math.sqrt((Leg4.X_base_position - self.ICC[0])**2 + (Leg4.Y_base_position - self.ICC[1])**2)#Distance from ICC to the leg end effector

        if self.leg_base_ICC_distance4 > self.max_base_ICC_distance:
            self.max_base_ICC_distance = self.leg_base_ICC_distance4

        self.leg_base_ICC_distance5 = math.sqrt((Leg5.X_base_position - self.ICC[0])**2 + (Leg5.Y_base_position - self.ICC[1])**2)#Distance from ICC to the leg end effector

        if self.leg_base_ICC_distance5 > self.max_base_ICC_distance:
            self.max_base_ICC_distance = self.leg_base_ICC_distance5

        self.leg_base_ICC_distance6 = math.sqrt((Leg6.X_base_position - self.ICC[0])**2 + (Leg6.Y_base_position - self.ICC[1])**2)#Distance from ICC to the leg end effector

        if self.leg_base_ICC_distance6 > self.max_base_ICC_distance:
            self.max_base_ICC_distance = self.leg_base_ICC_distance6

    def calculate_leg_positions(self):#Calculates all the legs target positions
        #self.calculate_ICC()#Calculates the ICC position
        ICC_Parallel()#Calculates the ICC position
        self.calculate_leg_ICC_distances()#Calculates the ICC distances from each leg and the furthest distance from a leg
        self.calculate_base_leg_ICC_distances()#Calculates the distances from each leg's base position to the ICC

        self.calculate_target_position(Leg1, self.leg_ICC_distance1, self.leg_base_ICC_distance1, self.X_dist1, self.Y_dist1)#Calculates the target position for the leg
        self.calculate_target_position(Leg3, self.leg_ICC_distance3, self.leg_base_ICC_distance3, self.X_dist3, self.Y_dist3)#Calculates the target position for the leg
        self.calculate_target_position(Leg5, self.leg_ICC_distance5, self.leg_base_ICC_distance5, self.X_dist5, self.Y_dist5)#Calculates the target position for the leg

        self.calculate_target_position(Leg2, self.leg_ICC_distance2, self.leg_base_ICC_distance2, self.X_dist2, self.Y_dist2)#Calculates the target position for the leg
        self.calculate_target_position(Leg4, self.leg_ICC_distance4, self.leg_base_ICC_distance4, self.X_dist4, self.Y_dist4)#Calculates the target position for the leg
        self.calculate_target_position(Leg6, self.leg_ICC_distance6, self.leg_base_ICC_distance6, self.X_dist6, self.Y_dist6)#Calculates the target position for the leg

    def calculate_leg_positions_without_ICC_Parallel(self):
        self.calculate_leg_ICC_distances()#Calculates the ICC distances from each leg and the furthest distance from a leg
        self.calculate_base_leg_ICC_distances()#Calculates the distances from each leg's base position to the ICC

        self.calculate_target_position(Leg1, self.leg_ICC_distance1, self.leg_base_ICC_distance1, self.X_dist1, self.Y_dist1)#Calculates the target position for the leg
        self.calculate_target_position(Leg3, self.leg_ICC_distance3, self.leg_base_ICC_distance3, self.X_dist3, self.Y_dist3)#Calculates the target position for the leg
        self.calculate_target_position(Leg5, self.leg_ICC_distance5, self.leg_base_ICC_distance5, self.X_dist5, self.Y_dist5)#Calculates the target position for the leg

        self.calculate_target_position(Leg2, self.leg_ICC_distance2, self.leg_base_ICC_distance2, self.X_dist2, self.Y_dist2)#Calculates the target position for the leg
        self.calculate_target_position(Leg4, self.leg_ICC_distance4, self.leg_base_ICC_distance4, self.X_dist4, self.Y_dist4)#Calculates the target position for the leg
        self.calculate_target_position(Leg6, self.leg_ICC_distance6, self.leg_base_ICC_distance6, self.X_dist6, self.Y_dist6)#Calculates the target position for the leg


    def calculate_target_position(self, leg_object, leg_ICC_distance, leg_base_ICC_distance, X_dist, Y_dist):
        #print "Calculate the leg target positions"
        X_target = leg_object.X_Coordinate
        Y_target = leg_object.Y_Coordinate
        Z_target = leg_object.Z_Coordinate

        if leg_object.step_number >= 0 and leg_object.step_number <= self.support_steps:#Supporting phase
            leg_stride_length = self.stride_length * leg_ICC_distance / self.max_ICC_distance#Scales stride length to the proportional size for the distance from the ICC
            alpha = 2 * math.asin(0.5 * leg_stride_length / leg_ICC_distance)
            alpha_step = alpha/self.support_steps
            beta = math.atan2(Y_dist, X_dist)

            if self.ICC[2] == "CW":
                beta = beta + alpha_step
            elif self.ICC[2] == "CCW":
                beta = beta - alpha_step
            elif self.ICC[2] == "Stop":
                return 0

            X_from_ICC = math.cos(beta) * leg_ICC_distance
            Y_from_ICC = math.sin(beta) * leg_ICC_distance

            X_target = X_from_ICC + self.ICC[0]
            Y_target = Y_from_ICC + self.ICC[1]
            Z_target = leg_object.Z_base_position
        elif leg_object.step_number > self.support_steps and leg_object.step_number <= self.support_steps + self.lifting_steps and self.ICC[2] != "Stop":#Lifting phase
            Z_target = leg_object.Z_Coordinate + self.stride_height/self.lifting_steps
        elif leg_object.step_number > self.support_steps + self.lifting_steps and leg_object.step_number <= self.support_steps + self.lifting_steps + self.swing_steps:#Swing phase
            leg_stride_length = self.stride_length * leg_base_ICC_distance / self.max_base_ICC_distance#Scales stride length to the proportional size for the distance from the ICC
            alpha = math.asin(0.5 * leg_stride_length / leg_base_ICC_distance)#need to check all this properly
            gamma = math.atan2(leg_object.Y_base_position - self.ICC[1], leg_object.X_base_position - self.ICC[0])

            if self.ICC[2] == "CW":
                beta = gamma - alpha
            elif self.ICC[2] == "CCW":
                beta = gamma + alpha
            elif self.ICC[2] == "Stop":
                return 0

            X_from_ICC = math.cos(beta) * leg_base_ICC_distance
            Y_from_ICC = math.sin(beta) * leg_base_ICC_distance

            X_final_target = X_from_ICC + self.ICC[0]
            Y_final_target = Y_from_ICC + self.ICC[1]

            X_target = leg_object.X_Coordinate + ((X_final_target - leg_object.X_Coordinate) / self.swing_steps) * (leg_object.step_number - (self.support_steps + self.lifting_steps))
            Y_target = leg_object.Y_Coordinate + ((Y_final_target - leg_object.Y_Coordinate) / self.swing_steps) * (leg_object.step_number - (self.support_steps + self.lifting_steps))

        elif leg_object.step_number > self.support_steps + self.lifting_steps + self.swing_steps and leg_object.step_number <= self.support_steps + self.lifting_steps + self.swing_steps + self.lower_steps  and self.ICC[2] != "Stop":#Lowering phase
            Z_target = leg_object.Z_Coordinate - self.stride_height/self.lower_steps
        elif self.ICC[2] != "Stop":
            print "Calculate target position failed"
            #Exit_Error("Calculate target position failed")

        #Sets the leg desired coordinates to the target coordinates
        leg_object.X_Coordinate = X_target
        leg_object.Y_Coordinate = Y_target
        leg_object.Z_Coordinate = Z_target

    def standUp(self):
        #print "standing up"
        Leg1.Y_Coordinate = 217.0
        Leg1.Z_Coordinate = -20
        Leg2.Y_Coordinate = -217.0
        Leg2.Z_Coordinate = -20
        Leg3.Y_Coordinate = 217.0
        Leg3.Z_Coordinate = -20
        Leg4.Y_Coordinate = -217.0
        Leg4.Z_Coordinate = -20
        Leg5.Y_Coordinate = 217.0
        Leg5.Z_Coordinate = -20
        Leg6.Y_Coordinate = -217.0
        Leg6.Z_Coordinate = -20
        leg_Inverse_Kinamatics()
        moveLegs()
        time.sleep(3)

        for i in range(-20, -110, -1):
            Leg1.Z_Coordinate = i
            Leg2.Z_Coordinate = i
            Leg3.Z_Coordinate = i
            Leg4.Z_Coordinate = i
            Leg5.Z_Coordinate = i
            Leg6.Z_Coordinate = i
            leg_Inverse_Kinamatics()
            moveLegs()

    def leg_to_base_position(self, leg, XYnumber, zNumber):
        Z_change_step = self.stride_height/zNumber
        X_change_step = (leg.X_base_position - leg.X_Coordinate)/XYnumber
        Y_change_step = (leg.Y_base_position - leg.Y_Coordinate)/XYnumber
        
        for i in range(0, zNumber, 1):#for loop raises the leg
            leg.Z_Coordinate = leg.Z_Coordinate + Z_change_step
            leg.inverse_kinematics()#Calculates the inverse kinematics for the leg
            moveLegs()

        for i in range(0, XYnumber, 1):#for loop moves the leg to the X, Y base positions
            leg.X_Coordinate = leg.X_Coordinate + X_change_step
            leg.Y_Coordinate = leg.Y_Coordinate + Y_change_step
            leg.inverse_kinematics()#Calculates the inverse kinematics for the leg
            moveLegs() 

        leg.X_Coordinate = leg.X_base_position
        leg.Y_Coordinate = leg.Y_base_position
                       
        for i in range(zNumber, 0, -1):#for loop lowers the leg
            leg.Z_Coordinate = leg.Z_Coordinate - Z_change_step
            leg.inverse_kinematics()#Calculates the inverse kinematics for the leg
            moveLegs()
            
        self.reset_step_numbers()
        
    def all_legs_support(self, inverse_Speed):
        
        if self.ICC[2] == "Stop":
            zStep = self.stride_height/inverse_Speed
            for i in range (0, inverse_Speed, 1):
                if Leg1.Z_Coordinate > body.height:
                    Leg1.Z_Coordinate = Leg1.Z_Coordinate - zStep
                    leg1flag = 0
                else:
                    leg1flag = 1
                if Leg2.Z_Coordinate > body.height:
                    Leg2.Z_Coordinate = Leg2.Z_Coordinate - zStep
                    leg2flag = 0
                else:
                    leg2flag = 1
                if Leg3.Z_Coordinate > body.height:
                    Leg3.Z_Coordinate = Leg3.Z_Coordinate - zStep
                    leg3flag = 0
                else:
                    leg3flag = 1
                if Leg4.Z_Coordinate > body.height:
                    Leg4.Z_Coordinate = Leg4.Z_Coordinate - zStep
                    leg4flag = 0
                else:
                    leg4flag = 1
                if Leg5.Z_Coordinate > body.height:
                    Leg5.Z_Coordinate = Leg5.Z_Coordinate - zStep
                    leg5flag = 0
                else:
                    leg5flag = 1
                if Leg6.Z_Coordinate > body.height:
                    Leg6.Z_Coordinate = Leg6.Z_Coordinate - zStep
                    leg6flag = 0
                else:
                    leg6flag = 1
                    
                if leg1flag == 0 or leg2flag == 0 or leg3flag == 0 or leg4flag == 0 or  leg5flag == 0 or leg6flag == 0:     
                    leg_Inverse_Kinamatics()#Calculate inverse kinematics
                    moveLegs()#Moves the leg servos
                else:
                    break
                
    def reposition_legs_to_base(self, inverse_speed):
        self.all_legs_support(inverse_speed)
        self.leg_to_base_position(Leg1, inverse_speed, inverse_speed)
        self.leg_to_base_position(Leg2, inverse_speed, inverse_speed)
        self.leg_to_base_position(Leg3, inverse_speed, inverse_speed)
        self.leg_to_base_position(Leg4, inverse_speed, inverse_speed)
        self.leg_to_base_position(Leg5, inverse_speed, inverse_speed)
        self.leg_to_base_position(Leg6, inverse_speed, inverse_speed)
        
    def legs_to_base_position(self, inverse_speed, LegA = "None", LegB = "None", LegC = "None"):
        self.all_legs_support(5)
    
        if not (LegA == Leg1 or LegA == Leg2 or LegA == Leg3 or LegA == Leg4 or LegA == Leg5 or LegA == Leg6 or "None"):
            return 0

        if LegA == Leg1 or LegA == Leg2 or LegA == Leg3 or LegA == Leg4 or LegA == Leg5 or LegA == Leg6:
            LegA_X_step = (LegA.X_base_position - LegA.X_Coordinate)/inverse_speed
            LegA_Y_step = (LegA.Y_base_position - LegA.Y_Coordinate)/inverse_speed
            LegA_Z_step = self.stride_height/inverse_speed
            
        if LegB == Leg1 or LegB == Leg2 or LegB == Leg3 or LegB == Leg4 or LegB == Leg5 or LegB == Leg6:
            LegB_X_step = (LegB.X_base_position - LegB.X_Coordinate)/inverse_speed
            LegB_Y_step = (LegB.Y_base_position - LegB.Y_Coordinate)/inverse_speed
            LegB_Z_step = self.stride_height/inverse_speed
            
        if LegC == Leg1 or LegC == Leg2 or LegC == Leg3 or LegC == Leg4 or LegC == Leg5 or LegC == Leg6:
            LegC_X_step = (LegC.X_base_position - LegC.X_Coordinate)/inverse_speed
            LegC_Y_step = (LegC.Y_base_position - LegC.Y_Coordinate)/inverse_speed
            LegC_Z_step = self.stride_height/inverse_speed
        
        for i in range(0, inverse_speed, 1):#for loop raises the leg
            if LegA == Leg1 or LegA == Leg2 or LegA == Leg3 or LegA == Leg4 or LegA == Leg5 or LegA == Leg6:
                LegA.Z_Coordinate = LegA.Z_Coordinate + LegA_Z_step
                LegA.inverse_kinematics()#Calculates the inverse kinematics for the leg
                
            if LegB == Leg1 or LegB == Leg2 or LegB == Leg3 or LegB == Leg4 or LegB == Leg5 or LegB == Leg6:  
                LegB.Z_Coordinate = LegB.Z_Coordinate + LegA_Z_step
                LegB.inverse_kinematics()#Calculates the inverse kinematics for the leg

            if LegC == Leg1 or LegC == Leg2 or LegC == Leg3 or LegC == Leg4 or LegC == Leg5 or LegC == Leg6:
                LegC.Z_Coordinate = LegC.Z_Coordinate + LegA_Z_step
                LegC.inverse_kinematics()#Calculates the inverse kinematics for the leg
                
            moveLegs()

        for i in range(0, inverse_speed, 1):#for loop moves the leg to the X, Y base positions
            if LegA == Leg1 or LegA == Leg2 or LegA == Leg3 or LegA == Leg4 or LegA == Leg5 or LegA == Leg6:
                LegA.X_Coordinate = LegA.X_Coordinate + LegA_X_step
                LegA.Y_Coordinate = LegA.Y_Coordinate + LegA_Y_step
                LegA.inverse_kinematics()#Calculates the inverse kinematics for the leg
                
            if LegB == Leg1 or LegB == Leg2 or LegB == Leg3 or LegB == Leg4 or LegB == Leg5 or LegB == Leg6:  
                LegB.X_Coordinate = LegB.X_Coordinate + LegB_X_step
                LegB.Y_Coordinate = LegB.Y_Coordinate + LegB_Y_step
                LegB.inverse_kinematics()#Calculates the inverse kinematics for the leg

            if LegC == Leg1 or LegC == Leg2 or LegC == Leg3 or LegC == Leg4 or LegC == Leg5 or LegC == Leg6:
                LegC.X_Coordinate = LegC.X_Coordinate + LegC_X_step
                LegC.Y_Coordinate = LegC.Y_Coordinate + LegC_Y_step 
                LegC.inverse_kinematics()#Calculates the inverse kinematics for the leg
                
            moveLegs()

        if LegA == Leg1 or LegA == Leg2 or LegA == Leg3 or LegA == Leg4 or LegA == Leg5 or LegA == Leg6:
            LegA.X_Coordinate = LegA.X_base_position
            LegA.Y_Coordinate = LegA.Y_base_position
        
        if LegB == Leg1 or LegB == Leg2 or LegB == Leg3 or LegB == Leg4 or LegB == Leg5 or LegB == Leg6:
            LegB.X_Coordinate = LegB.X_base_position
            LegB.Y_Coordinate = LegB.Y_base_position
            
        if LegC == Leg1 or LegC == Leg2 or LegC == Leg3 or LegC == Leg4 or LegC == Leg5 or LegC == Leg6:
            LegC.X_Coordinate = LegC.X_base_position
            LegC.Y_Coordinate = LegC.Y_base_position
                       
        for i in range(inverse_speed, 0, -1):#for loop lowers the leg
            if LegA == Leg1 or LegA == Leg2 or LegA == Leg3 or LegA == Leg4 or LegA == Leg5 or LegA == Leg6:
                LegA.Z_Coordinate = LegA.Z_Coordinate - LegA_Z_step
                LegA.inverse_kinematics()#Calculates the inverse kinematics for the leg
                
            if LegB == Leg1 or LegB == Leg2 or LegB == Leg3 or LegB == Leg4 or LegB == Leg5 or LegB == Leg6:  
                LegB.Z_Coordinate = LegB.Z_Coordinate - LegA_Z_step
                LegB.inverse_kinematics()#Calculates the inverse kinematics for the leg

            if LegC == Leg1 or LegC == Leg2 or LegC == Leg3 or LegC == Leg4 or LegC == Leg5 or LegC == Leg6:
                LegC.Z_Coordinate = LegC.Z_Coordinate - LegA_Z_step
                LegC.inverse_kinematics()#Calculates the inverse kinematics for the leg
                
            moveLegs()
            
        self.reset_step_numbers()

    def tripod_legs_to_base_position(self, inverse_speed):
        self.legs_to_base_position(inverse_speed, LegA = Leg1, LegB = Leg4, LegC = Leg5)
        self.legs_to_base_position(inverse_speed, LegA = Leg2, LegB = Leg3, LegC = Leg6)
        
    def double_ripple_legs_to_base_position(self, inverse_speed):
        self.legs_to_base_position(inverse_speed, LegA = Leg1, LegB = Leg4)
        self.legs_to_base_position(inverse_speed, LegA = Leg3, LegB = Leg6)
        self.legs_to_base_position(inverse_speed, LegA = Leg5, LegB = Leg2)
        
    def update_step_numbers(self):
        if self.gaitType == "tripod":
            self.tripod_step_numbers()
        elif self.gaitType == "double_ripple":
            self.double_ripple_14_36_52()
        elif self.gaitType == "ripple_123456":
            self.ripple_123456_step_numbers()
        elif self.gaitType == "ripple_654321":
            self.ripple_654321_step_numbers()
        elif self.gaitType == "ripple_135246":
            self.ripple_135246_step_numbers()
        else:
            Exit_Error("Updating step numbers failed. (No matching gait type.)")

        if self.step_number == 0 and self.ICC[2] != "Stop":
            self.one_gait_cycle_flag = 1
            
        if self.one_gait_cycle_flag == 0:
            self.stride_length = 20
        else:
            self.stride_length = 45
            
    def change_gait(self, inverse_Support_Speed, numberOfSteps, gait_Type):
        self.all_legs_support(inverse_Support_Speed)#Puts all the legs on the ground
        self.reset_step_numbers()#Resets all the step numbers
        self.gaitType = gait_Type
        self.number_of_steps = numberOfSteps
        self.one_gait_cycle_flag = 0
        if self.gaitType == "tripod":
            self.tripod_Cycle_Steps(self.number_of_steps)
        elif self.gaitType == "double_ripple":
            self.double_ripple_Cycle_Steps(self.number_of_steps)
        elif self.gaitType == "ripple_123456":
            self.ripple_Cycle_Steps(self.number_of_steps)
        elif self.gaitType == "ripple_654321":
            self.ripple_Cycle_Steps(self.number_of_steps)
        elif self.gaitType == "ripple_135246":
            self.ripple_Cycle_Steps(self.number_of_steps)
        else:
            Exit_Error("Invalid gait type.")
            
    def ripple_135246(self):
        print "ripple_135246 gait"

    def ripple_145236(self):
        print "ripple_145236 gait"

    def catapilar(self):
        print "Catapilar gait"

    def quadrapod(self):
        print "Quadrapod gait"

#=================================================================================

class Eyes:
    rToggle = 0
    gToggle = 0
    bToggle = 0

    def toggleRed(self):
        Red = map_number(self.rToggle, 0, 1, 0, 4095)
        pwm3.setPWM(0, 0, Red)#Sets the RGB Red
        self.rToggle ^= 1

    def toggleGreen(self):
        Green = map_number(self.gToggle, 0, 1, 0, 4095)
        pwm3.setPWM(1, 0, Green)#Sets the RGB Green
        self.gToggle ^= 1

    def toggleBlue(self):
        Blue = map_number(self.bToggle, 0, 1, 0, 4095)
        pwm3.setPWM(2, 0,Blue)#Sets the RGB Blue
        self.bToggle ^= 1

    def custom_eye_colour(self, R, G, B):
        Blue = map_number(B, 0, 255, 0, 4095)
        Green = map_number(G, 0, 255, 0, 4095)
        Red = map_number(R, 0, 255, 0, 4095)

        pwm3.setPWM(0, 0, Red)#Sets RGB LED Red
        pwm3.setPWM(1, 0, Green)#Sets RGB LED Green
        pwm3.setPWM(2, 0, Blue)#Sets RGB LED Blue

    def red(self):#sets eye colour to red
        pwm3.setPWM(0, 0, 4095)#Sets RGB LED Red
        pwm3.setPWM(1, 0, 0)#Sets RGB LED Green
        pwm3.setPWM(2, 0, 0)#Sets RGB LED Blue

    def green(self):#sets eye colour to green
        pwm3.setPWM(0, 0, 0)#Sets RGB LED Red
        pwm3.setPWM(1, 0, 4095)#Sets RGB LED Green
        pwm3.setPWM(2, 0, 0)#Sets RGB LED Blue

    def blue(self):#sets eye colour to blue
        pwm3.setPWM(0, 0, 0)#Sets RGB LED Red
        pwm3.setPWM(1, 0, 0)#Sets RGB LED Green
        pwm3.setPWM(2, 0, 4095)#Sets RGB LED Blue

    def yellow(self):#sets eye colour to yellow
        pwm3.setPWM(0, 0, 4095)#Sets RGB LED Red
        pwm3.setPWM(1, 0, 4095)#Sets RGB LED Green
        pwm3.setPWM(2, 0, 0)#Sets RGB LED Blue

    def violet(self):#sets eye colour to violet
        pwm3.setPWM(0, 0, 4095)#Sets RGB LED Red
        pwm3.setPWM(1, 0, 0)#Sets RGB LED Green
        pwm3.setPWM(2, 0, 4095)#Sets RGB LED Blue

    def turquoise(self):#sets eye colour to violet
        pwm3.setPWM(0, 0, 0)#Sets RGB LED Red
        pwm3.setPWM(1, 0, 4095)#Sets RGB LED Green
        pwm3.setPWM(2, 0, 2048)#Sets RGB LED Blue

    def orange(self):#sets eye colour to orange
        pwm3.setPWM(0, 0, 4095)#Sets RGB LED Red
        pwm3.setPWM(1, 0, 1024)#Sets RGB LED Green
        pwm3.setPWM(2, 0, 0)#Sets RGB LED Blue

    def purple(self):#sets eye colour to purple
        pwm3.setPWM(0, 0, 4095)#Sets RGB LED Red
        pwm3.setPWM(1, 0, 0)#Sets RGB LED Green
        pwm3.setPWM(2, 0, 3000)#Sets RGB LED Blue

    def pink(self):#sets eye colour to pink
        pwm3.setPWM(0, 0, 4095)#Sets RGB LED Red
        pwm3.setPWM(1, 0, 0)#Sets RGB LED Green
        pwm3.setPWM(2, 0, 2000)#Sets RGB LED Blue

    def white(self):#sets eye colour to white
        pwm3.setPWM(0, 0, 4095)#Sets RGB LED Red
        pwm3.setPWM(1, 0, 4095)#Sets RGB LED Green
        pwm3.setPWM(2, 0, 2250)#Sets RGB LED Blue

    def off(self):#sets eye colour to white
        pwm3.setPWM(0, 0, 0)#Sets RGB LED Red
        pwm3.setPWM(1, 0, 0)#Sets RGB LED Green
        pwm3.setPWM(2, 0, 0)#Sets RGB LED Blue

    def green_to_red(self):
        pwm3.setPWM(0, 0, 0)#Sets RGB LED Red
        pwm3.setPWM(1, 0, 4095)#Sets RGB LED Green
        pwm3.setPWM(2, 0, 0)#Sets RGB LED Blue

        for i in range(0, 4096, 8):
            pwm3.setPWM(0, 0, i)#Sets RGB LED Red
            pwm3.setPWM(1, 0, 4095-i)#Sets RGB LED Green

    def green_to_blue(self):
        pwm3.setPWM(0, 0, 0)#Sets RGB LED Red
        pwm3.setPWM(1, 0, 4095)#Sets RGB LED Green
        pwm3.setPWM(2, 0, 0)#Sets RGB LED Blue

        for i in range(0, 4096, 8):
            pwm3.setPWM(1, 0, 4095-i)#Sets RGB LED Green
            pwm3.setPWM(2, 0, i)#Sets RGB LED Blue

    def red_to_green(self):
        pwm3.setPWM(0, 0, 4095)#Sets RGB LED Red
        pwm3.setPWM(1, 0, 0)#Sets RGB LED Green
        pwm3.setPWM(2, 0, 0)#Sets RGB LED Blue

        for i in range(0, 4096, 8):
            pwm3.setPWM(0, 0, 4095-i)#Sets RGB LED Red
            pwm3.setPWM(1, 0, i)#Sets RGB LED Green

    def change_colour(self, red_start_value, green_start_value, blue_start_value, red_end_value, green_end_value, blue_end_value, speed):
        pwm3.setPWM(0, 0, red_start_value)#Sets RGB LED Red
        pwm3.setPWM(1, 0, green_start_value)#Sets RGB LED Green
        pwm3.setPWM(2, 0, blue_start_value)#Sets RGB LED Blue

        speed_variable = 4096 / speed

        red_step = (red_end_value - red_start_value) / speed_variable
        green_step = (green_end_value - green_start_value) / speed_variable
        blue_step = (blue_end_value - blue_start_value) / speed_variable


        for i in range(0, speed_variable, 1):
            pwm3.setPWM(0, 0, red_start_value + red_step * i)#Sets RGB LED Red
            pwm3.setPWM(1, 0, green_start_value + green_step * i)#Sets RGB LED Green
            pwm3.setPWM(2, 0, blue_start_value + blue_step * i)#Sets RGB LED Blue

#=================================================================================

Leg1 = Leg()#Create the object for leg one

#Correct the values for the leg
Leg1.X_Offset = 95.0#X offset for the leg
Leg1.Y_Offset = 45.5#Y offset for the leg
Leg1.Z_Offset = 0.0#Z offset for the leg

Leg1.X_base_position = 95.0#Base X coordinate for the end effector
Leg1.Y_base_position = 80.0#Base Y coordinate for the end effector
Leg1.Z_base_position = -80.0#Base Z coordinate for the end effector

Leg1.X_Coordinate = 95.0#Desired X coordinate for the end effector
Leg1.Y_Coordinate = 80.0#Desired Y coordinate for the end effector
Leg1.Z_Coordinate = -80.0#Desired Z coordinate for the end effector

#Side the leg is mounted on
Leg1.Leg_Side = "Left"

#Defining servo limits for the leg
Leg1.servoMin_S1 = 147#Servo 1 S1L1 minimum pulse count limit
Leg1.servoMax_S1 = 627#Servo 1 S1L1 maximum pulse count limit
Leg1.servoMin_S2 = 175#Servo 2 S2L1 minimum pulse count limit
Leg1.servoMax_S2 = 649#Servo 2 S2L1 maximum pulse count limit
Leg1.servoMin_S3 = 174#Servo 3 S3L1 minimum pulse count limit
Leg1.servoMax_S3 = 625#Servo 3 S3L1 maximum pulse count limit

#=================================================================================

Leg2 = Leg()#Create the object for leg one

#Correct the values for the leg
Leg2.X_Offset = 95.0#X offset for the leg
Leg2.Y_Offset = -45.5#Y offset for the leg
Leg2.Z_Offset = 0.0#Z offset for the leg

Leg2.X_base_position = 95.0#Base X coordinate for the end effector
Leg2.Y_base_position = -80.0#Base Y coordinate for the end effector
Leg2.Z_base_position = -80.0#Base Z coordinate for the end effector

Leg2.X_Coordinate = 95.0#Desired X coordinate for the end effector
Leg2.Y_Coordinate = -180.0#Desired Y coordinate for the end effector
Leg2.Z_Coordinate = -80.0#Desired Z coordinate for the end effector

#Side the leg is mounted on
Leg2.Leg_Side = "Right"

#Defining servo limits for the leg
Leg2.servoMin_S1 = 173#Servo 1 S1L1 minimum pulse count limit
Leg2.servoMax_S1 = 645#Servo 1 S1L1 maximum pulse count limit
Leg2.servoMin_S2 = 170#Servo 2 S2L1 minimum pulse count limit
Leg2.servoMax_S2 = 630#Servo 2 S2L1 maximum pulse count limit
Leg2.servoMin_S3 = 155#Servo 3 S3L1 minimum pulse count limit
Leg2.servoMax_S3 = 625#Servo 3 S3L1 maximum pulse count limit

#=================================================================================

Leg3 = Leg()#Create the object for leg three

#Correct the values for the leg
Leg3.X_Offset = 0.0#X offset for the leg
Leg3.Y_Offset = 45.5#Y offset for the leg
Leg3.Z_Offset = 0.0#Z offset for the leg

Leg3.X_base_position = 0.0#Base X coordinate for the end effector
Leg3.Y_base_position = 80.0#Base Y coordinate for the end effector
Leg3.Z_base_position = -80.0#Base Z coordinate for the end effector

Leg3.X_Coordinate = -20.0#Desired X coordinate for the end effector
Leg3.Y_Coordinate = 180.0#Desired Y coordinate for the end effector
Leg3.Z_Coordinate = -80.0#Desired Z coordinate for the end effector

#Side the leg is mounted on
Leg3.Leg_Side = "Left"

#Defining servo limits for the leg
Leg3.servoMin_S1 = 168#Servo 7 S1L3 minimum pulse count limit
Leg3.servoMax_S1 = 652#Servo 7 S1L3 maximum pulse count limit
Leg3.servoMin_S2 = 146#Servo 8 S2L3 minimum pulse count limit
Leg3.servoMax_S2 = 632#Servo 8 S2L3 maximum pulse count limit
Leg3.servoMin_S3 = 174#Servo 9 S3L3 minimum pulse count limit
Leg3.servoMax_S3 = 653#Servo 9 S3L3 maximum pulse count limit

#=================================================================================

Leg4 = Leg()#Create the object for leg one

#Correct the values for the leg
Leg4.X_Offset = 0.0#X offset for the leg
Leg4.Y_Offset = -45.5#Y offset for the leg
Leg4.Z_Offset = 0.0#Z offset for the leg

Leg4.X_base_position = 0.0#Base X coordinate for the end effector
Leg4.Y_base_position = -80.0#Base Y coordinate for the end effector
Leg4.Z_base_position = -80.0#Base Z coordinate for the end effector

Leg4.X_Coordinate = -20.0#Desired X coordinate for the end effector
Leg4.Y_Coordinate = -180.0#Desired Y coordinate for the end effector
Leg4.Z_Coordinate = -80.0#Desired Z coordinate for the end effector

#Side the leg is mounted on
Leg4.Leg_Side = "Right"

#Defining servo limits for the leg
Leg4.servoMin_S1 = 159#Servo 1 S1L1 minimum pulse count limit
Leg4.servoMax_S1 = 624#Servo 1 S1L1 maximum pulse count limit
Leg4.servoMin_S2 = 130#Servo 2 S2L1 minimum pulse count limit
Leg4.servoMax_S2 = 607#Servo 2 S2L1 maximum pulse count limit
Leg4.servoMin_S3 = 130#Servo 3 S3L1 minimum pulse count limit
Leg4.servoMax_S3 = 580#Servo 3 S3L1 maximum pulse count limit

#=================================================================================

Leg5 = Leg()#Create the object for leg five

#Correct the values for the leg
Leg5.X_Offset = -95.0#X offset for the leg
Leg5.Y_Offset = 45.5#Y offset for the leg
Leg5.Z_Offset = 0.0#Z offset for the leg

Leg5.X_base_position = -135.0#Base X coordinate for the end effector
Leg5.Y_base_position = 80.0#Base Y coordinate for the end effector
Leg5.Z_base_position = -80.0#Base Z coordinate for the end effector

Leg5.X_Coordinate = -135.0#Desired X coordinate for the end effector
Leg5.Y_Coordinate = 180.0#Desired Y coordinate for the end effector
Leg5.Z_Coordinate = -80.0#Desired Z coordinate for the end effector

#Side the leg is mounted on
Leg5.Leg_Side = "Left"

#Defining servo limits for the leg
Leg5.servoMin_S1 = 152#Servo 1 S1L1 minimum pulse count limit
Leg5.servoMax_S1 = 632#Servo 1 S1L1 maximum pulse count limit
Leg5.servoMin_S2 = 140#Servo 2 S2L1 minimum pulse count limit
Leg5.servoMax_S2 = 625#Servo 2 S2L1 maximum pulse count limit
Leg5.servoMin_S3 = 170#Servo 3 S3L1 minimum pulse count limit
Leg5.servoMax_S3 = 650#Servo 3 S3L1 maximum pulse count limit

#=================================================================================

Leg6 = Leg()#Create the object for leg one

#Correct the values for the leg
Leg6.X_Offset = -95.0#X offset for the leg
Leg6.Y_Offset = -45.5#Y offset for the leg
Leg6.Z_Offset = 0.0#Z offset for the leg

Leg6.X_base_position = -135.0#Base X coordinate for the end effector
Leg6.Y_base_position = -80.0#Base Y coordinate for the end effector
Leg6.Z_base_position = -80.0#Base Z coordinate for the end effector

Leg6.X_Coordinate = -135.0#Desired X coordinate for the end effector
Leg6.Y_Coordinate = -180.0#Desired Y coordinate for the end effector
Leg6.Z_Coordinate = -80.0#Desired Z coordinate for the end effector

#Side the leg is mounted on
Leg6.Leg_Side = "Right"

#Defining servo limits for the leg
Leg6.servoMin_S1 = 144#Servo 1 S1L1 minimum pulse count limit
Leg6.servoMax_S1 = 605#Servo 1 S1L1 maximum pulse count limit
Leg6.servoMin_S2 = 142#Servo 2 S2L1 minimum pulse count limit
Leg6.servoMax_S2 = 613#Servo 2 S2L1 maximum pulse count limit
Leg6.servoMin_S3 = 160#Servo 3 S3L1 minimum pulse count limit
Leg6.servoMax_S3 = 615#Servo 3 S3L1 maximum pulse count limit

#=================================================================================

Left_Claw = Claw()

#X, Y, Z Offset for the servo mounting position relitive to the defined origin
Left_Claw.X_Offset = 174.932#X coordinate offset
Left_Claw.Y_Offset = 88.8#Y coordinate offset
Left_Claw.Z_Offset = 9.3#Z coordinate offset

#Desired X, Y, Z coordinates for the end effector
Left_Claw.X_Coordinate = 270.0
Left_Claw.Y_Coordinate = 90.0
Left_Claw.Z_Coordinate = 30.0

#Joint angles
Left_Claw.Shoulder_XZ = 90.0
Left_Claw.Shoulder_XY = 45.0
Left_Claw.Elbow = 90.0
Left_Claw.Wrist = 20
Left_Claw.Pincer = 50

#Servo pulse counts
Left_Claw.servoMin_S1 = 139#up
Left_Claw.servoMax_S1 = 622#down
Left_Claw.servoMin_S2 = 148#in
Left_Claw.servoMax_S2 = 636#out
Left_Claw.servoMin_S3 = 152#extension
Left_Claw.servoMax_S3 = 642#in
Left_Claw.servoMin_S4 = 300#out
Left_Claw.servoMax_S4 = 574#in
Left_Claw.servoMin_S5 = 150#
Left_Claw.servoMax_S5 = 550#

Left_Claw.PincerOpen = 470
Left_Claw.PincerClosed = 310

#Side the claw is mounted on
Left_Claw.Claw_Side = "Left"

#=================================================================================

Right_Claw = Claw()

#X, Y, Z Offset for the servo mounting position relitive to the defined origin
Right_Claw.X_Offset = 174.932#X coordinate offset
Right_Claw.Y_Offset = -88.8#Y coordinate offset
Right_Claw.Z_Offset = 9.3#Z coordinate offset

#Desired X, Y, Z coordinates for the end effector
Right_Claw.X_Coordinate = 270.0
Right_Claw.Y_Coordinate = -90.0
Right_Claw.Z_Coordinate = 30.0

#Joint angles
Right_Claw.Shoulder_XZ = 0.0
Right_Claw.Shoulder_XY = 0.0
Right_Claw.Elbow = 0.0
Right_Claw.Wrist = 20#In degrees -15 to 90
Right_Claw.Pincer = 50

#Servo pulse counts
Right_Claw.servoMin_S1 = 141#points arm up
Right_Claw.servoMax_S1 = 630
Right_Claw.servoMin_S2 = 148#Arm in
Right_Claw.servoMax_S2 = 636#arm out
Right_Claw.servoMin_S3 = 150
Right_Claw.servoMax_S3 = 607#full extension
Right_Claw.servoMin_S4 = 197#curled in
Right_Claw.servoMax_S4 = 480#Max curved out
Right_Claw.servoMin_S5 = 145#Open claw
Right_Claw.servoMax_S5 = 290#closed

Right_Claw.PincerOpen = 145
Right_Claw.PincerClosed = 290

#Side the claw is mounted on
Right_Claw.Claw_Side = "Right"
#=================================================================================

def leg_Inverse_Kinamatics():
    #Legs on the Left side
    Leg1.inverse_kinematics()#Calculates the inverse kinematics for the leg
    Leg3.inverse_kinematics()#Calculates the inverse kinematics for the leg
    Leg5.inverse_kinematics()#Calculates the inverse kinematics for the leg

    #Legs on the Right side
    Leg2.inverse_kinematics()#Calculates the inverse kinematics for the leg
    Leg4.inverse_kinematics()#Calculates the inverse kinematics for the leg
    Leg6.inverse_kinematics()#Calculates the inverse kinematics for the leg

#=================================================================================

def moveTail():#Function to move all the tail servos into position
    pwm3.setPWM(15, 0, body.Tail_base_S29)#servo 29
    pwm3.setPWM(14, 0, body.Tail_base_S30)#servo 30
    pwm3.setPWM(13, 0, body.Tail_Segment_1)#servo 24
    pwm3.setPWM(12, 0, body.Tail_Segment_2)#servo 23
    pwm3.setPWM(11, 0, body.Tail_Segment_3)#servo 22
    pwm3.setPWM(10, 0, body.Tail_Segment_4)#servo 20
    pwm3.setPWM(9, 0, body.Tail_Segment_5)#servo 19

#=================================================================================

def moveLeftClaw():#Function to set the claw positions
    pwm1.setPWM(11, 0, Left_Claw.Shoulder_XZ)#Servo 1 Left Claw
    pwm1.setPWM(12, 0, Left_Claw.Shoulder_XY)#Servo 2 Left Claw
    pwm1.setPWM(13, 0, Left_Claw.Elbow)#Servo 3 Left Claw
    wristAngle = int(round(map_number(Left_Claw.Wrist, -15, 90, Left_Claw.servoMin_S4, Left_Claw.servoMax_S4)))
    #print "Left wrist = %d" % wristAngle
    pwm1.setPWM(14, 0, wristAngle)#Servo 4 Left Claw

#=================================================================================

def moveRightClaw():#Function to set the claw positions
    pwm2.setPWM(4, 0, Right_Claw.Shoulder_XZ)#Servo 1 Right Claw
    pwm2.setPWM(3, 0, Right_Claw.Shoulder_XY)#Servo 2 Right Claw
    pwm2.setPWM(2, 0, Right_Claw.Elbow)#Servo 3 Right Claw
    wristAngle = int(round(map_number(Right_Claw.Wrist, -15, 90, Right_Claw.servoMax_S4, Right_Claw.servoMin_S4)))
    #print "Right wrist = %d" % wristAngle
    pwm2.setPWM(1, 0, wristAngle)#Servo 4 Right Claw

#=================================================================================

def moveLegs():#Function to set the leg positions
    pwm1.setPWM(8, 0, Leg1.Tibia_Joint)#Servo 3 Leg 1
    pwm1.setPWM(9, 0, Leg1.Femur_Joint)#Servo 2 Leg 1
    pwm1.setPWM(10, 0, Leg1.Hip_Joint)#Servo 1 Leg 1

    pwm1.setPWM(5, 0, Leg3.Tibia_Joint)#Servo 3 Leg 3
    pwm1.setPWM(6, 0, Leg3.Femur_Joint)#Servo 2 Leg 3
    pwm1.setPWM(7, 0, Leg3.Hip_Joint)#Servo 1 Leg 3

    pwm1.setPWM(2, 0, Leg5.Tibia_Joint)#Servo 3 Leg 5
    pwm1.setPWM(3, 0, Leg5.Femur_Joint)#Servo 2 Leg 5
    pwm1.setPWM(4, 0, Leg5.Hip_Joint)#Servo 1 Leg 5

    pwm2.setPWM(5, 0, Leg2.Hip_Joint)#Servo 3 Leg 2
    pwm2.setPWM(6, 0, Leg2.Femur_Joint)#Servo 2 Leg 2
    pwm2.setPWM(7, 0, Leg2.Tibia_Joint)#Servo 1 Leg 2

    pwm2.setPWM(8, 0, Leg4.Hip_Joint)#Servo 3 Leg 4
    pwm2.setPWM(9, 0, Leg4.Femur_Joint)#Servo 2 Leg 4
    pwm2.setPWM(10, 0, Leg4.Tibia_Joint)#Servo 1 Leg 4

    pwm2.setPWM(11, 0, Leg6.Hip_Joint)#Servo 3 Leg 6
    pwm2.setPWM(12, 0, Leg6.Femur_Joint)#Servo 2 Leg 6
    pwm2.setPWM(13, 0, Leg6.Tibia_Joint)#Servo 1 Leg 6

#=================================================================================

def PS3_Eye_Colour():#Uses Circle, Triangle and x to control the RGB LEDs respectively

    red = 0
    green = 0
    blue = 0
    
    while PS3Controller.ps3.circle_Toggle == 1:
        PS3Controller.ps3.update_PS3_Controller_Values()#Read and updates values
        red = int(round(map_number(PS3Controller.ps3.axis0, -1, 1, 0, 255)))
        green = int(round(map_number(PS3Controller.ps3.axis1, -1, 1, 255, 0)))
        blue = int(round(map_number(PS3Controller.ps3.axis0, -1, 1, 255, 0)))

        red = int(round(map_number(red, 0, 255, 0, 255 - green)))
        blue = int(round(map_number(blue, 0, 255, 0, 255 - green)))
        
        pwm3.setPWM(0, 0, red)#Sets RGB LED Red
        pwm3.setPWM(1, 0, green)#Sets RGB LED Green
        pwm3.setPWM(2, 0, blue)#Sets RGB LED Blue

#=================================================================================
#R1_first_flag = 0
#R2_first_flag = 0
def PS3_Right_Pincer():
    open_change = map_number(PS3Controller.ps3.axis13, -1, 1, 0, 10)
    close_change = map_number(PS3Controller.ps3.axis15, -1, 1, 0, -10)
    
    if close_change < 0 and Right_Claw.Pincer + close_change > 0 and close_change != -5:
        Right_Claw.Pincer = Right_Claw.Pincer + close_change
        Right_Claw.pincer_position(Right_Claw.Pincer)
        
    elif open_change > 0 and Right_Claw.Pincer + open_change < 100 and open_change != 5:
        Right_Claw.Pincer = Right_Claw.Pincer + open_change
        Right_Claw.pincer_position(Right_Claw.Pincer)

#=================================================================================

def PS3_Left_Pincer():
    open_change = map_number(PS3Controller.ps3.axis12, -1, 1, 0, 10)
    close_change = map_number(PS3Controller.ps3.axis14, -1, 1, 0, -10)
    
    if close_change < 0 and Left_Claw.Pincer + close_change > 0 and close_change != -5:
        Left_Claw.Pincer = Left_Claw.Pincer + close_change
        Left_Claw.pincer_position(Left_Claw.Pincer)
        
    elif open_change > 0 and Left_Claw.Pincer + open_change < 100 and open_change != 5:
        Left_Claw.Pincer = Left_Claw.Pincer + open_change
        Left_Claw.pincer_position(Left_Claw.Pincer)
#=================================================================================

def PS3_Right_Claw():
    Last_X = Right_Claw.X_Coordinate
    Last_Y = Right_Claw.Y_Coordinate
    Last_Z = Right_Claw.Z_Coordinate

    if PS3Controller.ps3.R3_Toggle == 1:#if R3 has been pressed
        X_change = map_number(PS3Controller.ps3.axis1, -1, 1, 6, -6)
        Y_change = map_number(PS3Controller.ps3.axis0, -1, 1, 6, -6)
        Z_change = map_number(PS3Controller.ps3.axis3, -1, 1, 6, -6)

        Right_Claw.X_Coordinate = Right_Claw.X_Coordinate + X_change
        Right_Claw.Y_Coordinate = Right_Claw.Y_Coordinate + Y_change
        Right_Claw.Z_Coordinate = Right_Claw.Z_Coordinate + Z_change
        
        if PS3Controller.ps3.up_Toggle == 1 and Right_Claw.Wrist > -15:
            Right_Claw.Wrist = Right_Claw.Wrist - 2
        elif  PS3Controller.ps3.down_Toggle == 1 and Right_Claw.Wrist < 90:
            Right_Claw.Wrist = Right_Claw.Wrist + 2

        if math.sqrt((Right_Claw.X_Coordinate - Right_Claw.X_Offset) ** 2 + (Right_Claw.Z_Coordinate - Right_Claw.Z_Offset) ** 2) < 45 or math.sqrt((Right_Claw.X_Coordinate - Right_Claw.X_Offset) ** 2 + (Right_Claw.Y_Coordinate - Right_Claw.Y_Offset) ** 2 + (Right_Claw.Z_Coordinate - Right_Claw.Z_Offset) ** 2) > (Right_Claw.L1 + Right_Claw.L2) or Right_Claw.X_Coordinate < 0:
            Right_Claw.X_Coordinate = Last_X
            Right_Claw.Y_Coordinate = Last_Y
            Right_Claw.Z_Coordinate = Last_Z
        else:
            Right_Claw.inverse_kinematics()
            moveRightClaw()

#=================================================================================

def PS3_Left_Claw():
    Last_X = Left_Claw.X_Coordinate
    Last_Y = Left_Claw.Y_Coordinate
    Last_Z = Left_Claw.Z_Coordinate

    if PS3Controller.ps3.L3_Toggle == 1:
        X_change = map_number(PS3Controller.ps3.axis1, -1, 1, 6, -6)
        Y_change = map_number(PS3Controller.ps3.axis0, -1, 1, 6, -6)
        Z_change = map_number(PS3Controller.ps3.axis3, -1, 1, 6, -6)

        Left_Claw.X_Coordinate = Left_Claw.X_Coordinate + X_change
        Left_Claw.Y_Coordinate = Left_Claw.Y_Coordinate + Y_change
        Left_Claw.Z_Coordinate = Left_Claw.Z_Coordinate + Z_change

        if PS3Controller.ps3.up_Toggle == 1 and Left_Claw.Wrist > -15:
            Left_Claw.Wrist = Left_Claw.Wrist - 2
        elif  PS3Controller.ps3.down_Toggle == 1 and Left_Claw.Wrist < 90:
            Left_Claw.Wrist = Left_Claw.Wrist + 2

        if math.sqrt((Left_Claw.X_Coordinate - Left_Claw.X_Offset) ** 2 + (Left_Claw.Z_Coordinate - Left_Claw.Z_Offset) ** 2) < 45 or math.sqrt((Left_Claw.X_Coordinate - Left_Claw.X_Offset) ** 2 + (Left_Claw.Y_Coordinate - Left_Claw.Y_Offset) ** 2 + (Left_Claw.Z_Coordinate - Left_Claw.Z_Offset) ** 2) > (Left_Claw.L1 + Left_Claw.L2) or Left_Claw.X_Coordinate < 0:
            Left_Claw.X_Coordinate = Last_X
            Left_Claw.Y_Coordinate = Last_Y
            Left_Claw.Z_Coordinate = Last_Z
        else:
            Left_Claw.inverse_kinematics()
            moveLeftClaw()

#=================================================================================

def ICC_Parallel():
    direction = 0

    if PS3Controller.ps3.axis1 < PS3Controller.ps3.axis3:
        Gait.ICC[2] = "CW"
    elif PS3Controller.ps3.axis1 > PS3Controller.ps3.axis3:
        Gait.ICC[2] = "CWW"

    if PS3Controller.ps3.axis1 <= 0 and PS3Controller.ps3.axis3 <= 0:
        direction = 0
    elif PS3Controller.ps3.axis1 > 0 and PS3Controller.ps3.axis3 > 0:
        direction = 1

    if direction == 0 and PS3Controller.ps3.axis1 < PS3Controller.ps3.axis3:
        Gait.ICC[2] = "CW"
    elif direction == 0 and PS3Controller.ps3.axis1 >= PS3Controller.ps3.axis3:
        Gait.ICC[2] = "CCW"
    elif direction == 1 and PS3Controller.ps3.axis1 < PS3Controller.ps3.axis3:
        Gait.ICC[2] = "CW"
    elif direction == 1 and PS3Controller.ps3.axis1 >= PS3Controller.ps3.axis3:
        Gait.ICC[2] = "CCW"

    if Gait.ICC[2] == "CCW":
        Y = map_number((PS3Controller.ps3.axis1 + PS3Controller.ps3.axis3) / 2, -1, 1, 1000, -1000)
    elif Gait.ICC[2] == "CW":
        Y = map_number((PS3Controller.ps3.axis1 + PS3Controller.ps3.axis3) / 2, -1, 1, -1000, 1000)

    if Gait.ICC[2] == "CCW":
        X = map_number((PS3Controller.ps3.axis0 + PS3Controller.ps3.axis2) / 2, -1, 1, -1000, 1000)
    elif Gait.ICC[2] == "CW":
        X = map_number((PS3Controller.ps3.axis0 + PS3Controller.ps3.axis2) / 2, -1, 1, 1000, -1000)

    if PS3Controller.ps3.axis0 == 0 and PS3Controller.ps3.axis1 == 0 and PS3Controller.ps3.axis2 == 0 and PS3Controller.ps3.axis3 == 0:
        Gait.ICC[2] = "Stop"
    Gait.ICC[0] = int(round(X))
    Gait.ICC[1] = int(round(Y))

#=================================================================================

def PS3_Speed():
    maximum = 0
    if Gait.gaitType == Gait.gaitType == "ripple_123456" or Gait.gaitType == "ripple_654321" or Gait.gaitType == "ripple_135246":
        maximumSpeed = 40
    elif Gait.gaitType == "tripod":
        maximumSpeed = 25
    elif Gait.gaitType == "double_ripple":
        maximumSpeed = 30    
    else:
        maximumSpeed = 50
        
    axis0 = abs(PS3Controller.ps3.axis0)
    axis1 = abs(PS3Controller.ps3.axis1)
    axis2 = abs(PS3Controller.ps3.axis2)
    axis3 = abs(PS3Controller.ps3.axis3)

    if axis0 > maximum:
        maximum = axis0
    if axis1 > maximum:
        maximum = axis1
    if axis2 > maximum:
        maximum = axis2
    if axis3 > maximum:
        maximum = axis3
    
    speed = int(round(map_number(maximum, 0, 1, 100, maximumSpeed)))

    Leg1.step_number = int(round(map_number(Leg1.step_number, 0, Gait.number_of_steps, 0, speed)))
    Leg2.step_number = int(round(map_number(Leg2.step_number, 0, Gait.number_of_steps, 0, speed)))
    Leg3.step_number = int(round(map_number(Leg3.step_number, 0, Gait.number_of_steps, 0, speed)))
    Leg4.step_number = int(round(map_number(Leg4.step_number, 0, Gait.number_of_steps, 0, speed)))
    Leg5.step_number = int(round(map_number(Leg5.step_number, 0, Gait.number_of_steps, 0, speed)))
    Leg6.step_number = int(round(map_number(Leg6.step_number, 0, Gait.number_of_steps, 0, speed)))
    Gait.step_number = int(round(map_number(Gait.step_number, 0, Gait.number_of_steps, 0, speed)))

    Gait.number_of_steps = speed
    
    if Gait.gaitType == "tripod":
        Gait.tripod_Cycle_Steps(Gait.number_of_steps)
    elif Gait.gaitType == "double_ripple":
        Gait.double_ripple_Cycle_Steps(Gait.number_of_steps)
    elif Gait.gaitType == "ripple_123456" or Gait.gaitType == "ripple_654321" or Gait.gaitType == "ripple_135246":
        Gait.ripple_Cycle_Steps(Gait.number_of_steps)

#=================================================================================
    
def PS3_Height():   
    if PS3Controller.ps3.down_Toggle == 1 and body.height < -80:
        body.height = body.height + 2
        Leg1.Z_Coordinate = Leg1.Z_Coordinate + 2#Sets the base Z coordinate for the leg
        Leg2.Z_Coordinate = Leg2.Z_Coordinate + 2#Sets the base Z coordinate for the leg
        Leg3.Z_Coordinate = Leg3.Z_Coordinate + 2#Sets the base Z coordinate for the leg
        Leg4.Z_Coordinate = Leg4.Z_Coordinate + 2#Sets the base Z coordinate for the leg
        Leg5.Z_Coordinate = Leg5.Z_Coordinate + 2#Sets the base Z coordinate for the leg
        Leg6.Z_Coordinate = Leg6.Z_Coordinate+ 2#Sets the base Z coordinate for the leg
        body.set_height(body.height)
    elif PS3Controller.ps3.up_Toggle == 1 and body.height > -150:
        body.height = body.height - 2
        Leg1.Z_Coordinate = Leg1.Z_Coordinate - 2#Sets the base Z coordinate for the leg
        Leg2.Z_Coordinate = Leg2.Z_Coordinate - 2#Sets the base Z coordinate for the leg
        Leg3.Z_Coordinate = Leg3.Z_Coordinate - 2#Sets the base Z coordinate for the leg
        Leg4.Z_Coordinate = Leg4.Z_Coordinate - 2#Sets the base Z coordinate for the leg
        Leg5.Z_Coordinate = Leg5.Z_Coordinate - 2#Sets the base Z coordinate for the leg
        Leg6.Z_Coordinate = Leg6.Z_Coordinate - 2#Sets the base Z coordinate for the leg
        body.set_height(body.height)
    
    

#=================================================================================

def PS3_Leg_Position(legNum):#Move a leg individually
    
    Last_X = legNum.X_Coordinate
    Last_Y = legNum.Y_Coordinate
    Last_Z = legNum.Z_Coordinate

    if PS3Controller.ps3.axis1 < 0:
        legNum.X_Coordinate = legNum.X_Coordinate + 1
    elif PS3Controller.ps3.axis1 > 0:
        legNum.X_Coordinate = legNum.X_Coordinate - 1

    if PS3Controller.ps3.axis0 < 0:
        legNum.Y_Coordinate = legNum.Y_Coordinate + 1
    elif PS3Controller.ps3.axis0 > 0:
        legNum.Y_Coordinate = legNum.Y_Coordinate - 1

    if PS3Controller.ps3.axis3 < 0:
        legNum.Z_Coordinate = legNum.Z_Coordinate + 1
    elif PS3Controller.ps3.axis3 > 0:
        legNum.Z_Coordinate = legNum.Z_Coordinate - 1

    if 1==0:#leg is outside of its limits:
        legNum.X_Coordinate = Last_X
        legNum.Y_Coordinate = Last_Y
        legNum.Z_Coordinate = Last_Z
    else:
        legNum.inverse_kinematics()
        moveLegs()#Moves the servos in the legs

#=================================================================================

def PS3_translation():#Body translations
    while PS3Controller.ps3.start_Toggle == 1:
        PS3Controller.ps3.update_PS3_Controller_Values()#Read and updates the PS3 controller values

        X_change = map_number(PS3Controller.ps3.axis1, -1, 1, -5, 5)
        Y_change = map_number(PS3Controller.ps3.axis0, -1, 1, -5, 5)
        Z_change = map_number(PS3Controller.ps3.axis3, -1, 1, -5, 5)
        #body.X_Tail_Coordinate = body.X_Tail_Coordinate + X_change

        Leg1.X_Coordinate = Leg1.X_Coordinate + X_change
        Leg2.X_Coordinate = Leg2.X_Coordinate + X_change
        Leg3.X_Coordinate = Leg3.X_Coordinate + X_change
        Leg4.X_Coordinate = Leg4.X_Coordinate + X_change
        Leg5.X_Coordinate = Leg5.X_Coordinate + X_change
        Leg6.X_Coordinate = Leg6.X_Coordinate + X_change

        Leg1.Y_Coordinate = Leg1.Y_Coordinate + Y_change
        Leg2.Y_Coordinate = Leg2.Y_Coordinate + Y_change
        Leg3.Y_Coordinate = Leg3.Y_Coordinate + Y_change
        Leg4.Y_Coordinate = Leg4.Y_Coordinate + Y_change
        Leg5.Y_Coordinate = Leg5.Y_Coordinate + Y_change
        Leg6.Y_Coordinate = Leg6.Y_Coordinate + Y_change

        Leg1.Z_Coordinate = Leg1.Z_Coordinate + Z_change
        Leg2.Z_Coordinate = Leg2.Z_Coordinate + Z_change
        Leg3.Z_Coordinate = Leg3.Z_Coordinate + Z_change
        Leg4.Z_Coordinate = Leg4.Z_Coordinate + Z_change
        Leg5.Z_Coordinate = Leg5.Z_Coordinate + Z_change
        Leg6.Z_Coordinate = Leg6.Z_Coordinate + Z_change

        Leg1.inverse_kinematics()
        Leg2.inverse_kinematics()
        Leg3.inverse_kinematics()
        Leg4.inverse_kinematics()
        Leg5.inverse_kinematics()
        Leg6.inverse_kinematics()
        
        if Leg1.limit_Flag == 1 or Leg2.limit_Flag == 1 or Leg3.limit_Flag == 1 or Leg4.limit_Flag == 1 or Leg5.limit_Flag == 1 or Leg6.limit_Flag == 1:
            
            Leg1.X_Coordinate = Leg1.previous_X
            Leg1.Y_Coordinate = Leg1.previous_Y
            Leg1.Z_Coordinate = Leg1.previous_Z

            Leg2.X_Coordinate = Leg2.previous_X
            Leg2.Y_Coordinate = Leg2.previous_Y
            Leg2.Z_Coordinate = Leg2.previous_Z

            Leg3.X_Coordinate = Leg3.previous_X
            Leg3.Y_Coordinate = Leg3.previous_Y
            Leg3.Z_Coordinate = Leg3.previous_Z

            Leg4.X_Coordinate = Leg4.previous_X
            Leg4.Y_Coordinate = Leg4.previous_Y
            Leg4.Z_Coordinate = Leg4.previous_Z

            Leg5.X_Coordinate = Leg5.previous_X
            Leg5.Y_Coordinate = Leg5.previous_Y
            Leg5.Z_Coordinate = Leg5.previous_Z

            Leg6.X_Coordinate = Leg6.previous_X
            Leg6.Y_Coordinate = Leg6.previous_Y
            Leg6.Z_Coordinate = Leg6.previous_Z

            Leg1.inverse_kinematics()
            Leg2.inverse_kinematics()
            Leg3.inverse_kinematics()
            Leg4.inverse_kinematics()
            Leg5.inverse_kinematics()
            Leg6.inverse_kinematics()
            
        else:
            
            Leg1.previous_X = Leg1.X_Coordinate
            Leg1.previous_Y = Leg1.Y_Coordinate
            Leg1.previous_Z = Leg1.Z_Coordinate

            Leg2.previous_X = Leg2.X_Coordinate
            Leg2.previous_Y = Leg2.Y_Coordinate
            Leg2.previous_Z = Leg2.Z_Coordinate

            Leg3.previous_X = Leg3.X_Coordinate
            Leg3.previous_Y = Leg3.Y_Coordinate
            Leg3.previous_Z = Leg3.Z_Coordinate

            Leg4.previous_X = Leg4.X_Coordinate
            Leg4.previous_Y = Leg4.Y_Coordinate
            Leg4.previous_Z = Leg4.Z_Coordinate
            
            Leg5.previous_X = Leg5.X_Coordinate
            Leg5.previous_Y = Leg5.Y_Coordinate
            Leg5.previous_Z = Leg5.Z_Coordinate
            
            Leg6.previous_X = Leg6.X_Coordinate
            Leg6.previous_Y = Leg6.Y_Coordinate
            Leg6.previous_Z = Leg6.Z_Coordinate
            
        moveLegs()#Moves the servos in the legs
#=================================================================================

def PS3_camber():
    camber = body.camber
    if PS3Controller.ps3.left_Toggle == 1 and camber > 280:
        camber = camber - 2
        body.set_camber(camber)
        #print "Camber = %d" % camber
    elif PS3Controller.ps3.right_Toggle == 1 and camber < 400:
        camber = camber + 2
        body.set_camber(camber)
        #print "Camber = %d" % camber
    
#=================================================================================

def PS3_tail():
    #put limit variables in
    X_change = map_number(PS3Controller.ps3.axis1, -1, 1, 4, -4)#might want to int round to stop drift
    Z_change = map_number(PS3Controller.ps3.axis3, -1, 1, 4, -4)#might want to int round to stop drift
    body.X_Tail_Coordinate = body.X_Tail_Coordinate + X_change
    body.Z_Tail_Coordinate = body.Z_Tail_Coordinate + Z_change

    if PS3Controller.ps3.axis24 > 0.15:
        PS3Controller.ps3.axis24 = 0.15
    elif PS3Controller.ps3.axis24 < -0.15:
        PS3Controller.ps3.axis24 = -0.15

    step = map_number(PS3Controller.ps3.axis24, -0.15, 0.15, -0.05235987756, 0.05235987756)

    if step < 0 and body.Camera_Angle > (-1.570796327 + step):
        body.Camera_Angle = body.Camera_Angle + step
    if step > 0 and body.Camera_Angle < (1.570796327 + step):
        body.Camera_Angle = body.Camera_Angle + step
        
    if PS3Controller.ps3.down_Toggle == 1 and body.synch > -1.570796327 + 0.01:
        body.synch = body.synch - 0.05
    if PS3Controller.ps3.up_Toggle == 1 and body.synch < 0.7853981634 - 0.01:
        body.synch = body.synch + 0.05
    #put limits condisions in
    body.tail_inverse_kinematics()#calculates the inverse kinematics for the tail
    moveTail()#moves the servos in the tail
        
#=================================================================================

def PS3_roll():
    while PS3Controller.ps3.triangle_Toggle == 1:
        PS3Controller.ps3.update_PS3_Controller_Values()#Read and updates the PS3 controller values
        roll_angle_change = map_number(PS3Controller.ps3.axis0, -1, 1, 0.05, -0.05)
        body.roll = body.roll + roll_angle_change
        body.calculate_roll()
        leg_Inverse_Kinamatics()
        all_leg_limits()
        moveLegs()
    body.roll = 0

#=================================================================================

def PS3_pitch():
    while PS3Controller.ps3.triangle_Toggle == 1:
        PS3Controller.ps3.update_PS3_Controller_Values()#Read and updates the PS3 controller values
        pitch_angle_change = map_number(PS3Controller.ps3.axis3, -1, 1, -0.05, 0.05)
        body.pitch = body.pitch + pitch_angle_change
        body.calculate_pitch()
        leg_Inverse_Kinamatics()
        all_leg_limits()
        moveLegs()
    body.pitch = 0

#=================================================================================

def PS3_yaw():
    while PS3Controller.ps3.triangle_Toggle == 1:
        PS3Controller.ps3.update_PS3_Controller_Values()#Read and updates the PS3 controller values
        yaw_angle_change = map_number(PS3Controller.ps3.axis2, -1, 1, 0.05, -0.05)
        body.yaw = body.yaw + yaw_angle_change
        body.calculate_yaw()
        leg_Inverse_Kinamatics()
        all_leg_limits()
        moveLegs()
    body.yaw = 0

#=================================================================================

def PS3_RPY():
    while PS3Controller.ps3.triangle_Toggle == 1:
        PS3Controller.ps3.update_PS3_Controller_Values()#Read and updates the PS3 controller values
        
        roll_angle_change = map_number(PS3Controller.ps3.axis2, -1, 1, 0.05, -0.05)
        body.roll = body.roll + roll_angle_change
        
        pitch_angle_change = map_number(PS3Controller.ps3.axis3, -1, 1, -0.05, 0.05)
        body.pitch = body.pitch + pitch_angle_change
        
        yaw_angle_change = map_number(PS3Controller.ps3.axis0, -1, 1, 0.05, -0.05)
        body.yaw = body.yaw + yaw_angle_change
        
        body.calculate_RPY()
        leg_Inverse_Kinamatics()#calculates the inverse kinematics for all the legs
        all_leg_limits()#Checks all the legs are within the limits and sets them to the previous values if they are not
        moveLegs()#moves the legs
    body.roll = 0
    body.pitch = 0
    body.yaw = 0
#=================================================================================

def PS3_RPY_SIXAXIS():
    while PS3Controller.ps3.triangle_Toggle == 1:
        PS3Controller.ps3.update_PS3_Controller_Values()#Read and updates the PS3 controller values

        if PS3Controller.ps3.axis23 > 0.15:
            PS3Controller.ps3.axis23 = 0.15
        elif PS3Controller.ps3.axis23 < -0.15:
            PS3Controller.ps3.axis23 = -0.15

        if PS3Controller.ps3.axis24 > 0.15:
            PS3Controller.ps3.axis24 = 0.15
        elif PS3Controller.ps3.axis24 < -0.15:
            PS3Controller.ps3.axis24 = -0.15
            
        roll_angle_change = map_number(PS3Controller.ps3.axis23, -0.15, 0.15, -0.08, 0.08)
        pitch_angle_change = map_number(PS3Controller.ps3.axis24, -0.15, 0.15, -0.08, 0.08)
        yaw_angle_change = map_number(PS3Controller.ps3.axis26, -1, 1, -0.06, 0.06)
            
        body.roll = body.roll + roll_angle_change
        body.pitch = body.pitch + pitch_angle_change
        body.yaw = body.yaw + yaw_angle_change
        
        body.calculate_RPY()
        leg_Inverse_Kinamatics()#calculates the inverse kinematics for all the legs
        all_leg_limits()#Checks all the legs are within the limits and sets them to the previous values if they are not
        moveLegs()#moves the legs
    body.roll = 0
    body.pitch = 0
    body.yaw = 0
    
#=================================================================================
    
def all_leg_limits():
    if Leg1.limit_Flag == 1 or Leg2.limit_Flag == 1 or Leg3.limit_Flag == 1 or Leg4.limit_Flag == 1 or Leg5.limit_Flag == 1 or Leg6.limit_Flag == 1:     
        Leg1.X_Coordinate = Leg1.previous_X
        Leg1.Y_Coordinate = Leg1.previous_Y
        Leg1.Z_Coordinate = Leg1.previous_Z

        Leg2.X_Coordinate = Leg2.previous_X
        Leg2.Y_Coordinate = Leg2.previous_Y
        Leg2.Z_Coordinate = Leg2.previous_Z

        Leg3.X_Coordinate = Leg3.previous_X
        Leg3.Y_Coordinate = Leg3.previous_Y
        Leg3.Z_Coordinate = Leg3.previous_Z

        Leg4.X_Coordinate = Leg4.previous_X
        Leg4.Y_Coordinate = Leg4.previous_Y
        Leg4.Z_Coordinate = Leg4.previous_Z

        Leg5.X_Coordinate = Leg5.previous_X
        Leg5.Y_Coordinate = Leg5.previous_Y
        Leg5.Z_Coordinate = Leg5.previous_Z

        Leg6.X_Coordinate = Leg6.previous_X
        Leg6.Y_Coordinate = Leg6.previous_Y
        Leg6.Z_Coordinate = Leg6.previous_Z
        
        body.roll = body.previous_roll
        body.pitch = body.previous_pitch
        body.yaw = body.previous_yaw
    
        leg_Inverse_Kinamatics()
    else:      
        Leg1.previous_X = Leg1.X_Coordinate
        Leg1.previous_Y = Leg1.Y_Coordinate
        Leg1.previous_Z = Leg1.Z_Coordinate

        Leg2.previous_X = Leg2.X_Coordinate
        Leg2.previous_Y = Leg2.Y_Coordinate
        Leg2.previous_Z = Leg2.Z_Coordinate

        Leg3.previous_X = Leg3.X_Coordinate
        Leg3.previous_Y = Leg3.Y_Coordinate
        Leg3.previous_Z = Leg3.Z_Coordinate

        Leg4.previous_X = Leg4.X_Coordinate
        Leg4.previous_Y = Leg4.Y_Coordinate
        Leg4.previous_Z = Leg4.Z_Coordinate
            
        Leg5.previous_X = Leg5.X_Coordinate
        Leg5.previous_Y = Leg5.Y_Coordinate
        Leg5.previous_Z = Leg5.Z_Coordinate
            
        Leg6.previous_X = Leg6.X_Coordinate
        Leg6.previous_Y = Leg6.Y_Coordinate
        Leg6.previous_Z = Leg6.Z_Coordinate

        body.previous_roll = body.roll
        body.previous_pitch = body.pitch
        body.previous_yaw = body.yaw

#=================================================================================
    
Eye_colour = Eyes()#Creates the object for the eyes
Eye_colour.custom_eye_colour(255, 0, 0)#Sets the eyes to red
PS3Controller.ps3.controller_init()#Attempt to connect to the controller
Eye_colour.custom_eye_colour(0, 255, 0)#Sets eyes to green
piMusic.pygameInit()#Initialzes pygame for the audio

Gait = Gaits()#Sets the Gait object
Gait.stride_length = 45.0#Maximum length of each step taken
Gait.stride_height = 20.0#Maximum height of each step taken

body = Body()#Sets the body object
body.set_leg_spacing(95, -20, -135)#Sets the spacing between each pair of legs
body.set_camber(340)#Sets the leg width
body.set_height(-110)#Sets the body height
body.Camera_Angle = -0.3
body.X_Tail_Coordinate = -180
body.Z_Tail_Coordinate = 230
body.tail_inverse_kinematics()#calculates the inverse kinematics for the tail
moveTail()

Right_Claw.inverse_kinematics()#Calculates the inverse kinematics for the right claw
Left_Claw.inverse_kinematics()#Calculates the inverse kinematics for the left claw
moveLeftClaw()#Moves the servos in the left claw
moveRightClaw()#Moves the servo in the right claw

Left_Claw.pincer_position(50)#sets the pincer position on the left claw
Right_Claw.pincer_position(50)#sets the pincer position on the right claw

Gait.standUp()#Makes Taurus stand up from initially being on its belly with its legs up
Gait.tripod_legs_to_base_position(5)

piMusic.playAudio("/home/pi/Desktop/Taurus/Sounds/Drapion Cry.mp3", 0, 0.8)
Gait.change_gait(5, 30, "tripod")  
    
while 1:        
    if map_number(MCP3008_ADC.adcValue(1), 0, 1023, 0.0, 13.0) < 10.0 and not piMusic.pygame.mixer.music.get_busy():#Gives a low battery warning
        piMusic.playAudio("/home/pi/Desktop/Taurus/Sounds/Warning low battery.mp3", 0, 1)
        
    if PS3Controller.ps3.L3_Toggle == 0 and PS3Controller.ps3.R3_Toggle == 0 and PS3Controller.ps3.start_Toggle == 0 and PS3Controller.ps3.select_Toggle == 0 and PS3Controller.ps3.triangle_Toggle == 0:
        PS3_Speed()
        Gait.update_step_numbers()    
        PS3_Height()
        PS3_camber()
        Gait.calculate_leg_positions()
        leg_Inverse_Kinamatics()
        moveLegs()#Moves the leg servos
        PS3_Eye_Colour()
        
            #piMusic.playAudio("/home/pi/Desktop/Taurus/Sounds/Noot Noot.mp3", 0, 1)
        #if PS3Controller.ps3.triangle == 1:
            #PS3_roll()
            #piMusic.playAudio("/home/pi/Desktop/Taurus/Sounds/R2D2.mp3", 0, 0.8)
        #print "sl = %d" % Gait.stride_length
        #print "flag = %d" % Gait.one_gait_cycle_flag
        if PS3Controller.ps3.x == 1:
            piMusic.playAudio("/home/pi/Desktop/Taurus/Sounds/R2D2.mp3", 0, 0.85)#18 offset
        if PS3Controller.ps3.square == 1:
            piMusic.playAudio("/home/pi/Desktop/Taurus/Sounds/Dog Barking.mp3", 0, 0.85)
        if PS3Controller.ps3.ps == 1:
            if Gait.gaitType == "tripod":
                Gait.tripod_legs_to_base_position(5)
            elif Gait.gaitType == "double_ripple":
                Gait.double_ripple_legs_to_base_position(5)
            elif Gait.gaitType == "ripple_123456" or Gait.gaitType == "ripple_654321":
                Gait.reposition_legs_to_base(5)
        if PS3Controller.ps3.R1 == 1:
            Gait.change_gait(5, 30, "tripod")
        if PS3Controller.ps3.R2 == 1:
            Gait.change_gait(5, 30, "double_ripple")
        if PS3Controller.ps3.L1 == 1:
            Gait.change_gait(5, 30, "ripple_123456")
        if PS3Controller.ps3.L2 == 1:
            Gait.change_gait(5, 30, "ripple_654321")
##        if PS3Controller.ps3.L2 == 1:
##            Gait.change_gait(5, 30, "ripple_135246")
    elif PS3Controller.ps3.L3_Toggle == 0 and PS3Controller.ps3.R3_Toggle == 0 and PS3Controller.ps3.start_Toggle == 1 and PS3Controller.ps3.select_Toggle == 0 and PS3Controller.ps3.triangle_Toggle == 0:
        if Gait.gaitType == "tripod":
            Gait.tripod_legs_to_base_position(5)
        elif Gait.gaitType == "double_ripple":
            Gait.double_ripple_legs_to_base_position(5)
        elif Gait.gaitType == "ripple_123456" or Gait.gaitType == "ripple_654321":
            Gait.reposition_legs_to_base(5)
        PS3_translation()#Body translations
        body.leg_translation_to_base_position(12)
    elif (PS3Controller.ps3.L3_Toggle == 1 or PS3Controller.ps3.R3_Toggle == 1) and PS3Controller.ps3.start_Toggle == 0 and PS3Controller.ps3.select_Toggle == 0 and PS3Controller.ps3.triangle_Toggle == 0:
        PS3_Right_Claw()
        PS3_Left_Claw()
        PS3_Right_Pincer()
        PS3_Left_Pincer()
        if PS3Controller.ps3.square == 1:
            piMusic.playAudio("/home/pi/Desktop/Taurus/Sounds/Noot Noot.mp3", 0, 1)
        if PS3Controller.ps3.x == 1:
            piMusic.playAudio("/home/pi/Desktop/Taurus/Sounds/DO IT, JUST DO IT.mp3", 0, 0.75)
        if PS3Controller.ps3.circle == 1:
            piMusic.playAudio("/home/pi/Desktop/Taurus/Sounds/Dr Zoidberg Woop.mp3", 0, 0.9)
    elif PS3Controller.ps3.L3_Toggle == 0 and PS3Controller.ps3.R3_Toggle == 0 and PS3Controller.ps3.start_Toggle == 0 and PS3Controller.ps3.select_Toggle == 1 and PS3Controller.ps3.triangle_Toggle == 0:
        PS3_tail()
        if PS3Controller.ps3.square == 1:
            piMusic.playAudio("/home/pi/Desktop/Taurus/Sounds/Rick roll.mp3", 0, 0.85)
        if PS3Controller.ps3.x == 1:
            piMusic.playAudio("/home/pi/Desktop/Taurus/Sounds/AND HIS NAME IS JOHN CENA.mp3", 0, 0.85)
        if PS3Controller.ps3.circle == 1:
            piMusic.playAudio("/home/pi/Desktop/Taurus/Sounds/Sad Trombone.mp3", 0, 0.9)
    elif PS3Controller.ps3.L3_Toggle == 0 and PS3Controller.ps3.R3_Toggle == 0 and PS3Controller.ps3.start_Toggle == 0 and PS3Controller.ps3.select_Toggle == 0 and PS3Controller.ps3.triangle_Toggle == 1:
        if Gait.gaitType == "tripod":
            Gait.tripod_legs_to_base_position(5)
        elif Gait.gaitType == "double_ripple":
            Gait.double_ripple_legs_to_base_position(5)
        elif Gait.gaitType == "ripple_123456" or Gait.gaitType == "ripple_654321":
            Gait.reposition_legs_to_base(5)
        #PS3_RPY()
        PS3_RPY_SIXAXIS()
        body.linear_leg_to_base_position(15)
        
    PS3Controller.ps3.update_PS3_Controller_Values()#Read and updates the PS3 controller values
    moveTail()#Moves the tail servos
