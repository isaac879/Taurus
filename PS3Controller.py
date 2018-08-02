import pygame
import time
import numbers
import os
import sys
from pygame.locals import *
#=================================================================================

#Maps a number from one range to another
def map_number(input_variable, min_input, max_input, min_output, max_output):
    return (input_variable - min_input) * (max_output - min_output)/(max_input - min_input) + min_output

#=================================================================================


class PS3:
	def controller_init(self):
		global joystick
		connect_controller_flag = 0#Flag to check if the controller is connected

		while connect_controller_flag == 0:#keep trying to initialize until a controller is connected
			print "Initializing pygame"
			pygame.init()
			print "Initializing pygame.joystick"
			pygame.joystick.init()
			print "joystick.init()"
			try:
				print "Trying to create joystick object"
				joystick = pygame.joystick.Joystick(0)
				print "Trying Initialization"
				joystick.init()
				print "Connected"
				connect_controller_flag = 1
				
			except:
				print "Not connected"
				pygame.joystick.quit()	
				pass
			time.sleep(0.2)
#Variables for the Analog sticks
	axis0 = 0.0
	axis1 = 0.0
	axis2 = 0.0
	axis3 = 0.0
	axis12 = -1.0#L2
	axis13 = -1.0#R2
	axis14 = -1.0#L1
	axis15 = -1.0#R1
	axis23 = 0.0#Roll Y accelerometer
	axis24 = 0.0#Pitch 1 X accelerometer
	axis25 = 0.0#pitch 2 Z accelerometer
        axis26 = 0.0#yaw gyro
        
	select = 0
	L3 = 0
	R3 = 0
	start = 0
	up = 0
	right = 0
	down = 0
	left = 0
	L2 = 0
	R2 = 0
	L1 = 0
	R1 = 0
	triangle = 0
	circle = 0
	x = 0
	square = 0
	ps = 0
	
	R3_Toggle = 0
	L3_Toggle = 0
	up_Toggle = 0
	down_Toggle = 0
	left_Toggle = 0
	right_Toggle = 0
	square_Toggle = 0
	circle_Toggle = 0
	start_Toggle = 0
	select_Toggle = 0
	triangle_Toggle = 0
	
	def update_PS3_Controller_Values(self):		
		self.select = 0
		self.L3 = 0
		self.R3 = 0
		self.start = 0
		self.up = 0
		self.right = 0
		self.down = 0
		self.left = 0
		self.triangle = 0
		self.circle = 0
		self.x = 0
		self.square = 0
		self.ps = 0
		
		for event in pygame.event.get():
			if event.type == pygame.JOYAXISMOTION:
				self.axis0 = joystick.get_axis(0)
				self.axis1 = joystick.get_axis(1)
				self.axis2 = joystick.get_axis(2)
				self.axis3 = joystick.get_axis(3)
				self.axis12 = joystick.get_axis(12)#L2
                                self.axis13 = joystick.get_axis(13)#R2
                                self.axis14 = joystick.get_axis(14)#L1
                                self.axis15 = joystick.get_axis(15)#R1
                                self.axis23 = joystick.get_axis(23)#Roll Y accelerometer
				self.axis24 = joystick.get_axis(24)#Pitch 1 X accelerometer
				self.axis25 = joystick.get_axis(25)#Pitch 2 Z accelerometer
				self.axis26 = joystick.get_axis(26)#yaw
			elif event.type == pygame.JOYBUTTONDOWN:
				if event.button == 0:
					self.select = 1
					self.select_Toggle ^= 1
				if event.button == 1:
					self.L3 = 1
					self.L3_Toggle ^= 1
				if event.button == 2:
					self.R3 = 1
					self.R3_Toggle ^= 1
				if event.button == 3:
					self.start = 1
					self.start_Toggle ^= 1
				if event.button == 4:
					self.up = 1
					self.up_Toggle ^= 1
				if event.button == 5:
					self.right = 1
					self.right_Toggle ^= 1
				if event.button == 6:
					self.down = 1
					self.down_Toggle ^= 1
				if event.button == 7:
					self.left = 1
					self.left_Toggle ^= 1
				if event.button == 8:
					self.L2 = 1
				if event.button == 9:
					self.R2 = 1
				if event.button == 10:
					self.L1 = 1
				if event.button == 11:
					self.R1 = 1
				if event.button == 12:
					self.triangle = 1
					self.triangle_Toggle ^= 1
				if event.button == 13:
					self.circle = 1
					self.circle_Toggle ^= 1
				if event.button == 14:
					self.x = 1
				if event.button == 15:
					self.square = 1
					self.square_Toggle ^= 1
				if event.button == 16:
					self.ps = 1          
			elif event.type == pygame.JOYBUTTONUP:
				if event.button == 0:
					select = 0
				if event.button == 1:
					self.L3 = 0
				if event.button == 2:
					self.R3 = 0
				if event.button == 3:
					self.start = 0
				if event.button == 4:
					self.up = 0
					self.up_Toggle ^= 1
				if event.button == 5:
					self.right = 0
					self.right_Toggle ^= 1
				if event.button == 6:
					self.down = 0
					self.down_Toggle ^= 1
				if event.button == 7:
					self.left = 0
					self.left_Toggle ^= 1
				if event.button == 8:
					self.L2 = 0
				if event.button == 9:
					self.R2 = 0
				if event.button == 10:
					self.L1 = 0
				if event.button == 11:
					self.R1 = 0
				if event.button == 12:
					self.triangle = 0
				if event.button == 13:
					self.circle = 0
					self.circle_Toggle ^= 1
				if event.button == 14:
					self.x = 0
				if event.button == 15:
					self.square = 0
				if event.button == 16:
					self.ps = 0  
ps3 = PS3()#Sets the object
