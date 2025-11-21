#!/usr/bin/env python
# encoding: utf-8

import os
import time
import getpass
import threading
from time import sleep

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, Bool
import pygame

def get_joystick_names():
    # Only initialize joystick subsystem to avoid audio errors in Docker
    os.environ['SDL_AUDIODRIVER'] = 'dummy'
    pygame.joystick.init()

    joystick_names = []
    joystick_count = pygame.joystick.get_count()
    
    if joystick_count == 0:
        print("no")
    else:
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            joystick_names.append(joystick.get_name())

    pygame.joystick.quit()
    return joystick_names
    
class JoyTeleop(Node):
	def __init__(self,name):
		super().__init__(name)
		self.Joy_active = True
		self.user_name = getpass.getuser()
		self.linear_Gear = 1
		self.angular_Gear = 1
		
		#create pub
		self.pub_cmdVel = self.create_publisher(Twist,'cmd_vel',  10)
		self.pub_JoyState = self.create_publisher(Bool,"JoyState",  10)
		
		#create sub
		self.sub_Joy = self.create_subscription(Joy,'joy', self.buttonCallback,10)
		
		#declare parameter and get the value
		self.declare_parameter('xspeed_limit',0.5)
		self.declare_parameter('yspeed_limit',0.5)
		self.declare_parameter('angular_speed_limit',2.0)  # Increased for faster turning
		self.declare_parameter('invert_controls', False)  # Set to True if motor controller boots in reverse
		self.xspeed_limit = self.get_parameter('xspeed_limit').get_parameter_value().double_value
		self.yspeed_limit = self.get_parameter('yspeed_limit').get_parameter_value().double_value
		self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value
		self.invert_controls = self.get_parameter('invert_controls').get_parameter_value().bool_value
		
		self.get_logger().info(f"Controls inverted: {self.invert_controls}")
		joysticks = get_joystick_names()
		self.joysticks = joysticks[0] if len(joysticks) != 0 else "no"
		
		print(f"[DEBUG] Detected joystick: '{self.joysticks}'")
		
		self.switch_dict = {
			"Xbox 360 Controller": [9,10,3],
			"Xbox One S Controller": [9,10,2],  # Axis 2 for right stick horizontal (turning)
			"SHANWAN Android Gamepad": [13,14,2],
		}
		
		if self.joysticks in self.switch_dict:
			print(f"[DEBUG] ✓ Button mapping found for '{self.joysticks}'")
		else:
			print(f"[DEBUG] ❌ NO button mapping for '{self.joysticks}'!")
			print(f"[DEBUG] Available mappings: {list(self.switch_dict.keys())}")
			print(f"[DEBUG] You need to add a mapping for this controller")

	def buttonCallback(self,joy_data):
		print(f"[DEBUG] buttonCallback triggered")
		print(f"[DEBUG] Joy data: {joy_data}")
		if not isinstance(joy_data, Joy):
			print(f"[DEBUG] ❌ Received non-Joy message type: {type(joy_data)}")
			return
		print(f"[DEBUG] User: {self.user_name}")
		if self.user_name == "root": self.user_jetson(joy_data)
		else: self.user_pc(joy_data)
    
	def user_jetson(self, joy_data):
			#linear Gear control
		if joy_data.buttons[self.switch_dict[self.joysticks][0]] == 1:
			if self.linear_Gear == 1.0: self.linear_Gear = 1.0 / 3
			elif self.linear_Gear == 1.0 / 3: self.linear_Gear = 2.0 / 3
			elif self.linear_Gear == 2.0 / 3: self.linear_Gear = 1
			# angular Gear control
		if joy_data.buttons[self.switch_dict[self.joysticks][1]] == 1:
			if self.angular_Gear == 1.0: self.angular_Gear = 1.0 / 4
			elif self.angular_Gear == 1.0 / 4: self.angular_Gear = 1.0 / 2
			elif self.angular_Gear == 1.0 / 2: self.angular_Gear = 3.0 / 4
			elif self.angular_Gear == 3.0 / 4: self.angular_Gear = 1.0
		
		# Debug: Show button presses to find B button
		#self.get_logger().info(f"Axes: {[f'{i}:{v:.2f}' for i, v in enumerate(joy_data.axes)]}")
		#if any(joy_data.buttons):
		#	self.get_logger().info(f"Buttons: {[f'{i}:{v}' for i, v in enumerate(joy_data.buttons) if v == 1]}")
		
		# Two-stick controls: left stick forward/back, right stick turning
		xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear  # Left stick up/down
		ylinear_speed = 0.0  # No strafe on this robot
		angular_speed = self.filter_data(joy_data.axes[2]) * self.angular_speed_limit * self.angular_Gear  # Right stick left/right
		
		# Apply inversion if needed (for motor controller bug)
		if self.invert_controls:
			xlinear_speed = -xlinear_speed
			ylinear_speed = -ylinear_speed
			angular_speed = -angular_speed
		
		if xlinear_speed > self.xspeed_limit: xlinear_speed = self.xspeed_limit
		elif xlinear_speed < -self.xspeed_limit: xlinear_speed = -self.xspeed_limit
		if ylinear_speed > self.yspeed_limit: ylinear_speed = self.yspeed_limit
		elif ylinear_speed < -self.yspeed_limit: ylinear_speed = -self.yspeed_limit
		if angular_speed > self.angular_speed_limit: angular_speed = self.angular_speed_limit
		elif angular_speed < -self.angular_speed_limit: angular_speed = -self.angular_speed_limit
		twist = Twist()
		twist.linear.x = xlinear_speed
		twist.linear.y = ylinear_speed
		twist.angular.z = angular_speed
		if self.Joy_active == True:
			print("joy control now")
			self.pub_cmdVel.publish(twist)
        
	def user_pc(self, joy_data):
			# Gear control
		if joy_data.buttons[self.switch_dict[self.joysticks][0]] == 1:
			if self.linear_Gear == 1.0: self.linear_Gear = 1.0 / 3
			elif self.linear_Gear == 1.0 / 3: self.linear_Gear = 2.0 / 3
			elif self.linear_Gear == 2.0 / 3: self.linear_Gear = 1
		if joy_data.buttons[self.switch_dict[self.joysticks][1]] == 1:
			if self.angular_Gear == 1.0: self.angular_Gear = 1.0 / 4
			elif self.angular_Gear == 1.0 / 4: self.angular_Gear = 1.0 / 2
			elif self.angular_Gear == 1.0 / 2: self.angular_Gear = 3.0 / 4
			elif self.angular_Gear == 3.0 / 4: self.angular_Gear = 1.0
		xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
		ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit * self.linear_Gear
		angular_speed = self.filter_data(joy_data.axes[self.switch_dict[self.joysticks][2]]) * self.angular_speed_limit * self.angular_Gear
		if xlinear_speed > self.xspeed_limit: xlinear_speed = self.xspeed_limit
		elif xlinear_speed < -self.xspeed_limit: xlinear_speed = -self.xspeed_limit
		if ylinear_speed > self.yspeed_limit: ylinear_speed = self.yspeed_limit
		elif ylinear_speed < -self.yspeed_limit: ylinear_speed = -self.yspeed_limit
		if angular_speed > self.angular_speed_limit: angular_speed = self.angular_speed_limit
		elif angular_speed < -self.angular_speed_limit: angular_speed = -self.angular_speed_limit
		twist = Twist()
		twist.linear.x = xlinear_speed
		twist.linear.y = ylinear_speed
		twist.angular.z = angular_speed
		self.pub_cmdVel.publish(twist)
        
	def filter_data(self, value):
		if abs(value) < 0.2: value = 0
		return value		
			
def main():
	rclpy.init()
	joy_ctrl = JoyTeleop('joy_ctrl')
	rclpy.spin(joy_ctrl)	
	
main()		