#!/usr/bin/env python
from MotorControl import ThreadedMotorControl
import time

class MagicBotMotorFrame:
	def __init__(self, drivers):
		self.leftDriver = ThreadedMotorControl(drivers[0])
		self.rightDriver = ThreadedMotorControl(drivers[1])

	#Simple Motor Control
	def start(self):
		self.leftDriver.start()
		self.rightDriver.start()

	def move(self, leftSpeed, rightSpeed):
		self.leftDriver.speed = leftSpeed
		self.rightDriver.speed = -rightSpeed

	def close(self):
		self.leftDriver.close()
		self.rightDriver.close()

	def stop(self):
		self.move(0, 0)




