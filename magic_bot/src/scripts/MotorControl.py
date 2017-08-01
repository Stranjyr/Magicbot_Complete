#!/usr/bin/env python
from H_Bridge import H_Bridge
#from scipy import signal
import time
import RPi.GPIO as gpio
import threading
import math
'''
A threaded Wrapper for the H_Bridge class
updating the self.speed variable to a value between -100 and 100
will set the speed and direction of the motor
'''

class MotorControl:
	def __init__(self, driver):
		self.driver = H_Bridge(driver[0], driver[1], driver[2])
		self.__currentSpeed = 0
	def setSpeed(self, speed):
		self.__currentSpeed = min(max(-100, speed), 100)
		self.driver.runMotor(self.__currentSpeed)
	def close(self):
		self.driver.close()

class ThreadedMotorControl(MotorControl):
	'''
	Constructor
	Inputs : 
		driver : Either a H_Bridge driver object, 
				or a list defining the A, B, and PWM pins for a new H_Bridge
		delay : Float object representing the sleep time in each update loop
	Use : Instatiate a new ThreadedMotorControl object
	'''
	def __init__(self, driver, delay = .1):
		MotorControl.__init__(self, driver)
		
		#thread stuff
		self.main_thread = None
		self.stopThread = False
		self.speed = 0
		self.__currentSpeed = 0
		self.delay = delay

	''' 
	start
	inputs: 
		NONE
	Use : Start the motor update thread
	'''
	def start(self):
		self.main_thread = threading.Thread(target=self.run_thread)
		self.main_thread.daemon = True
		self.main_thread.start()

	'''
	run_thread
	inputs :
		NONE
	Use : Loop until self.stopThread == True, updating the motor speed each loop
	'''
	def run_thread(self):
		while self.stopThread == False:
			if self.__currentSpeed != self.speed:
				self.driver.runMotor(self.speed)
				self.setSpeed(self.speed)
			time.sleep(self.delay)
		self.driver.runMotor(0); #stop the motor when the loop exits
	'''
	close
	inputs: 
		NONE
	Use : Shutdown the update thread, and close the H_Bridge pins
	'''
	def close(self):
		self.stopThread = True
		self.driver.close()




if __name__ == '__main__':
	drive = (27, 22, 17)
	control = ThreadedMotorControl(drive)
	control.start()
	
	try:
		while True:
			control.speed = float(input("Enter a speed between -100 and 100:  "))
	except KeyboardInterrupt:
		control.close()
		print("Shutting Down")

