from MotorControl import ThreadedMotorControl
import time
from Encoder import Encoder
from PID import PID
import threading

def main():
	kP = float(input("Enter the kP:  "))
	kI = float(input("Enter the kI:  "))
	kD = float(input("Enter the kD:  "))
	main.pid = PID(kP, kI, kD)
	main.pid.setSampleTime(0.01)

	main.motor = ThreadedMotorControl((27, 22, 17))
	main.enc = Encoder(5, 6)
	main.enc.getAndClearDelta()
	main.motor.start()
	main.encVel = 0

	def callback():
		while True:
			main.pid.update(main.encVel)
			print(main.pid.last_error)
			time.sleep(0.01)

	loopthread = threading.Thread(target = callback)
	loopthread.daemon = True
	loopthread.start()
	main.lastTime = time.time()
	main.currTime = main.lastTime

	def encCallback():
		while True:
			main.currTime = time.time()
			main.encVel = main.enc.getAndClearDelta()/(main.currTime-main.lastTime)
			main.lastTime = main.currTime
			time.sleep(0.001)

	encthread = threading.Thread(target = encCallback)
	encthread.daemon = True
	encthread.start()
	try:
		while True:
			main.pid.setpoint = int(input("Enter a new speed : 0 - 500:  "))
			main.motor.speed = main.pid.output
			print(main.pid.setpoint)
	except KeyboardInterrupt:
		main.motor.close()

if __name__ == '__main__':
	main()