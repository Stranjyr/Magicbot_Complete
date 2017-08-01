#!/usr/bin/env python
import curses
from MagicBotRosWrapper import MagicBotRosWrapper
import sys, select, termios, tty
import numpy as np
import time

#getKey method taken from teleop_twist_keyboard

#Binding Dictionaries
moveBindings = {
				'a':(-30, 30 ),
				'd':(30 , -30),
				'w':(30, 30),
				's':(-30, -30)
				}

#Non-blocking read from stdin
def getKey(timeout = 0.1):
	tty.setraw(sys.stdin.fileno())
	i, o, e = select.select([sys.stdin], [], [], timeout)
	for s in i:
		if s == sys.stdin:
			key = sys.stdin.read(1)
			#termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
			return key
	#termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return None

#Calculate a deltax and deltay given a distance and a heading
def calcMove(d, t):
	rect = d * np.exp( 1j * (t*np.pi/180.0 ))
	return [np.real([rect, 0])[0], np.imag([rect, 0])[0]]

def getPolVector(x, y):
    d = np.sqrt(x**2 + y**2)
    t = np.arctan2(y, x)
    return(d, t)
#Tries to drive home - does not recalculate after turning
def home(mb, speed = 20):
	try:
		mb.stop()
		lastAngle = mb.theta
		dist, theta = getPolVector(mb.x, mb.y)
		theta = theta*180.0/np.pi
		theta = (theta-180)%360
		thetaTurn = (theta - lastAngle)%360 - 180
		mb.moveBase(0, np.sign(thetaTurn)*speed)
		while (abs(lastAngle - mb.theta) > 10):
			lastAngle = mb.theta%360
			time.sleep(0.0001)
		mb.stop()
		mb.moveBase(speed, 0)
		ltime = time.time()
		timeout = 0
		while abs(mb.x) > 2 and abs(mb.y) > 2 and timeout < 5 :
			ctime = time.time()
			timeout+=ctime - ltime
			ltime = ctime
			time.sleep(0.001)
		mb.stop()
	except KeyboardInterrupt:
		mb.stop()
		#mb.close()


def main(myscreen):
	#curses setup
	myscreen.border(0)
	myscreen.addstr(5, 17, "WASD and Arrow Keys to Drive")
	myscreen.addstr(7, 30, "Type q to quit")
	myscreen.refresh()


	lspeed = 0
	rspeed = 0
	mb = MagicBotRosWrapper([(27, 22, 17), (23, 24, 18)], [(5, 6), (13, 19)])
	mb.start()

	timeout = 0
	keytime = time.time()
	#Telop Loop
	while True:
		mb.updateOdom()
		key = getKey()
		if key:
			timeout = 0
			keytime = time.time()
			if key in moveBindings.keys():
				lspeed = moveBindings[key][0]
				rspeed = moveBindings[key][1]
			elif key == 'h':
				myscreen.addstr(14, 20, 'Robot is returning to home              ')
				myscreen.refresh()
				home(mb)
				continue
			else:
				lspeed = 0
				rspeed = 0
				if key == 'q':
					break
			mb.move(lspeed, rspeed)
		else:
			timout = time.time() - keytime
			if timout > 5:
				mb.stop()
				keytime = time.time()
				timemout = 0

		#Add distance to totalDistance
		myscreen.addstr(10, 5, '                                     ')
		myscreen.addstr(10, 20, 'Motors are at {} {}'.format(lspeed, rspeed))
		myscreen.addstr(11, 20, 'The Heading is {:7.4f}'.format(mb.theta))
		myscreen.addstr(14, 20, 'Estimated Position is {:5.2f}, {:5.2f}'.format(mb.x, mb.y))
		myscreen.addstr(15, 20, 'Encoder totals are {:5d} {:5d}'.format(mb.leftTicks, mb.rightTicks))
		myscreen.addstr(15, 20, 'Encoder deltas are {:5d} {:5d}'.format(mb.dlTicks, mb.drTicks))
		myscreen.refresh()
	mb.stop()
	#mb.close()

if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)
	curses.wrapper(main)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
