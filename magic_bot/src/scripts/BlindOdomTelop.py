#!/usr/bin/env python
import curses
from MagicBotMotorFrame import MagicBotMotorFrame
import sys, select, termios, tty
from BNO_Reader import *
from Encoder import *
from collections import defaultdict
import numpy as np

#getKey method taken from teleop_twist_keyboard

#Binding Dictionaries
moveBindings = {
				'a':(50, -50 ),
				'd':(-50 , 50),
				'w':(50, 50),
				's':(-50, -50)
				}
sb = {
	 'a':"TURN",
	 'd':"TURN",
	 'w':"STRAIGHT",
	 's':"STRAIGHT"
	 }
stateBindings = defaultdict(lambda:"STOP", sb)

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
def home(mb, bno, enc, loc, speed = 20):
	try:
		encodeToInchConst = (8*np.pi)/(24.0*74.0)
		mb.stop()
		lastAngle = bno.getReadings()[0][0]%360
		saveAngle = lastAngle
		dist, theta = getPolVector(loc[0], loc[1])
		theta = theta*180.0/np.pi
		theta = (theta-180)%360
		thetaTurn = (theta - lastAngle)%360 - 180
		mb.move(-1*np.sign(thetaTurn)*speed, np.sign(thetaTurn)*speed)
		while (abs(lastAngle - theta) > 2):
			lastAngle = bno.getReadings()[0][0]%360
			time.sleep(0.001)
		mb.stop()
		enc.getAndClearDelta()
		accumDist = 0
		mb.move(speed, speed)
		while accumDist < dist:
			accumDist+= enc.getAndClearDelta()*encodeToInchConst
			time.sleep(0.001)
		mb.stop()
		return (accumDist, lastAngle)

	except KeyboardInterrupt:
		mb.stop()
		mb.close()
		return(0, 0)


def main(myscreen):
	#curses setup
	myscreen.border(0)
	myscreen.addstr(5, 17, "WASD and Arrow Keys to Drive")
	myscreen.addstr(7, 30, "Type q to quit")
	myscreen.refresh()
	boost = 1
	defaultSpeed = 20
	lspeed = 0
	rspeed = 0
	mb = MagicBotMotorFrame([(27, 22, 17), (23, 24, 18)])
	mb.start()

	#Init BNO
	bno = BNO_Reader(100);
	bno.start_bno_thread();

	#Init Encoder
	enc = Encoder(5, 6)
	totalMove = enc.getAndClearDelta()
	encodeToInchConst = (8*np.pi)/(24.0*74.0)

	#Robot Vars
	robot_state = "STOP"
	robot_x = 0
	robot_y = 0
	while True:
		key = getKey()
		if key:
			if key in moveBindings.keys():
				lspeed = moveBindings[key][0]
				rspeed = moveBindings[key][1]
			elif key == 'h':
				myscreen.addstr(14, 20, 'Robot is returning to home')
				myscreen.refresh()
				d, t = home(mb, bno, enc, [robot_x, robot_y], 20)
				x, y = calcMove(d, t)
				robot_x+=x
				robot_y+=y
				totalMove+=d
				continue
			else:
				lspeed = 0
				rspeed = 0
				if key == 'q':
					break
			mb.move(lspeed, rspeed)
			robot_state = stateBindings[key]
		#Add distance to totalDistance
		delta=enc.getAndClearDelta()*encodeToInchConst
		heading=bno.getReadings()[0][0]
		if robot_state == "STRAIGHT":
			totalMove+=delta
			dx, dy = calcMove(delta, heading)
			robot_x+=dx
			robot_y+=dy
		myscreen.addstr(10, 5, '                                     ')
		myscreen.addstr(10, 20, 'Motors are at {} {}'.format(lspeed, rspeed))
		myscreen.addstr(11, 20, 'The Heading is {:7.4f}'.format(heading))
		myscreen.addstr(12, 20, 'Distance Traveled is {:7.4f} inches'.format(totalMove))
		myscreen.addstr(13, 20, 'Robot State is {:10}'.format(robot_state))
		myscreen.addstr(14, 20, 'Estimated Position is {}, {}'.format(robot_x, robot_y))
		myscreen.refresh()
	mb.stop()
	mb.close()

if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)
	curses.wrapper(main)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
