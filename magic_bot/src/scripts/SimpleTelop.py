#!/usr/bin/env python
import curses
from MagicBotMotorFrame import MagicBotMotorFrame
import sys, select, termios, tty
from BNO_Reader import *
from Encoder import *
import math

'''getKey method taken from teleop_twist_keyboard
'''

moveBindings = {
				'a':(50, -50 ),
				'd':(-50 , 50),
				'w':(50, 50),
				's':(-50, -50)
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



def main(myscreen):
	#curses setup
	myscreen.border(0)
	myscreen.addstr(5, 17, "WASD and Arrow Keys to Drive")
	myscreen.addstr(7, 30, "Type q to quit")
	myscreen.refresh()
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
	encodeToInchConst = (8*math.pi)/(24.0*74.0)
	while True:
		key = getKey()
		if key:
			if key in moveBindings.keys():
				lspeed = moveBindings[key][0]
				rspeed = moveBindings[key][1]
			else:
				lspeed = 0
				rspeed = 0
				if key == 'q':
					break
			mb.move(lspeed, rspeed)
		#Add distance to totalDistance
		totalMove+=enc.getAndClearDelta()*encodeToInchConst
		myscreen.addstr(10, 5, '                                     ')
		myscreen.addstr(10, 20, 'Motors are at {} {}'.format(lspeed, rspeed))
		myscreen.addstr(11, 20, 'The Heading is {:7.4f}'.format(bno.getReadings()[0][0]))
		myscreen.addstr(12, 20, 'Distance Traveled is {:7.4f} inches'.format(totalMove))
		myscreen.refresh()
	mb.close()

if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)
	curses.wrapper(main)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
