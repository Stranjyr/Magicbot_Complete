from MagicBotMotorFrame import MagicBotMotorFrame
import sys, select, termios, tty
from BNO_Reader import *
from Encoder import *
from collections import defaultdict
import numpy as np
import time

def main():
	mb = MagicBotMotorFrame([(27, 22, 17), (23, 24, 18)])
	mb.start()
	try:
		enc = Encoder(5, 6)
		totalMove = 0
		encodeToInchConst = (8*np.pi)/(24.0*74.0)

		dist = int(input("Enter distance to travel there and back::  "))
		mb.move(20, 20)
		while(totalMove < dist):	
			totalMove+=enc.getAndClearDelta()*encodeToInchConst
			time.sleep(.01)
		mb.stop()
		totalMove+=enc.getAndClearDelta()*encodeToInchConst
		print("Moved {:7.4f} inches".format(totalMove))
		time.sleep(.1)
		#input("type and key and press enter to continue")
		mb.move(-20, -20)
		while(totalMove > 0):
			totalMove+=enc.getAndClearDelta()*encodeToInchConst
			time.sleep(.01)
		mb.stop()
		mb.close()
		totalMove+=enc.getAndClearDelta()*encodeToInchConst
		print("Currenly at {:7.4f}".format(totalMove))

	except KeyboardInterrupt:
		mb.stop()
		mb.close()
		print(totalMove)
		print("Closed due to keyboard interrupt")

if __name__ == '__main__':
	main()



