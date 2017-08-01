import RPi.GPIO as gpio

class Encoder:
	#An encoder control library
	#Uses software interupts to count the encoder changes
	def __init__(self, aPin, bPin):
		self.deltaLookup = [0, 1, 0, -1]
		self.aPin = aPin
		self.bPin = bPin
		gpio.setwarnings(False)
		gpio.setmode (gpio.BCM)
		#outputs
		gpio.setup(self.aPin, gpio.IN, pull_up_down=gpio.PUD_UP)
		gpio.setup(self.bPin, gpio.IN, pull_up_down=gpio.PUD_UP)
		gpio.add_event_detect(self.aPin, gpio.BOTH, callback=self.encodeCount)
		gpio.add_event_detect(self.bPin, gpio.BOTH, callback=self.encodeCount)
		self.delta = 0
		self.last = (gpio.input(self.aPin) ^ gpio.input(self.bPin)) | gpio.input(self.bPin) << 1
	#callback for setting the new encoder position
	def encodeCount(self, channel):
		curr = (gpio.input(self.aPin) ^ gpio.input(self.bPin)) | gpio.input(self.bPin) << 1
		self.delta+=self.deltaLookup[(curr - self.last) % 4]
		self.last = curr
	#returns the encoder position in steps and clears the position
	#for fast turning motors where we are worried about overflow
	#or where position is tracked in the controller
	def getAndClearDelta(self):
		a = self.delta
		self.delta = 0
		return a
	#returns the position in steps
	def getDelta(self):
		return self.delta