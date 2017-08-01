#!/usr/bin/env python
from MotorControl import MotorControl
import time
import math
from BNO_Reader import BNO_Reader
from Encoder import Encoder
import numpy as np
from PID import PID
import threading

class MagicBot:
    '''
    init
    drivers : a list of items to setup the motor params. Each element should have a list of pins
        as pin A , pin B, and a pwm pin
    encs: a list of items to setup the encoders. Each element should have a list of pins as 
        pin A, pin B
    '''
    def __init__(self, drivers, encs, cal = 'calibration.json'):

        #Components
        self.leftDriver = MotorControl(drivers[0])
        self.rightDriver = MotorControl(drivers[1])
        self.leftEnc = Encoder(*encs[0])
        self.rightEnc = Encoder(*encs[1])
        self.bno = BNO_Reader(100, cal);
    

        #Constants
        self.ticksToMeter = (8*0.0254*np.pi)/(24.0*74.0)
        self.baseDist = 20*0.0254 #Meters

        #Variables
        #Position (Inch and deg)
        self.x = 0
        self.y = 0
        self.theta = 0

        #Change in Pos since last update
        self.vX = 0
        self.vL = 0
        self.vR = 0
        self.dX = 0
        self.dY = 0
        self.dTh = 0

        #Encoder Values (Ticks)
        self.leftTicks = 0
        self.rightTicks = 0

        #Change in encoder since last update
        self.dlTicks = 0
        self.drTicks = 0

        #Desired Speeds (For the PID controller)
        self.goalVL = 0
        self.goalVR = 0

        #PID Params and vars
        self.kPL = 100
        self.kPR = 100
        self.kDL = 0
        self.kDR = 0
        self.lastErrorL = 0
        self.lastErrorR = 0

        #Loop times (For calculating speeds)
        self.lastTime = time.time()
        self.currTime = self.lastTime
        self.dt = 0

        #Threading Stuff
        self.updateMux = threading.Condition()
        self.speedMux = threading.Condition()
        self.closeThread = False
        self.updateThread = None


    #Simple Motor Control
    def start(self):
        self.bno.start_bno_thread();
        self.updateThread = threading.Thread(target=self.updateLoop)
        self.updateThread.daemon = True
        self.updateThread.start()

    def move(self, leftSpeed, rightSpeed):
        self.leftDriver.setSpeed(leftSpeed)
        self.rightDriver.setSpeed(-rightSpeed)

    def stop(self):
        self.move(0, 0)

    def close(self):
        with self.updateMux:
            self.closeThread = True
        self.leftDriver.close()
        self.rightDriver.close()
        

    #Encoder Helper Functions
    '''
    deltaDist
    returns the distance in encoder ticks since last call of this function
    as [left_dist_in_inches, right_dist_in_inches]

    Side Effect : updates self.leftTicks and self.RightTicks to the new totals
    stored in the encoder
    '''
    def deltaDist(self):
        newEncL = self.leftEnc.getDelta()
        newEncR = self.rightEnc.getDelta();
        dL = newEncL - self.leftTicks
        dR = newEncR - self.rightTicks
        self.leftTicks = newEncL
        self.rightTicks = newEncR
        self.dlTicks = dL
        self.drTicks = dR
        return [dL*self.ticksToMeter, dR*self.ticksToMeter]
    

    '''
    deltaOdom
    returns a dx, dy, dtheta, dtime calculated from the wheel encoders
    if called with a list for deltas, uses those values for encoders instead of calling deltaDist
    side effect: resets the values of self.lastTime to the value of time.time when this is called
    '''
    def deltaOdom(self, deltas = None):
        dL, dR = self.deltaDist()
        self.currTime = time.time()
        dt = self.currTime-self.lastTime
        self.lastTime = self.currTime
        self.vL = dL/dt
        self.vR = dR/dt

        self.vX = (self.vL+self.vR)/2.0

        Th = self.bno.getReadings()[0][0]
        dTh = self.theta - Th
        self.theta = Th
        dX = self.vX * math.cos((self.theta + dTh/2.0)*math.pi/180.0) * dt
        dY = self.vX * math.sin((self.theta + dTh/2.0)*math.pi/180.0) * dt
        self.dX, self.dY, self.dTh, self.dt = dX, dY, dTh, dt
        return (dX, dY, dTh, dt)
    
    #updates the internal odom measurements using the delta values
    def updateOdom(self):
        self.deltaOdom()
        self.x += self.dX
        self.y += self.dY
        self.theta%=360

    #updates the left motor using the PID controller
    def updateSpeedL(self):
        currError = self.goalVL - self.vL 
        dError = currError - self.lastErrorL
        self.lastErrorL = currError
        return self.kPL*currError + self.kDL*(dError/self.dt)

    #updates the right motor using the PID controller
    def updateSpeedR(self):
        currError = self.goalVR - self.vR 
        dError = currError - self.lastErrorR
        self.lastErrorR = currError
        return self.kPR*currError + self.kDR*(dError/self.dt)

    #updates all motors with PID controls
    def updateSpeed(self):
        if self.dt > 0:
            with self.speedMux:
                l = self.updateSpeedL()
                r = self.updateSpeedR()
            self.move(l, r)

    #loop which updates odom and speed in turn
    def updateLoop(self):
        print 'Update Loop'
        while True:
            self.updateOdom()
            self.updateSpeed()
            time.sleep(0.01)
            with self.updateMux:
                if self.closeThread:
                    return;


    #Sets the PID setpoints such that we have forward and angular motion
    #Needs to have actual calculations instead of the placeholder values
    def moveBase(self, vX, vTh):
        with self.speedMux:
            self.goalVL = vX+vTh
            self.goalVR = vX-vTh

if __name__ == '__main__':
    mb = MagicBot([(27, 22, 17), (23, 24, 18)], [(5, 6), (13, 19)])
    mb.start()
    try:
        while True:
            print mb.theta
            time.sleep(.1)
    except KeyboardInterrupt:
        mb.close()
        print 'Closing'
