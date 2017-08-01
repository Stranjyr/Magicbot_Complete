import threading
import time
from Adafruit_BNO055 import BNO055
import os.path
import json

class BNO_Reader:
    def __init__(self, freq = 100, cal = 'calibration.json'):
        # How often to update the BNO sensor data (in hertz).
        self.BNO_UPDATE_FREQUENCY_HZ = freq

        self.CALIBRATION_FILE = cal
        # Create and configure the BNO sensor connection.  Make sure only ONE of the
        # below 'bno = ...' lines is uncommented:
        # Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
        #bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
        # BeagleBone Black configuration with default I2C connection (SCL=P9_19, SDA=P9_20),
        # and RST connected to pin P9_12:
        self.bno = BNO055.BNO055()
        self.BNO_AXIS_REMAP = { 'x': BNO055.AXIS_REMAP_X,
                   'y': BNO055.AXIS_REMAP_Z,
                   'z': BNO055.AXIS_REMAP_Y,
                   'x_sign': BNO055.AXIS_REMAP_POSITIVE,
                   'y_sign': BNO055.AXIS_REMAP_POSITIVE,
                   'z_sign': BNO055.AXIS_REMAP_NEGATIVE }
        self.bno_data = {}
        self.bno_changed = threading.Condition()
        self.bno_thread = None
        self.calibrate = False

        


    def read_bno(self):
        """Function to read the BNO sensor and update the bno_data object with the
        latest BNO orientation, etc. state.  Must be run in its own thread because
        it will never return
        """
        while True:
            # Grab new BNO sensor readings.
            temp = 0#self.bno.read_temp()
            heading, roll, pitch = self.bno.read_euler()
            x, y, z, w = self.bno.read_quaternion()
            sys, gyro, accel, mag = self.bno.get_calibration_status()
            status, self_test, error = self.bno.get_system_status(run_self_test=False)
            if status == 0x01:
                print('Error! Value: {0}'.format(error))
            # Capture the lock on the bno_changed condition so the bno_data shared
            # state can be updated.
            with self.bno_changed:
                self.bno_data['euler'] = (heading, roll, pitch)
                self.bno_data['temp'] = temp
                self.bno_data['quaternion'] = (x, y, z, w)
                self.bno_data['calibration'] = (sys, gyro, accel, mag)
                # Notify any waiting threads that the BNO state has been updated.
                self.bno_changed.notifyAll()
            # Sleep until the next reading.
            time.sleep(1.0/self.BNO_UPDATE_FREQUENCY_HZ)


    def start_bno_thread(self):
        # Initialize BNO055 sensor.
        if not self.bno.begin():
            raise RuntimeError('Failed to initialize BNO055!')
        self.bno.set_axis_remap(**self.BNO_AXIS_REMAP)
        # Kick off BNO055 reading thread.
        self.bno_thread = threading.Thread(target=self.read_bno)
        self.bno_thread.daemon = True  # Don't let the BNO reading thread block exiting.
        self.bno_thread.start()

        if os.path.isfile(self.CALIBRATION_FILE):
            self.calibrate = True
            with open(self.CALIBRATION_FILE, 'r') as cal_file:
                data = json.load(cal_file)
            # Grab the lock on BNO sensor access to serial access to the sensor.
            with self.bno_changed:
                self.bno.set_calibration(data)
                print "Calibrate"
        else:
            print 'No Calibrate'

    def getReadings(self):
        with self.bno_changed:
            self.bno_changed.wait()
            # A new reading is available!  Grab the reading value and then give
            # up the lock.
            heading, roll, pitch = self.bno_data['euler']
            temp = self.bno_data['temp']
            x, y, z, w = self.bno_data['quaternion']
            sys, gyro, accel, mag = self.bno_data['calibration']
            return (heading, roll, pitch), (temp), (x, y, z, w), (sys, gyro, accel, mag)

if __name__ == '__main__':
    br = BNO_Reader(10)
    br.start_bno_thread()
    print("Starting")
    print br.calibrate
    raw_input("Press a enter to continue...")
    while True:
        r = br.getReadings()
        print("heading: {0:.6} :roll: {1:.6} :pitch: {1:.6}\n-------------------\n".format(r[0][0], r[0][1], r[0][2]))
        time.sleep(.01)
