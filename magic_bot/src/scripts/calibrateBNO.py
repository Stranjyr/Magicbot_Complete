from Adafruit_BNO055 import BNO055
import json

CALIBRATION_FILE = 'calibration.json'
def main():
	bno = BNO055.BNO055(rst=7)
	print("Press ctr-c to save calibration file")
	data = []
	try:
		while True:
			data = bno.get_calibration()
			print(data)
	except KeyboardInterrupt:
	    # Write the calibration to disk.
	    with open(CALIBRATION_FILE, 'w') as cal_file:
			json.dump(data, cal_file)

if __name__ == '__main__':
	main()