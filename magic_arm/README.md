# MagicArm


## Quickstart
This is a ROS compatible project. Add the magicarm folder to a ROS workspace and rebuild to start using it  
Running roslaunch magicarm MagicArm.launch will launch the arm control and a ROSbridge connection for the web interface  


## Important files
- MagicArm.py : The main control file. Manages the servos and stepper motor.
- MagicArmRosWrapper.py : A simple wrapper for MagicArm that allows it to subscribe to JointTrajectory messages
- ServoDriver.py : Interface for using a single servo on the adafruit board. 
- StepperDriver.py : Interface for driving a servo with a limit switch
