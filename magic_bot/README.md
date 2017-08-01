#Magicbot

##Quickstart
This is a ROS compatible folder. Add it to a ROS workspace and rebuild to setup.  
Launch with roslaunch magicbot MagicBot.launch  
If the arm is installed, roslaunch magicbot Zombiebot.launch will launch both the arm and the base  

## Important Files
- MagicBot.py : The main control file for the magicbot base
- MagicBotRosWrapper.py : Allows for ROS controll of the magicbot base
- MotorControl.py : Interfaces the H-Bridge.py file with a more intuitive motor control
- H_Bridge.py : python h_bridge driver
- Endcoder.py : Interupt based motor encoder