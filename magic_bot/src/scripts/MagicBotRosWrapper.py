#!/usr/bin/env python
from MagicBot import MagicBot
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
import threading
from Watchdog import Watchdog

class MagicBotRosWrapper(MagicBot):
    '''
    init
    drivers : a list of items to setup the motor params. Each element should have a list of pins
        as pin A , pin B, and a pwm pin
    encs: a list of items to setup the encoders. Each element should have a list of pins as 
        pin A, pin B
    '''
    def __init__(self, drivers, encs):
        rospy.init_node("magicbot")
        cal = rospy.get_param('~bno_cal_file', 'calibration.json')
        print cal
        MagicBot.__init__(self, drivers, encs, cal)
        
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
        #self.odom_broadcast = tf.TransformBroadcaster()

        self.vel_sub = rospy.Subscriber("cmd_vel", Twist, self.setVel, queue_size=1)

        self.watchdog = Watchdog(1, self.moveBase(0, 0))
    
    #launches the needed threads and starts the ros subscriber and publisher
    def start(self):
        MagicBot.start(self)
        self.odom_thread = threading.Thread(target=self.rosLoop)
        self.odom_thread.daemon = True  # Don't let the thread block exiting.
        self.odom_thread.start()
        rospy.on_shutdown(self.close)
        rospy.spin()

    #callback for the ros subscriber - sets the velocity of the motors
    def setVel(self, data):
        x_speed = data.linear.x
        a_speed = data.angular.z
        self.watchdog.reset()
        self.moveBase(x_speed, a_speed)

    #main publisher loop
    def rosLoop(self):
        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
            '''self.odom_broadcast.sendTransform(
                (self.x, self.y, 0.0),
                (odom_quat),
                self.lastTime,
                "base_link",
                "odom"
                )'''
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "odom"
            odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*odom_quat))
            odom.child_frame_id = "base_link"

            if self.dt != 0:
                odom.twist.twist = Twist(Vector3(self.vX/self.dt, 0.0, 0.0), Vector3(0, 0, self.dTh/self.dt))
            else:
                odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            self.odom_pub.publish(odom)
            r.sleep()

#Start a magicbot with the default pins
if __name__ == '__main__':
    mb = MagicBotRosWrapper([(27, 22, 17), (23, 24, 18)], [(5, 6), (13, 19)])
    mb.start()