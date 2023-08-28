#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

ini_odom = Odometry()
current_odom = Odometry()

def odom_callback(data):
    global current_odom
    current_odom = data

def reset_odometry():
    global ini_odom
    global current_odom

    rospy.init_node('reset_odometry', anonymous=True)
    ini_odom = rospy.wait_for_message("/odom", Odometry, timeout=None)
    current_odom = rospy.wait_for_message("/odom", Odometry, timeout=None)

    rospy.Subscriber("/odom", Odometry, odom_callback)

    pub = rospy.Publisher("/reset_odom", Odometry, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        reset_odom = current_odom
        reset_odom.pose.pose.position.x = current_odom.pose.pose.position.x - ini_odom.pose.pose.position.x
        reset_odom.pose.pose.position.y = current_odom.pose.pose.position.y - ini_odom.pose.pose.position.y
        rospy.loginfo("\n%s" % reset_odom.pose.pose.position)
        pub.publish(reset_odom)
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        reset_odometry()
    except rospy.ROSInterruptException:
        pass