#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

base_speed = 3
currentPosition = None
roll = 0
pitch = 0
yaw = 0

def get_current_position():
    global currentPosition
    global roll, pitch, yaw
    
    rospy.init_node('move_to_center', anonymous=True)
    data = rospy.wait_for_message("/reset_odom", Odometry, timeout=None)

    currentPosition = data.pose.pose
    qvalue = [
        currentPosition.orientation.x,
        currentPosition.orientation.y,
        currentPosition.orientation.z,
        currentPosition.orientation.w,
    ]  # creates list from quaternion since it was not originally
    (roll, pitch, yaw) = euler_from_quaternion(qvalue)

def move_to_center():
    get_current_position()

    pub1 = rospy.Publisher('/mybot/joint1_velocity_controller/command', Float64, queue_size=100)
    pub2 = rospy.Publisher('/mybot/joint2_velocity_controller/command', Float64, queue_size=100)
    pub3 = rospy.Publisher('/mybot/joint3_velocity_controller/command', Float64, queue_size=100)
    pub4 = rospy.Publisher('/mybot/joint4_velocity_controller/command', Float64, queue_size=100)
    pub5 = rospy.Publisher('/mybot/joint5_velocity_controller/command', Float64, queue_size=100)
    pub6 = rospy.Publisher('/mybot/joint6_velocity_controller/command', Float64, queue_size=100)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        get_current_position()
        if currentPosition.position.y < 0.15:
            move_str = "Move to the center %s" % currentPosition.position.y
            rospy.loginfo(move_str)

            flcontrol = base_speed
            frcontrol = base_speed
            blcontrol = base_speed
            brcontrol = base_speed
        else:
            move_str = "Center! %s" % currentPosition.position.y
            rospy.loginfo(move_str)

            flcontrol = 0
            frcontrol = 0
            blcontrol = 0
            brcontrol = 0

            pub1.publish(flcontrol) 
            pub2.publish(frcontrol) 
            pub3.publish(blcontrol) 
            pub4.publish(brcontrol)
            pub5.publish(blcontrol) 
            pub6.publish(brcontrol)
            rospy.signal_shutdown("Center finished!")
        
        pub1.publish(flcontrol) 
        pub2.publish(frcontrol) 
        pub3.publish(blcontrol) 
        pub4.publish(brcontrol)
        pub5.publish(blcontrol) 
        pub6.publish(brcontrol)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_to_center()
    except rospy.ROSInterruptException:
        pass