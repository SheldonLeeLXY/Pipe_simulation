#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from cv_bridge import CvBridge, CvBridgeError
import cv2
from math import pi
import os
from tf.transformations import euler_from_quaternion

currentJointState = JointState()
camera_ini_pos = None
last_cam = None
count = 0
sum_imageset = 6
campath = [0] * sum_imageset
cam_link_pose = None
roll = 0
pitch = 0
yaw = 0

def image_callback(msg):
    global count
    global last_cam
    try:
        bridge = CvBridge()
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        if last_cam == None:
            print("Receive an image!")
            last_cam = currentJointState.position[6]
            count += 1
            path = "/home/sheldon/camvideo_9/record_imageset"
            if not os.path.exists(path):
                os.makedirs(path)
                print("------New Folder!-----")
            with open(path+"/info.txt", "w") as f:
                f.write("cam_joint_pos:\n" + str(cam_link_pose) + "\nroll: " + str(roll) + "\npitch: " + str(pitch) + "\nyaw: " + str(yaw))
            cv2.imwrite(path + "/camera_image" + str(count) + ".jpeg", cv2_img)
        else:
            # print("Current: " + str(currentJointState.position[6]))
            # print(campath[count-1])
            if abs(currentJointState.position[6] - campath[count-1]) < 0.05  and count < sum_imageset:
                print("Receive an image!")
                last_cam = currentJointState.position[6]
                count += 1
                path = "/home/sheldon/camvideo_9/record_imageset"
                if not os.path.exists(path):
                    os.makedirs(path)
                    print("------New Folder!-----")
                cv2.imwrite(path + "/camera_image" + str(count) + ".jpeg", cv2_img)

def get_cam_joint_state():
    global currentJointState
    
    rospy.init_node("record_images")
    data = rospy.wait_for_message("/mybot/joint_states", JointState, timeout=None)
    currentJointState = data

def get_cam_link_pose():
    global cam_link_pose, roll, pitch, yaw

    data = rospy.wait_for_message("/gazebo/link_states", LinkStates, timeout=None)
    cam_link_pose = data.pose[9].position
    qvalue = [
        data.pose[9].orientation.x,
        data.pose[9].orientation.y,
        data.pose[9].orientation.z,
        data.pose[9].orientation.w,
    ]  # creates list from quaternion since it was not originally
    (roll, pitch, yaw) = euler_from_quaternion(qvalue)
    rospy.loginfo("cam_position: %s \n roll: %s pitch: %s yaw: %s" % (cam_link_pose, roll, pitch, yaw))

def record_images():
    global camera_ini_pos
    global campath

    get_cam_joint_state()
    get_cam_link_pose()

    camera_ini_state = currentJointState.position[6]
    for i in range(len(campath)):
        campath[i] = (i + 1) * (2 * pi / len(campath)) + camera_ini_state
    # print(campath)

    rospy.loginfo("Rotate from: %s - %s" % (camera_ini_state, camera_ini_state + 2 * pi))
    pub = rospy.Publisher("/mybot/joint7_position_controller/command", Float64, queue_size=100)
    rate = rospy.Rate(10)

    rospy.Subscriber("/camera1/image_raw", Image, image_callback)

    while not rospy.is_shutdown():
        get_cam_joint_state()
        if currentJointState.position[6] < camera_ini_state + 2 * pi - 0.1:
            pass
            rospy.loginfo("Rotateing the camera: %s" % currentJointState.position[6])
        else:
            rospy.loginfo("Rotate finished! %s" % currentJointState.position[6])
            rospy.signal_shutdown("Rotate finished!")
        camcontrol = campath[count-1]
        pub.publish(camcontrol)
        rate.sleep()
    

if __name__ == "__main__":
    try:
        record_images()
    except rospy.ROSInterruptException:
        pass