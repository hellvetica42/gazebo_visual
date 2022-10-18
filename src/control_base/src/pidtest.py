#!/usr/local/bin/python3
from logging.config import listen
import pprint
import rospy
import nubot_common.msg as msgs # type: ignore
from std_msgs.msg import Float32
import time
from utils import shortestAngleDiff

if __name__ == "__main__":

    ROBOT_NUM = 1
    ROBOT = f'nubot{ROBOT_NUM}'

    kp, ki, kd = 3, 0, 0

    heading = 0
    target_heading = 0
    vel_theta = 0
    prev_time = time.time()
    cum_error = 0
    rate_error = 0
    last_error = 0

    def robotinfo_callback(data):
        global heading, vel_theta, prev_time, cum_error, rate_error, last_error
        thisrobot = data.robotinfo[ROBOT_NUM-1]
        heading = thisrobot.heading.theta

        curr_time = time.time()
        elapsed_time = curr_time - prev_time
        #error = target_heading - heading
        error = shortestAngleDiff(target_heading, heading)
        print("ERROR", error)

        cum_error += error * elapsed_time
        rate_error = (error - last_error)/elapsed_time

        vel_theta = kp*error + ki*cum_error + kd*rate_error
        vel_theta = max(-10, min(10, vel_theta))

        last_error = error
        prev_time = curr_time
        print(f"Heading: {heading}, target: {target_heading}, vel: {vel_theta}")

    def command_callback(data):
        global target_heading
        target_heading = data.data
        print("New target heading:", target_heading)

    def listener():
        global vel_theta
        pub = rospy.Publisher(f"{ROBOT}/nubotcontrol/velcmd", msgs._VelCmd.VelCmd)

        rospy.init_node(f"{ROBOT}_control", anonymous=True)

        rospy.Subscriber(f"{ROBOT}/omnivision/OmniVisionInfo", 
        msgs._OminiVisionInfo.OminiVisionInfo, robotinfo_callback)
        rospy.Subscriber(f"{ROBOT}/control_base/Heading", 
        Float32, command_callback)

        rate = rospy.Rate(10)

        velmsg = msgs._VelCmd.VelCmd()
        velmsg.Vx = 0.0
        velmsg.Vy = 0.0
        velmsg.w = 0.0
        while not rospy.is_shutdown():
            velmsg.w = vel_theta
            pub.publish(velmsg)
            rate.sleep()

    try:
        listener()
    except rospy.ROSInterruptException: pass

    