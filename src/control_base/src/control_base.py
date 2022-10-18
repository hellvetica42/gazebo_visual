#!/usr/local/bin/python3
import rospy
from nubot_common.msg._OminiVisionInfo import OminiVisionInfo # type: ignore
from nubot_common.msg._VelCmd import VelCmd # type: ignore
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from HeadingPID import HeadingPID
from PositionPID import PositionPID
import sys

if __name__ == "__main__":
    if len(sys.argv) < 2:
        ROBOT_NUM = 1
    else:
        ROBOT_NUM = int(sys.argv[1])
    ROBOT = f'nubot{ROBOT_NUM}'

    anglePID = HeadingPID()
    posPID = PositionPID()

    def control_loop_callback(data):
        global anglePID, posPID

        #print(data.ballinfo.real_pos.angle)
        thisrobot = data.robotinfo[ROBOT_NUM-1]

        heading = thisrobot.heading.theta
        anglePID.update(heading)
        #print(f"Heading: {anglePID.heading}, target: {anglePID.target_heading}, vel: {anglePID.vel_theta}")

        posX = thisrobot.pos.x
        posY = thisrobot.pos.y
        posPID.update(posX, posY)
        #print(f"Pos: {posPID.posX} {posPID.posY}, target: {posPID.targetX} {posPID.targetY}, vel: {posPID.velX} {posPID.velY}")

        #anglePID.updateTarget(heading + data.ballinfo.real_pos.angle)

    def heading_command_callback(data):
        global anglePID
        anglePID.updateTarget(data.data)

    def position_command_callback(data):
        global posPID
        #print(data)
        posPID.updateTarget(data.x, data.y)

    rospy.init_node(f"{ROBOT}_control", anonymous=True)
    velCmdPub = rospy.Publisher(f"{ROBOT}/nubotcontrol/velcmd", VelCmd)

    rospy.Subscriber(f"{ROBOT}/omnivision/OmniVisionInfo", 
    OminiVisionInfo, control_loop_callback)

    rospy.Subscriber(f"{ROBOT}/control_base/Heading", Float32, heading_command_callback)
    rospy.Subscriber(f"{ROBOT}/control_base/Position", Point, position_command_callback)

    rate = rospy.Rate(10)
    velmsg = VelCmd()
    velmsg.Vx = 0.0
    velmsg.Vy = 0.0
    velmsg.w = 0.0
    while not rospy.is_shutdown():
        velmsg.w = anglePID.vel_theta 
        velmsg.Vx = posPID.velX
        velmsg.Vy = posPID.velY
        velCmdPub.publish(velmsg)
        rate.sleep()

