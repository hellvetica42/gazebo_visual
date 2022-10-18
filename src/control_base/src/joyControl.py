#!/usr/local/bin/python3
import numpy as np
import rospy
import sys
from sensor_msgs.msg import Joy
from nubot_common.msg import VelCmd, OminiVisionInfo
from nubot_common.srv import BallHandle, Shoot
from HeadingPID import HeadingPID

if len(sys.argv) > 1:
    ROBOTNUM = int(sys.argv[1])
    JOYNUM = int(sys.argv[2])
else:
    ROBOTNUM = 1
    JOYNUM=0


lastX = 0
lastY = 0
lastW = 0

heading = 0
holdCommand = 0

ROBOTINDEX = None
headingPID = HeadingPID()

def joyCallback(data : Joy):
    global lastX, lastY, lastW, heading, holdCommand, ballShoot, headingPID
    joyV = np.array([-data.axes[0], data.axes[1]])
    cos, sin = np.math.cos(-heading), np.math.sin(-heading)
    rotmat = np.array([
        [cos, -sin],
        [sin, cos]

    ])
    #print(data)
    joyV = rotmat @ joyV
    joyV = joyV.astype(np.float32)
    lastX = joyV[0]*200
    lastY = joyV[1]*200

    #lastW = data.axes[3]*5

    tV = np.array([-data.axes[3], data.axes[4]])
    xAxis = np.array([1,0])

    cos = np.inner(tV, xAxis) / (np.linalg.norm(tV) * np.linalg.norm(xAxis))
    target = np.arccos(cos)
    if tV[1] < 0:
        target = -target

    if np.isnan(target):
        target = 0
    headingPID.updateTarget(target)

    holdCommand = data.buttons[5]

    if data.buttons[4]:
        force = 1 - (data.axes[2] + 1)/2
        if force < 0.1:
            force = 2
        else:
            force = force * 10
        try:
            ballShoot(force,1)
        except rospy.ServiceException as e:
            print("Failed to contact shoot service")
        #print(force)

    if data.buttons[10]:
        try:
            ballShoot(5,-1)
        except rospy.ServiceException as e:
            print("Failed to contact shoot service")







def omniCallback(data : OminiVisionInfo):
    global heading, ROBOTINDEX, headingPID, lastW
    if ROBOTINDEX is None:
        for i, robot in enumerate(data.robotinfo):
            if robot.AgentID == ROBOTNUM:
                ROBOTINDEX = i
                break
    heading = data.robotinfo[ROBOTINDEX].heading.theta

    lastW = headingPID.update(heading)

    #print(data.robotinfo[ROBOTINDEX].AgentID, lastX, lastY)


def start():
    global pub, holdCommand, ballShoot
    rospy.Subscriber(f'joy{JOYNUM}', Joy, joyCallback)
    rospy.Subscriber('/nubot{}/omnivision/OmniVisionInfo'.format(ROBOTNUM), OminiVisionInfo, omniCallback)

    pub = rospy.Publisher('/nubot{}/nubotcontrol/velcmd'.format(ROBOTNUM), VelCmd, queue_size=10)

    rospy.wait_for_service('/nubot{}/BallHandle'.format(ROBOTNUM))
    ballHandle = rospy.ServiceProxy('/nubot{}/BallHandle'.format(ROBOTNUM), BallHandle)
    ballShoot = rospy.ServiceProxy('/nubot{}/Shoot'.format(ROBOTNUM), Shoot)


    rospy.init_node(f'joyControl{ROBOTNUM}')
    rate = rospy.Rate(10)
    velcmd = VelCmd()
    while not rospy.is_shutdown():

        try:
            holdingBall = ballHandle(holdCommand)
            #print("HoldingBall", holdingBall)
        except rospy.ServiceException as e:
            print("Failed to contact service")

        velcmd.Vx = lastX
        velcmd.Vy = lastY
        velcmd.w = lastW
        pub.publish(velcmd)
        rate.sleep()
        

if __name__ == '__main__':
    start()