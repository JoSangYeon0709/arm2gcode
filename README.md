# arm2gcode
g코드로 움직이는 로봇팔
ros noetic와 파이썬 3 환경에서 작동했습니다.
다른 환경에서 작동은 보장할수 없습니다.
또한 Rethink Robotics 회사의 벡스터에서만 테스트 해봤습니다.
하지만 moveit패키지를 사용하므로 
joint_state_topic = ['joint_states:=/robot/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
rospy.init_node('moveit_test', anonymous=True)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("left_arm")
left_current_pose =group.get_current_pose(end_effector_link='left_gripper').pose
이부분에서 ()안을 자신의 로봇에 맞춰서 수정 한다면 작동할 것 같습니다.
 
사용되는 모듈은 다음과 같습니다.
import sys
import matplotlib.pylab as plt
import math
import numpy as np
import gif
import openpyxl
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import *
import time
pi= math.pi

코드 작동에 문제가 있으면 알려주세요.
제가 문제를 빨리 확인하지 못하고, 많은 도움은 못 줄 것 같지만 최대한 도와드리겠습니다.

'''----------------------------------------------------------------------------'''

operate the robot arm with g-code
It worked with ros noetic and python 3 environment.
Operation in other environments is not guaranteed.
Also, I only tested it on Baxter from the company Rethink Robotics.

But since I use the moveit package
joint_state_topic = ['joint_states:=/robot/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
rospy.init_node('moveit_test', anonymous=True)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("left_arm")
left_current_pose = group.get_current_pose(end_effector_link='left_gripper').pose
In this part, if you modify the inside () according to your robot, it will work.

 
The modules used are:
import sys
import matplotlib.pylab as plt
import math
import numpy as np
import gif
import openpyxl
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import *
import time
pi = math.pi

Please let me know if you have any problems getting the code to work.
I can't quickly identify the problem, and I don't think I can help much, but I'll do my best to help.
