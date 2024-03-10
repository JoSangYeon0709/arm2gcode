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

'''----------------------------------------------------------------------------'''
