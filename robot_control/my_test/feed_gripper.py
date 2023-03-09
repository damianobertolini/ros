import numpy as np
import ros
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import math
import helper
from base_controllers.components.controller_manager import ControllerManager


def talker():
    #pub = rospy.Publisher('command', JointState, queue_size=1)
    pub = rospy.Publisher('/ur5/joint_group_pos_controller/command', Float64MultiArray, queue_size=1)

    controller_manager = ControllerManager(conf.robot_params[self.robot_name])

    ControllerManager.gm.move_gripper(p.time % 101)
    print("gripper at", p.time % 100)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass