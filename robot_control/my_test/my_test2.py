#common stuff
from __future__ import print_function
import time as tm
import numpy as np
import numpy as nan
import math
import matplotlib.pyplot as plt
import csv
import helper

from base_controllers.utils.common_functions import *
from base_controllers.utils.ros_publish import RosPub
from base_controllers.components.inverse_kinematics.inv_kinematics_pinocchio import robotKinematics
import L1_conf as conf
import matplotlib.pyplot as plt

os.system("killall rosmaster rviz")
#instantiate graphic utils
ros_pub = RosPub("ur5")
robot = getRobotModel("ur5")
kin = robotKinematics(robot, conf.frame_name)

##############################
##exercise 2.6 : postural task
###############################
## desired task space position
p = np.array([0.5, -0.2, 0.5])
# initial guess
q_i = np.array([0.0, -0.0, 0.0, 0.0, 0., 0.0])

# postural elbow down
q_postural = np.array([0.0, 0.8, -0.8, -0.8, -0.8, 0.0])
# postural elbow up
#q_postural = np.array([0.0, -1.8, 1.8, -0.8, -0.8, 0.0])

q_ik, _, _ = kin.endeffectorInverseKinematicsLineSearch(p, conf.frame_name,
                                                        q_i,
                                                        verbose = True,
                                                        use_error_as_termination_criteria = False,
                                                        postural_task = True,
                                                        w_postural = 0.0001,
                                                        q_postural = q_postural)
print("Desired End effector \n", p)

robot.computeAllTerms(q_ik, np.zeros(6))
p_ik = robot.framePlacement(q_ik, robot.model.getFrameId(conf.frame_name)).translation
task_diff = p_ik - p
print("Point obtained with IK solution \n", p_ik)
print("Error at the end-effector: \n", np.linalg.norm(task_diff))
print("Final joint positions\n", q_ik)


print("q_i")
print(q_i)
tm.sleep(2.)
ros_pub.add_marker(p)
ros_pub.publish(robot, q_i)
tm.sleep(0.5)


ros_pub.add_marker(p)
ros_pub.publish(robot, q_postural)
tm.sleep(5.)

q_log = []



max=300
path = helper.sin_square(q_i,q_ik,max)
i=0
direction =0
while True:

#for i in range(max):


    if i >= max - 1 or i < 0:
        direction *= -1

    i += direction

    des_pos=path[i]

    #per il plot
    q_log.append(des_pos)

    ros_pub.add_marker(p)
    ros_pub.publish(robot, des_pos)
    print(des_pos)

    tm.sleep(0.5)



helper.linear(q_i,q_ik,max)

print("fine")
print(q_log)

# open file in write mode
with open("data.txt", 'w') as f:
   writer = csv.writer(f, delimiter=',')
   writer.writerows(q_log)
print('Done')

plt.plot(q_log)
plt.ylabel('test')
plt.show()

ros_pub.add_marker(p)
ros_pub.publish(robot, q_ik)
tm.sleep(5.)
ros_pub.deregister_node()







