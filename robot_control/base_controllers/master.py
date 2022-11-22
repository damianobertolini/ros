#common stuff
from __future__ import print_function
import time as  tm

import helper
import math

from base_controllers.utils.common_functions import *
from base_controllers.utils.ros_publish import RosPub
from base_controllers.components.inverse_kinematics.inv_kinematics_pinocchio import robotKinematics
import L1_conf as conf
import csv
#os.system("killall rosmaster rviz")
#instantiate graphic utils
ros_pub = RosPub("ur5",node_name="master",topic="/joint_states")
robot = getRobotModel("ur5")
kin = robotKinematics(robot, conf.frame_name)

##############################
##exercise 2.6 : postural task
###############################
## desired task space position
p = np.array([0.5, -0.2, 0.5])

initial_pos = helper.get_initial_pos()         #<------------
#initial_pos = helper.get_inc_pos()

initial_joint_pos = helper.get_initial_joint_pos()

print("initila_pos")
print(initial_pos)

print("initial_joint_pos")
print(initial_joint_pos)
# initial guess


hit_log=[]
miss_log=[]
hit_j_log=[]
miss_j_log=[]



for i in range(len(initial_pos)):

    p = initial_pos[i]

    q_i = np.array([0.0, -0.0, 0.0, 0.0, 0., 0.0])#joint state
    # q_i = initial_joint_pos[i]

    # postural elbow down
    q_postural = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
    # postural elbow up
    # q_postural = np.array([0.0, -1.8, 1.8, -0.8, -0.8, 0.0])

    q_ik, _, _ = kin.endeffectorInverseKinematicsLineSearch(p, conf.frame_name,
                                                            q_i,
                                                            verbose=True,
                                                            use_error_as_termination_criteria=False,
                                                            postural_task=True,
                                                            w_postural=0.0001,
                                                            q_postural=q_postural)
    print("Desired End effector \n", p)

    robot.computeAllTerms(q_ik, np.zeros(6))
    p_ik = robot.framePlacement(q_ik, robot.model.getFrameId(conf.frame_name)).translation
    task_diff = p_ik - p
    print("Point obtained with IK solution \n", p_ik)
    print("Error at the end-effector: \n", np.linalg.norm(task_diff))
    print("Final joint positions\n", q_ik)

    if math.fabs(np.linalg.norm(task_diff)) < 0.1:
        hit_log.append(p)
        hit_j_log.append(q_ik)
    else:
        miss_log.append(p)
        miss_j_log.append(q_ik)
    """
    tm.sleep(2.)
    ros_pub.add_marker(p)
    ros_pub.publish(robot, q_i)
    tm.sleep(5.)

    ros_pub.add_marker(p)
    ros_pub.publish(robot, q_postural)
    tm.sleep(5.)
    
    ros_pub.add_marker(p)
    ros_pub.publish(robot, q_ik)
    tm.sleep(5.)
    """
    print("done: " + str(i))

    #ros_pub.deregister_node()


"----------------------------------------------------------------------------------"

helper.save(hit_log,"hit_log.txt")
helper.save(miss_log,"miss_log.txt")
helper.save(hit_j_log,"hit_j_log.txt")
helper.save(miss_j_log,"miss_j_log.txt")



p = np.array([0.5, -0.2, 0.5])
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


tm.sleep(2.)
ros_pub.add_marker(p)
ros_pub.publish(robot, q_i)
tm.sleep(5.)

ros_pub.add_marker(p)
ros_pub.publish(robot, q_postural)
tm.sleep(5.)


ros_pub.add_marker(p)
ros_pub.publish(robot, q_ik)
tm.sleep(15.)

ros_pub.deregister_node()








