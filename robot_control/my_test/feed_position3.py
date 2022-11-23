import my_test as controller
import rospkg
import rospy as ros
import numpy as np
import csv

import base_controllers.params as conf
from base_controllers.utils.common_functions import plotJoint

robotName = "ur5"



def get_traj():
    re2 = []
    with open("data.txt", 'r') as file:
        csvreader = csv.reader(file)
        for row in csvreader:
            re2.append(row)
    t = len(re2[0])
    re = [[0 for i in range(t)] for j in range(len(re2))]
    for i in range(len(re2)):
        for j in range(len(re2[0])):
            re[i][j] = float(re2[i][j])
    return re



def talker(p):
    p.start()
    #p.startSimulator()

    if (robotName == 'ur5'):
        p.loadModelAndPublishers(rospkg.RosPack().get_path('ur_description') + '/urdf/' + p.robot_name + '.urdf.xacro',node_name="feed_position3")
    else:
        p.loadModelAndPublishers()

    p.initVars()
    p.startupProcedure()
    ros.sleep(1.0)
    p.q_des_q0 = conf.robot_params[p.robot_name]['q_0']
    # loop frequency
    rate = ros.Rate(1 / conf.robot_params[p.robot_name]['dt'])
    rate=ros.Rate(2) #2 hz
    i=0
    max=300
    direction = 1
    re = get_traj()
    # control loop
    while not ros.is_shutdown():

        if i >= max - 1 or i <0:
            direction *= -1
        i += direction

        p.q_des = re[i]
        # send commands to gazebo
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        ##flag1
        print(i)
        i+=1

        # log variables
        p.logData()

        # wait for synconization of the control loop
        rate.sleep()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),
                          3)  # to avoid issues of dt 0.0009999


if __name__ == '__main__':
    p = controller.BaseControllerFixed(robotName)

    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        if conf.plotting:
            plotJoint('position', 0, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
                      p.tau_ffwd_log, p.joint_names)
            plotJoint('torque', 1, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
                      p.tau_ffwd_log, p.joint_names)

