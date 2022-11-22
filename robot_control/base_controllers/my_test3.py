import rospy as ros
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import helper
from base_controllers.components.inverse_kinematics.inv_kinematics_pinocchio import robotKinematics
import base_controllers.utils.kin_dyn_utils as directKinematic
from base_controllers.utils.common_functions import *
import L1_conf as conf
import numpy

class driver:

    def send_des_jstate(publisher, q_des, qd_des=[0.0,0.0,0.0,0.0,0.0,0.0], tau_ffwd=[0.0,0.0,0.0,0.0,0.0,0.0]):
        # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
        msg = JointState()
        msg.position = q_des
        msg.velocity = qd_des
        msg.effort = tau_ffwd
        publisher.publish(msg)
        #print("published: ")
        #print(msg)

    def q_to_p(self,p):
        q_i = np.array([0.0, -0.0, 0.0, 0.0, 0., 0.0])
        q_ik, _, _ = self.kin.endeffectorInverseKinematicsLineSearch(p, conf.frame_name,verbose=True,)
        return q_ik

def talker(d):
    pub = ros.Publisher('command', JointState, queue_size=1)
    ros.init_node('talker', anonymous=False)
    rate = ros.Rate(20) # 10hz
    ros.sleep(1)

    q = helper.read_file("hit_j_log.txt")
    p = helper.read_file("hit_log.txt")

    q = numpy.asarray(q)
    p = numpy.asarray(p)

    robot = getRobotModel("ur5")
    d.kin = robotKinematics(robot, conf.frame_name)

    q_ik1=d.q_to_p(p[0])
    q_ik2=d.q_to_p(p[1])

    
    driver.send_des_jstate(pub, q_ik1)
    ros.sleep(1)

    driver.send_des_jstate(pub, q_ik2)
    ros.sleep(1)
    
    robot.computeAllTerms(q_ik2, np.zeros(6))
    p_ik1 = robot.framePlacement(q_ik1, robot.model.getFrameId(conf.frame_name)).translation
    p_ik2 = robot.framePlacement(q_ik2, robot.model.getFrameId(conf.frame_name)).translation
    task_diff = p_ik2 - p_ik1

    print("Error at the end-effector: \n", np.linalg.norm(task_diff))

    path=helper.sin_square(q_ik1,q_ik2,steps=100)


    helper.save(path,"path.txt")


    i = 0

    print("starting")
    while not ros.is_shutdown():
        #hello_str = "hello world %s" % ros.get_time()
        #ros.loginfo(hello_str)


        q_ik1 = d.q_to_p(p[i])
        q_ik2 = d.q_to_p(p[i + 1])


        path = helper.sin_square(q_ik1, q_ik2, steps=300)
        print(f"pos {i} e {i + 1}")
        for j in range(len(path)):

            #driver.send_des_jstate(pub,path[i])
            driver.send_des_jstate(pub,path[j])
            ros.sleep(0.006)
            print(f"{j}/{300} of {i} and {i + 1}")
            j+=1
        print("done")
        ros.sleep(1)
        i+=1

        rate.sleep()

if __name__ == '__main__':
    try:
        d = driver()
        talker(d)
    except ros.ROSInterruptException:
        pass