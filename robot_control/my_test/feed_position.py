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

    def send_des_jstate(publisher, q_des, qd_des=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        tau_ffwd=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
        msg = JointState()
        msg.position = q_des
        msg.velocity = qd_des
        msg.effort = tau_ffwd
        publisher.publish(msg)
        # print("published: ")
        # print(msg)

    def q_to_p(self, p):
        q_i = np.array([0.0, -0.0, 0.0, 0.0, 0., 0.0])
        q_ik, _, _ = self.kin.endeffectorInverseKinematicsLineSearch(p, conf.frame_name, verbose=True, )
        return q_ik


def talker(d):
    pub = ros.Publisher('command', JointState, queue_size=1)
    ros.init_node('talker', anonymous=False)
    rate = ros.Rate(20)  # 10hz
    ros.sleep(1)

    q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    q[0]=0.6
    driver.send_des_jstate(pub, q)
    print("pubblicato")
    """
    while not ros.is_shutdown():
    
        

        rate.sleep()
    """

if __name__ == '__main__':
    try:
        d = driver()
        talker(d)
    except ros.ROSInterruptException:
        pass