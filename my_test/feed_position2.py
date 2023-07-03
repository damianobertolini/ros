import numpy as np
import ros
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import math
import helper

def talker():
    #pub = rospy.Publisher('command', JointState, queue_size=1)
    pub = rospy.Publisher('/ur5/joint_group_pos_controller/command', Float64MultiArray, queue_size=1)

    rospy.init_node('talker', anonymous=False)
    rate = rospy.Rate(20) # 10hz
    rospy.sleep(1)

    q_des = np.array([  0.0110064,-0.454544-0.700532,-0.519572,0.0154225, 0.656797   ])
    #q_des = [math.pi, -math.pi / 2, 0.0, -3, -math.pi/2, 0.0]


    #posizione base
    q_des = np.array([-0.3223527113543909, 0,0,-0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085,  -1.0017417112933558])
    q_des = np.array([-0.153276,
                      0.0845086,
                      -1.96304,
                      0.307725,
                      1.5708,
                      1.72407,0,0])
    #q_des = np.array([-0.153267,-1.6676,2.15913,-2.06233,1.5708,1.72406,0,0])
    q_des = np.array([-0.153267,-1.21891,1.64208,1.14763,-1.5708,-1.41753,0,0])
    #q_des = np.array([-2.65803,-3.4786,1.64208,0.265731,1.5708,-2.05436,0,0])###
    #q_des = np.array([-2.65803,-3.48335,2.15913,2.89502,-1.5708,1.08723,0,0])###
    #q_des = np.array([-0.153267,0.341761,-2.15913,0.246578,1.5708,1.72406,0,0])###
    #q_des = np.array([-0.153267,0.337012,-1.64208,2.87586,-1.5708,-1.41753,0,0])###
    q_des = np.array([-2.65803,-1.92268,-1.64208,1.99397,1.5708,-2.05436,0,0])
    #q_des = np.array([-2.65803,-1.47399,-2.15913,-1.07926,-1.5708,1.08723,0,0])


    #q_des = np.array([6.14045,4.69121,2.08894,4.21541,1.57079,1.71351,0,0])
    q_des = np.array([0.673461,-1.7250,1.67929,4.75817,1.57078,0.897367,0,0])
    q_des = np.array([0.941642,-1.279,1.45634,4.53518,1.57076,0.629147, 0, 0])
    q_des = np.array([-0.3199939453914433,-0.7800018605937107,-2.5597365931600504, -1.6299984483416363, -1.569999324390567, 3.489998232683448,0,0,0])
    q_des = np.array([-0.32, -0.78, -2, -1.8, -1.57, 3.49,0,0])
    #q_des = np.array([3.175,-1,-2.1,    -1.5, -1.5, 1.77,2.5,2.5,2.5])
    #q_des = np.array([5.23,-1.35,-2.1,    -1.5, -1.5, 1.77,2.5,2.5,2.5])
    #q_des = np.array([-0.3199939453914433,-0.7800018605937107,-2.5597365931600504, -1.6299984483416363, -1.569999324390567, 3.489998232683448,0,0,0])

    #q_des = np.array([-0.3199939453914433,-0.7800018605937107,-2.5597365931600504, -1.6299984483416363, -1.569999324390567, -2.79318,0,0,0])
    #q_des = np.array([-0.714973,-1.48881,-2.28715,0.595743,-2.67036,0.0534161,0,0,0])#braccio incastrato

    #q_des = np.array([-0.4223527113543909, -0.505794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085,  -1.0017417112933558])

    #q_des = q_des = [math.pi, -math.pi / 2, -math.pi/2, -3, -math.pi/2, 0.0,0,0]


    #q_des = q_des = [math.pi, -math.pi / 2, -math.pi / 2, -3, -math.pi / 2, 0, -1, -1]



    msg = Float64MultiArray()
    msg.data = q_des
    pub.publish(msg)
    print("pubblicato")
    print(msg)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass