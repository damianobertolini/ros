import numpy as np
import ros
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('command', JointState, queue_size=1)
    rospy.init_node('talker', anonymous=False)
    rate = rospy.Rate(20) # 10hz
    rospy.sleep(1)

    q_des = np.array([ 0-0.011396809745719522,-0.058403114301534,0.057916919191157046,0.13126164724779377,-0.11657080834973482,2.59011309487694e-17    ])
    q_des = np.array([ -0.011396809745719522,-0.058403114301534,0.057916919191157046,0.13126164724779377,-0.11657080834973482,2.59011309487694e-17    ])
    q_des = np.array([ 0.42573190607248773,-0.1994619269005705,-0.10801933612422758,-0.5961038403990017,0.5908584642529011,-9.129249380472037e-17   ])
    q_des = np.array([ 0-0.011396809745719522,-0.058403114301534,0.057916919191157046,0.13126164724779377,-0.11657080834973482,2.59011309487694e-17    ])
    q_des = np.array([0.089159, 0, 0, 0.10915, 0.09465, 0.0823])
    q_des = np.array([0.026985191872504866,-0.6681302050626805,-0.3276068811262316,-0.0235015812501439,0.02016366270262715,6.381949412090099e-17])

    qd_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    tau_ffwd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


    msg = JointState()
    msg.position = q_des
    msg.velocity = qd_des
    msg.effort = tau_ffwd
    pub.publish(msg)
    print("pubblicato")
    rospy.sleep(5)

    q_des = np.array([-0.10879, -0.70291, -0.36752,  0.12198, -0.10335, -0.     ])

    msg.position = q_des
    msg.velocity = qd_des
    msg.effort = tau_ffwd
    pub.publish(msg)
    print("pubblicato")
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
