
import rospy
import csv
import time as tm
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import my_test as controller

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

def send_des_jstate(pub, q_des, qd_des=[-0.0, 0.0, -0.0, -0.0, -0.0, 0.0], tau_ffwd=[-0.0, 0.0, -0.0, -0.0, -0.0, 0.0]):
    # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
    msg = JointState()
    msg.position = q_des
    msg.velocity = qd_des
    msg.effort = tau_ffwd
    pub.publish(msg)

def talker(p):
    p.start()
    pub = rospy.Publisher('/command', String, queue_size=1)#pipe
    rospy.init_node('talker', anonymous=False)#programma
    rate = rospy.Rate(10)  # 10hz

    max = 300
    i = 0
    direction = 1
    re = get_traj()
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)

        #pub.publish(hello_str)

        rate.sleep()
        if __name__ == '__main__':
        #if __name__ == '__main__':
            try:


                rospy.loginfo(hello_str)
                if i >= max - 1 or i < 0:
                    direction *= -1
                i += direction

                p.send_des_jstate(re[i],[-0.0, 0.0, -0.0, -0.0, -0.0, 0.0],[-0.0, 0.0, -0.0, -0.0, -0.0, 0.0])

                tm.sleep(0.5)

            except rospy.ROSInterruptException:
                pass


robot_name="ur5"
p = controller.BaseControllerFixed(robot_name)
talker(p)

print("fine")