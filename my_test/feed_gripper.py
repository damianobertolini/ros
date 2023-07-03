import rospy
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('/gripper', Float64, queue_size=1)
    rospy.init_node('gripper_feeder', anonymous=True)
    rate = rospy.Rate(10) # 10hz




    grip=100


    pub.publish(grip)
    print(f"sent {grip}")


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass