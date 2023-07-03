import numpy as np
import ros
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import math
import helper
import time


from my_vision_messages.msg import Pointxyz
from my_vision_messages.msg import Block
from my_vision_messages.msg import BlockList

def talker():
    #pub = rospy.Publisher('command', JointState, queue_size=1)
    pub = rospy.Publisher('/vision_results', BlockList, queue_size=1)

    rospy.init_node('feed_vision_node', anonymous=False)
    rate = rospy.Rate(20) # 10hz


    p1 = Pointxyz()
    p2 = Pointxyz()
    b1 = Block()
    b2 = Block()
    block_list_confs = []
    block_list = BlockList()

    p1_x=0.435078
    p1_y=0.294027
    p1_z = 0.871311
    b1.point=p1

    b1.class_number = "3"
    #b1.rot_angle=0.360186
    b1_rot_angle=    0.44     *360/(math.pi*2)
    b1.top_btm_l_r = ["top_btm_l_r 1","top_btm_l_r 2"]
    b1.up_dw_stand_lean="up_dw_stand_lean 1"
    b1_confidence=99
    b1.world_point = p1
#------------------------------------
    """
    p2.x = 0.593995
    p2.y = 0.416553
    p2.z = 0.871311
    b2.point = p2

    b2.class_number="5"
    b2.rot_angle = 0.453415
    b2.top_btm_l_r = ["", ""]
    b2.up_dw_stand_lean = "up_dw_stand_lean 2"
    b2.confidence = 100
    b2.world_point = p2
    """

    #msg = BlockList([b1,b2])
    msg = BlockList()
    #msg.blocks.append(b1)

    p1_x=0.25078
    p1_y=0.454027
    p1_z = 0.871311

    point = Pointxyz(p1_x, p1_y, p1_z)

    block = Block()
    block.class_number = str(3)
    block.point = point
    block.rot_angle = 0.44     *360/(math.pi*2)
    block.top_btm_l_r = ["", ""]
    block.up_dw_stand_lean = ""
    block.confidence = 99
    block.world_point = point

    # this are not needed in this program
    block.top_btm_l_r = ["", ""]
    block.up_dw_stand_lean = ""


    msg.blocks.insert(0,block)
    while True:
      time.sleep(1)
      #msg.data = q_des
      pub.publish(msg)
      print("pubblicato")
      print(msg)



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

"""
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="default">
    <physics type='ode'>
      <gravity>0 0 -0.1</gravity>
      <!-- max step size has to be a multiple of the desired task rate-->
      <max_step_size>0.001</max_step_size> 
      <real_time_factor>1</real_time_factor>
    </physics>
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <name>tavolo</name>
      <uri>model://tavolo</uri>
      <pose>0.0 0.0 0.0 0 0 0</pose>
    </include>


    <include>
     <name>block5high</name>
     <uri>model://block5high</uri>
     <pose>0.59399 0.41658 0.97980 0.00000 0.00000 0.45357</pose>
    </include>


    <include>
     <name>block3high</name>
     <uri>model://block3high</uri>
     <pose>0.43508 0.29405 1.09258 0.00000 0.00000 0.36046</pose>
    </include>



    <gui>
     <camera name="gzclient_camera">
     <pose>1. 3.2 2.2 0. 0.4 -1.75</pose>
     </camera>
    </gui>

  </world>
</sdf>

"""