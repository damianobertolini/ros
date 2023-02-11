import random

from messages.msg import Pointxyz
from messages.msg import Block
from messages.msg import BlockList

import rospy

rospy.init_node('vision_node', anonymous=True)

res_pub = rospy.Publisher('vision_results', BlockList, queue_size=10)


while True:
    block_list = BlockList()
    block_list_confs = []

    for i in range(5):
        point = Pointxyz(random.randint(1000,10000) / 1000, random.randint(1000,10000) / 1000, random.randint(1000,10000) / 1000)

        block = Block()
        block.class_number = "1 e 2"
        block.point = point
        block.rot_angle = random.randint(0, 180)
        block.top_btm_l_r = "bottmo"
        block.up_dw_stand_lean = "ciao"
        block.confidence = random.randint(1, 1000) / 1000

        block_list.blocks.append(block)

        # print(block)

    res_pub.publish(block_list.blocks)


