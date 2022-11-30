import numpy as np
import matplotlib.pyplot as plt
import csv
import math
import helper
import rospy as ros
#from base_controllers.utils.ros_publish import RosPub
import random
import base_controllers.utils.kin_dyn_utils as directKinematic
from base_controllers.utils.ros_publish import RosPub
from base_controllers.utils.common_functions import *
import time
from base_controllers.components.inverse_kinematics.inv_kinematics_pinocchio import robotKinematics
import L1_conf as conf



def ra():
    return random.randint(-80000,80000) / 100000




"""
init = np.array([0.5, -0.2, 0.5])
q_ik = np.array([-0.69004, 0.04096, -1.65882,-0.85673, -0.8407,   0.])
q_i = np.array([0.0, -0.0, 0.0, 0.0, 0., 0.0])

max=100
for i in range(max):
    print(q_i-q_ik*(i/max))

print("-----------------------------------")
lis=[]
arr = np.array([1, 2, 3, 4, 5])
arr = np.append(arr,arr)
lis.append(arr)
lis.append(arr+arr)
lis.append(arr*0.5)
print(lis)
print(lis[2][5])

"---------------------------------------------------------------------"
# empty list to read list from a file
re = []


# open file and read the content in a list
with open("data.txt", 'r') as file:
    csvreader = csv.reader(file)
    for row in csvreader:
        re.append(row)
print(re)

for i in range (len(re)):
    for j in range(len(re[0])):
        re[i][j]=float(re[i][j])

ch1=[]
ch2=[]
ch3=[]
ch4=[]
ch5=[]

a=int(0.0)

for i in range(len(re)):
    ch1.append(math.trunc(float(re[i][0])*10000))
    ch2.append(math.trunc(float(re[i][0])*10000))
    ch3.append(math.trunc(float(re[i][0])*10000))
    ch4.append(math.trunc(float(re[i][0])*10000))
    ch5.append(math.trunc(float(re[i][0])*10000))
# display list

plt.plot(ch1)
plt.ylabel('test2')
#plt.show()


plt.plot(re)
plt.ylabel('test2')
#plt.show()

for i in range(len(re)):
    for j in range(len(re[0])):
        re[i][j] = float(re[i][j])
        


node_name = "test"
node_name = node_name + str(int(random.random()*10000))
print(node_name)

max=10.0
min=0.2
diff = max-min
pos=[]
print("------------------------------------------------")


for i in range(50):
    pos.append([ra(),ra(),ra()])
i=0

print(pos)

with open("start_pos.txt", 'w') as f:
   writer = csv.writer(f, delimiter=',')
   writer.writerows(pos)
print('Done')

sjp=[]
for i in range(50):
    sjp.append([ra(),ra(),ra(),ra(),ra(),ra()])
i=0

print(sjp)

with open("start_joint_pos.txt", 'w') as f:
   writer = csv.writer(f, delimiter=',')
   writer.writerows(sjp)
print('Done')

inc=[]
for i in range(500):
    inc.append([i/10,i/10,i/10])
i=0

print(inc)

with open("inc_pos.txt", 'w') as f:
   writer = csv.writer(f, delimiter=',')
   writer.writerows(inc)
print('Done')



initial_pos=[]
initial_pos.append([0.2,
                    0.5,
                    -1
                    ])


initial_pos.append([0.8,
                    -2.3
                    -1
                    ])


print(initial_pos)

q=helper.read_file("hit_j_log.txt")
q = np.array([-0.6108252804091919,0.579171342638101,1.4726841429611328,0.06362295383193603,-0.505157177615164,-3.5341390234751954e-16])
#q = np.array([0,0,0,0,0,0])
p=helper.read_file("hit_log.txt")
p = np.array([0,0,0])
#print(q)
q = np.squeeze(np.asarray(q))
p = np.squeeze(np.asarray(p))
#print(q)
#print(q[0])
robot = getRobotModel("ur5")
kin = robotKinematics(robot, conf.frame_name)
T_01, T_02, T_03, T_04, T_0e =directKinematic.directKinematics(q)
print("----------------------------------------------------------------------------")
#print(f"{T_01} \t{T_02}\t{T_03}\t{T_04}\t{T_0e}")
print(f"T1:\n{T_01}\n\n"
      f"T2:\n{T_02}\n\n"
      f"T3:\n{T_03}\n\n"
      f"T4:\n{T_04}\n\n"
      f"T0e:\n{T_0e}\n\n")
val=0
#val = directKinematic.numericalInverseKinematics(p[0],q[0])
pos=T_0e*[p[0],p[1],p[2],1]
print(pos)

#ros_pub = RosPub("ur5",node_name="bidone",topic="/command",launch_rviz=False)
#robot = getRobotModel("ur5")
#ros_pub.publish(robot,q)

print("pubblicato")


p= helper.read_file("miss_log.txt")
leng = []
j=0
for i in range(len(p)):
    tmp=p[i][0]**2+p[i][1]**2+p[i][2]**2
    tmp=math.sqrt(tmp)
    #leng.append([])
    leng.append(tmp)
    j+=1

#helper.save(leng,"lenght.txt")
#helper.save_list(leng,"lenght2.txt")

q_i=[-1.9884931599325442,-0.7582790842183948,1.0938652706635112,0.30672678134312403,-0.5872075261595473,-6.375991650382378e-17]
q_f=[-14.911376410896711,-5.413636800274412,4.195594835715124,-0.25605243446128045,1.7937850050115383,0.0
]
path = helper.sin_square(q_i,q_f,50)

print(path)

helper.save(path,"path.txt")

plt.plot(path)
plt.ylabel('test2')
plt.show()



q=helper.read_file("hit_j_log.txt")
p=helper.read_file("hit_log.txt")

robot = getRobotModel("ur5")
kin = robotKinematics(robot, conf.frame_name)
i=0

for i in range(len(q)):

    T_01, T_02, T_03, T_04, T_0e =directKinematic.directKinematics(q[i])

    #p_end=T_0e*[p[0],p[1],p[2],1]
    p_end=T_0e*[0.0,0.0,0.0,1]
    p_end_p=p_end[0:3,3]
    #print(p_end)
    task_diff = p_end_p - p[i]
    #print("p_log:" + str(p[i]))
    #print("p_end: "+ str(p_end))
    diff=np.linalg.norm(task_diff)
    if diff > 0.1:
        print(f"{i}) diff: ", np.linalg.norm(task_diff))



"""

#incrementi su tutti i joints
"""
path=[]
row=[]
min=-12
max=12
bound=(-10,10)



steps=50

for x in range(6):#6 joints
    for i in range(int(steps/2)): #da 0 a max a min a 0
        print(i)
        row = [0.0,0.0,0.0,0.0,0.0,0.0]
        row[x]=min + (i/steps)*(max-min)
        path.append(row)
    for i in range(steps,0,-1): #10 secondi per joint
        print(i)
        row = [0.0,0.0,0.0,0.0,0.0,0.0]
        row[x]=min + (i/steps)*(max-min)
        path.append(row)


helper.save(path,"path_calibration.txt")
"""

pos = np.array([0.089159, 0, 0, 0.10915, 0.09465, 0.0823])
pos = np.array([0.026985191872504866,-0.6681302050626805,-0.3276068811262316,-0.0235015812501439,0.02016366270262715,6.381949412090099e-17])
#pos = np.array([ 0.32509 ,-0.9816,   0.58257, -0.90407, -0.79274,  0.8    ])
#pos = np.array([ 0-0.011396809745719522,-0.058403114301534,0.057916919191157046,0.13126164724779377,-0.11657080834973482,2.59011309487694e-17    ])

T_01, T_02, T_03, T_04, T_0e =directKinematic.directKinematics(pos)

print(T_0e)

robot = getRobotModel("ur5")
kin = robotKinematics(robot, conf.frame_name)
#p= np.array([0.796818,0.181822,0.0215637])
p_end_p=T_0e[0:3,3]
print("end effector position")
print(p_end_p)

q_ik, _, _ = kin.endeffectorInverseKinematicsLineSearch(p_end_p, conf.frame_name,pos)

robot.computeAllTerms(q_ik, np.zeros(6))
p_ik = robot.framePlacement(q_ik, robot.model.getFrameId(conf.frame_name)).translation
task_diff = p_ik - p_end_p
print("Point obtained with IK solution \n", p_ik)
print("Error at the end-effector: \n", np.linalg.norm(task_diff))
print("Final joint positions\n", q_ik)

print("forward of inverse")
_,_,_,_, T =directKinematic.directKinematics(q_ik)
print(T[0:3,3])

print("q_ik")
print(q_ik)
print("q_start")
print(pos)


print("finito")























