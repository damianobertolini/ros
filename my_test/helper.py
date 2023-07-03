import math
import csv
import numpy

def linear(q_i,q_f,steps):
    path=[]

    for i in range (0,steps):
        path.append(q_i +(q_f-q_i) * (i/steps))
    return path

def parab(q_i,q_f,steps=300):
    #y=-x**2+2x

    q_i = numpy.asarray(q_i)
    q_f = numpy.asarray(q_f)

    diff = q_f - q_i
    b = 2 * diff
    a = -0.5
    path=[]
    for i in range (0,steps):
        path.append(q_i +(diff) *  increment_parab(diff/steps,steps,a,b))
    return path

def increment_parab(i,steps,a,b):
    #return i/steps #lineare
    return (a*i**2+b*i)

def sin_square(q_i,q_f,steps):
    path=[]
    q_i = numpy.asarray(q_i)
    q_f = numpy.asarray(q_f)
    diff = q_f - q_i
    for i in range(steps):
        path.append(
            q_i+ (math.sin((math.pi/2)*i/steps)**2)*diff
                    )
    return path

def get_initial_pos():
    initial_pos2 = []
    with open("start_pos.txt", 'r') as file:
        csvreader = csv.reader(file)
        for row in csvreader:
            initial_pos2.append(row)

    t = len(initial_pos2[0])
    initial_pos = [[0 for i in range(t)] for j in range(len(initial_pos2))]

    for i in range(len(initial_pos2)):
        for j in range(len(initial_pos2[0])):
            initial_pos[i][j] = float(initial_pos2[i][j])
    return  initial_pos

def get_initial_joint_pos():
    initial_joint_pos2 = []
    with open("start_joint_pos.txt", 'r') as file:
        csvreader = csv.reader(file)
        for row in csvreader:
            initial_joint_pos2.append(row)

    t = len(initial_joint_pos2[0])
    initial_joint_pos = [[0 for i in range(t)] for j in range(len(initial_joint_pos2))]

    for i in range(len(initial_joint_pos2)):
        for j in range(len(initial_joint_pos2[0])):
            initial_joint_pos[i][j] = float(initial_joint_pos2[i][j])
    return  initial_joint_pos

def get_inc_pos():
    initial_pos2 = []
    with open("inc_pos.txt", 'r') as file:
        csvreader = csv.reader(file)
        for row in csvreader:
            initial_pos2.append(row)

    t = len(initial_pos2[0])
    initial_pos = [[0 for i in range(t)] for j in range(len(initial_pos2))]

    for i in range(len(initial_pos2)):
        for j in range(len(initial_pos2[0])):
            initial_pos[i][j] = float(initial_pos2[i][j])
    return  initial_pos

def read_file(name):
    matrix2 = []
    with open(name, 'r') as file:
        csvreader = csv.reader(file)
        for row in csvreader:
            matrix2.append(row)

    t = len(matrix2[0])
    matrix = [[0 for i in range(t)] for j in range(len(matrix2))]

    for i in range(len(matrix2)):
        for j in range(len(matrix2[0])):
            matrix[i][j] = float(matrix2[i][j])
    return matrix


    return matrix


def save(matrix,name="placeholder.txt"):
    with open(name, 'w') as f:
        writer = csv.writer(f, delimiter=',')
        writer.writerows(matrix)
    print('Done')

def save_list(my_list,file_name="list.txt"):
    f = open(file_name,"w")
    for i in range(len(my_list)):
        f.write(str(my_list[i]))
        f.write("\n")

def lenght(p):
    return p[0]**2+p[1]**2+p[2]**2
