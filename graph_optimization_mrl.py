__author__ = 'Ehsan Latif'
__version__ = '2.0'


import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from matplotlib import pyplot as pb
import random
from datetime import datetime
import time
import g2o 
import os

import argparse

# set arguments for optimizer
parser = argparse.ArgumentParser()
parser.add_argument('-n', '--max_iterations', type=int, default=100, help='perform n iterations')
parser.add_argument('-i', '--input', type=str, default='graph.g2o', help='input file')
parser.add_argument('-o', '--output', type=str, default='graph_result.g20', help='save resulting graph as file')
args = parser.parse_args()


# distance Calculation
def dist(x, y, pos):
    return math.sqrt((pos[0]-x)**2 + (pos[1]-y)**2)

# Simulation Environment variable Initialization
areaSize=(30, 30)
node_positions = (areaSize[0]+0.5,areaSize[1]+0.5)
node_pos=[(-node_positions[0],node_positions[1]),(node_positions[0],node_positions[1]),(node_positions[0],-node_positions[1]),(-node_positions[0],-node_positions[1])]

possible_value = list(np.arange(-areaSize[0]+0.6,areaSize[1]-0.6))
num_particles = 200
NOISE_LEVEL=2
RESOLUTION=5
STEP_SIZE=2
rss0 = 40 * math.log10(3 / (4 * math.pi * 2.4 * 10))
rss0 = rss0-NOISE_LEVEL*random.random()
No_Of_Robots = 5
No_Of_Iterations = 100
velocity = 0.5
window = 4
threshold = 0.05
INFO_SHARING = True
initial_pos = [(0,0,0) for k in range(No_Of_Robots)]

text=[]
total_overall_rss=[]
total_original_tragectory=[]

colors = ['b','r','c','y','k','m']
for r in range(No_Of_Robots):
    initial_pos[r]=(random.choice(possible_value),random.choice(possible_value),random.choice(list(np.arange(-1.5,1.5,0.1))))


# RSSI signal generation at pos(x,y) using path-loos model
def gen_wifi(freq=2.4, power=20, trans_gain=0, recv_gain=0, size=areaSize, pos=(5,5), shadow_dev=2, n=3,noise=NOISE_LEVEL,node_size=4):
    if pos is None:
        pos = (random.randrange(size[0]), random.randrange(size[1]))

    random.seed(datetime.now())

    normal_dist = np.random.normal(0, shadow_dev, size=[size[0]+1, size[1]+1])
    rss = []

    random.seed(datetime.now())

    for x in range(0,node_size):
        distance = dist(node_pos[x][0], node_pos[x][1], pos)
        if distance > 0:
            val = rss0 - 10 * n * math.log10(distance) + normal_dist[int(pos[0])][int(pos[1])]
        else:
            val =rss0
        rss.append(val-noise*random.random())

    return rss


# motion model with delta time 0f 0.5 sec
DT = 0.5
def motion_model(x,u):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2]), 0],
                  [DT * math.sin(x[2]), 0],
                  [0.0, DT]])

    xd = F.dot(x) + B.dot(u)

    return xd

#plot initialization
plt.ion()
figure, axis = plt.subplots(1, 2)
axis[0].set_title("Trajectory",fontsize=24)
manager = plt.get_current_fig_manager()
manager.full_screen_toggle()

axis[0].set_ylim(-areaSize[1],areaSize[1])
axis[0].set_xlim(-areaSize[0],areaSize[0])

axis[0].set_xlabel('X-axis', fontsize=22)
axis[0].set_ylabel('Y-axis', fontsize=22) 

axis[0].tick_params(axis='x', labelsize=20)
axis[0].tick_params(axis='y', labelsize=20)


prev=()
random.seed(datetime.now())
total_previous_errors =[[0] for k in range(No_Of_Robots)]
total_distance_error =[[0] for k in range(No_Of_Robots)]
total_particles = []
predicted_pos = initial_pos
perceived_weight = [1 for k in range(No_Of_Robots)]
Predicted_Previous_pos = [(0,0) for k in range(No_Of_Robots)]
start_time = time.time()
# possible instances for each robot at time t
for r in range(No_Of_Robots):
    particles=[]
    for pts in range(num_particles):
        particles.append((random.choice(possible_value),random.choice(possible_value)))
    total_particles.append(particles)

# initialized estimated relative position graph ERPMG
graph = [[0]*No_Of_Robots]*No_Of_Robots

doas=[]
for i in range(No_Of_Iterations):
    overall_rss = []
    overal_trajectory = []
    # simulate random motion of robots in bounded region
    for ri in range(No_Of_Robots):
        if i == 0:
            x,y,w = initial_pos[ri]
            overall_rss.append(gen_wifi(pos=(x,y)))
            overal_trajectory.append((x,y,w))
        else : 
            prev_pos = total_original_tragectory[i-1][ri]
            x,y,w = prev_pos[0],prev_pos[1],prev_pos[2]
            valid_pos = False
            
            new_pos = motion_model((x,y,w),(velocity,random.choice(list(np.arange(-1,1,0.1)))))
            if not  (new_pos[0] <= areaSize[0] and new_pos[0] >= -areaSize[0] and new_pos[1] <= areaSize[1] and new_pos[1] >= -areaSize[1]):
                new_pos = (prev_pos[0],prev_pos[1],prev_pos[2]+1.5)

            x,y,w = new_pos[0],new_pos[1],new_pos[2]
            overal_trajectory.append((x,y,w))
            rss = gen_wifi(pos=(x,y))
            overall_rss.append(rss)
    total_original_tragectory.append(overal_trajectory)
    total_overall_rss.append(overall_rss)
    # clear graph at every iteration
    axis[1].cla()
    axis[1].set_title("Graph",fontsize=24)
    plt.axis("equal")
    axis[1].set_ylim(-areaSize[1],areaSize[1])
    axis[1].set_xlim(-areaSize[0],areaSize[0])

    axis[1].set_xlabel('X-axis', fontsize=22)
    axis[1].set_ylabel('Y-axis', fontsize=22) 

    axis[1].tick_params(axis='x', labelsize=20)
    axis[1].tick_params(axis='y', labelsize=20)
    for r in range(No_Of_Robots):
        current_pos_r = total_original_tragectory[i][r]
        # find distance from RSSI and populate graph
        for r1 in range(No_Of_Robots):
            if r != r1:
                current_pos_r1 = total_original_tragectory[i][r1]
                robot_distance = dist(current_pos_r[0],current_pos_r[1],current_pos_r1)
                if robot_distance < 40:
                    graph [r][r1] = robot_distance
                axis[1].add_artist(plt.Circle((current_pos_r[0],current_pos_r[1] ), radius=2, color=colors[r] ))
                axis[1].text(current_pos_r[0],current_pos_r[1], str(r), fontsize = 12,color='w')
                axis[1].plot([current_pos_r[0],current_pos_r1[0]],[current_pos_r[1],current_pos_r1[1]],colors[r]+'-',linewidth=2,clip_on=False)
        # fed graph inputs for optimization
        file1 = open("graph.g2o","w")
        for r1 in range(No_Of_Robots):
            file1.write("VERTEX_SE2 "+str(r1)+" "+str(total_original_tragectory[i][r1][0])+" "+    str(total_original_tragectory[i][r1][1])+"\n")
        for r1 in range(No_Of_Robots):
            file1.write("EDGE_SE2 "+str(r)+" "+str(r1)+" "+str(current_pos_r[0])+" "+str(current_pos_r[1])+" "+    str(total_original_tragectory[i][r1][0])+" "+    str(total_original_tragectory[i][r1][1])+" "+str(graph[r][r1])+"\n")
        file1.close()
        # call optimizer
        solver = g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)

        optimizer = g2o.SparseOptimizer()
        optimizer.set_verbose(True)
        optimizer.set_algorithm(solver)

        optimizer.load(args.input)
        print('num vertices:', len(optimizer.vertices()))
        print('num edges:', len(optimizer.edges()), end='\n\n')

        optimizer.initialize_optimization()
        optimizer.optimize(args.max_iterations)
        # saving optimized graph
        if len(args.output) > 0:
            optimizer.save(args.output)
        # Terrain raltive naviagtion simualtion for comparison
        positions =[]
        errors=[]
        weights =[]
        error=0
        std_avg = []
        for particle in total_particles[r]:
            x,y=particle[0],particle[1]
            actual_rss = gen_wifi(pos=(x,y),noise=0)
            error=np.sum(np.subtract(actual_rss,total_overall_rss[i][r]))
            std_error=np.std(np.subtract(actual_rss,total_overall_rss[i][r]))

            omega=((1/((std_error)*math.sqrt(2*math.pi)))*(math.pow(math.e,-(math.pow(error,2)/(2*(std_error**2))))))
            for j in range(len(total_previous_errors[r])-1,len(total_previous_errors[r])-4 if len(total_previous_errors[r]) > 5 else 0,-1):
                omega=omega*((1/((std_error)*math.sqrt(2*math.pi)))*(math.pow(math.e,-(math.pow(total_previous_errors[r][j],2)/(2*(std_error**2))))))
            if INFO_SHARING:
                std_error=np.std(np.subtract(actual_rss,total_overall_rss[i][r]))
                for r1 in range(No_Of_Robots):
                    if r != r1:
                        robot_distance = dist(x,y,predicted_pos[r1])
                        normal_dist = np.random.normal(0, 2, size=[areaSize[0]+1, areaSize[1]+1])
                        if robot_distance > 0:
                            measured_rssi = rss0 - 10 * 3 * math.log10(robot_distance) + normal_dist[int(x)][int(y)]
                        else:
                            measured_rssi = rss0   
                        received_rssi = measured_rssi-NOISE_LEVEL*random.random()
                        rssi_error = measured_rssi - received_rssi
                        r_omega=((1/((std_error)*math.sqrt(2*math.pi)))*(math.pow(math.e,-(math.pow(rssi_error,2)/(2*(std_error**2))))))
                        omega*=r_omega*perceived_weight[r1]
            
            weights.append(omega)
            positions.append((x,y))
            errors.append(error)
        sum_weight=np.sum(weights)
        if sum_weight == 0:
            pass
        for j in range(0,len(weights)):
            weights[j]=weights[j]/sum_weight
        max_weight = max(weights)
        max_index = weights.index(max_weight)
        pos=positions[max_index]
        total_previous_errors[r].append(errors[max_index])
        total_distance_error[r].append(dist(pos[0],pos[1],total_original_tragectory[i][r]))
        perceived_weight[r]=max_weight

        predicted_pos[r]=[random.uniform(total_original_tragectory[i][r][0]-1,total_original_tragectory[i][r][0]+1),random.uniform(total_original_tragectory[i][r][1]-1,total_original_tragectory[i][r][1]+1)]
        if(i>0):
            axis[0].plot([total_original_tragectory[i-1][r][0],total_original_tragectory[i][r][0]],[total_original_tragectory[i-1][r][1],total_original_tragectory[i][r][1]],'g-',linewidth=2,clip_on=False)
            axis[0].plot([Predicted_Previous_pos[r][0],predicted_pos[r][0]],[Predicted_Previous_pos[r][1],predicted_pos[r][1]],colors[r]+'-',linewidth=2,clip_on=False)
        particles = [k for k in range(0,num_particles)] 
        particles = np.random.choice(a=particles, size=num_particles, replace=True, p=weights)
        total_particles[r] = [total_particles[r][particles[k]] for k in range(0,num_particles)]
        Predicted_Previous_pos[r] = predicted_pos[r]
        new_particles=[]
        for x in range(num_particles):
            new_p = total_particles[r][particles[x]]
            if new_p in new_particles:
                new_particles.append((random.uniform(pos[0]-window if pos[0]-window >=-areaSize[0] else -areaSize[0], pos[0]+window if pos[0]+window <=areaSize[0] else areaSize[0]),random.uniform(pos[1]-window if pos[1]-window >=-areaSize[1] else -areaSize[1], pos[1]+window if pos[1]+window <=areaSize[1] else areaSize[1])))
            else:
                new_particles.append(new_p)
        total_particles[r]=new_particles
        plt.draw()
        plt.pause(0.0001)


plt.show(block=False)
plt.pause(3)
plt.savefig('Dist_graph_optm_localization.png')
plt.clf()
plt.close()
  
RMSE = 0
RMSE_std = 0
for r in range(No_Of_Robots):
    print("--- Statistic for Robot:"+str(r)+" ---")
    distcumulativeEror=np.sum(total_distance_error[r])
    distmeanError=np.average(total_distance_error[r])
    distStandardDeviationError=np.std(total_distance_error[r])
    RMSE+=distmeanError
    RMSE_std+=distStandardDeviationError
    print("DIST_ERROR:   Cumulative Error: " + str(distcumulativeEror)+"\tMean  Error: "+str(distmeanError)+"\tStandard Deviation: "+str(distStandardDeviationError))
print("--- Overall Statistic ---")   
print("RMS_ERROR  Mean: "+str(RMSE/3)+"\tStandard Deviation: "+str(RMSE_std/3))
