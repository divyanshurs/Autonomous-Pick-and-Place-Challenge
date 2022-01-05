import numpy as np
import random
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from copy import deepcopy
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from lib.calculateFK import FK


class Node: #a class so that we can keep a track of the current node configuration and the parent index
    def __init__(self):
        self.q=np.zeros(7)
        self.parent = None

def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (0x7).
    :param goal:        goal pose of the robot (0x7).
    :return:            returns an mx7 matrix, where each row consists of the configuration of the Panda at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is empty
    """

    # initialize path
    path = []
    fk = FK()
    q_start = Node()
    q_start.q = start
    goal_reached = False
    segments = 1 #no of segments to considered
    segment_length = 2 #length of each segment
    tree_nodes = []
    tree_nodes.append(q_start) #first point in the tree is the start point



    delta = 0.015#inflation param
    n_div = 20 #col detection fragment param
    tolerance = 1.5
    max_iterations = 100000


    obstacles_non_inflated= map[0]
    inflated_obstacles=[0]*(len(obstacles_non_inflated))
    for id,obstacles in enumerate(obstacles_non_inflated):
        print(id)
        inflated_obstacles[id] = list(np.add(obstacles,[-delta,-delta,-delta,delta,delta,delta]))

    print(inflated_obstacles)
    obstacles_non_inflated = inflated_obstacles
    lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])
    print("inflated_obs")



    iterations=0
    print("inflated_obs")


    while goal_reached != True and iterations <= max_iterations:
        # print(iterations)
        q_sampled = np.random.uniform(low = lowerLim, high = upperLim) #check for collision once here

        #nearest cong finding
        node_config_list=[node.q for node in tree_nodes]
        distance = np.linalg.norm(q_sampled-np.asarray(node_config_list), axis = 1)
        nearest_vertex_idx = np.argsort(distance)[0]
        nearest_vertex_config = node_config_list[nearest_vertex_idx]
        # unit_vec = (q_sampled - nearest_vertex_config)/  np.linalg.norm(q_sampled - nearest_vertex_config)
        # q_new = nearest_vertex_config + (segment_length)* unit_vec
        q_new = q_sampled


        for i in range(n_div):
            q_start = nearest_vertex_config + i*(q_new - nearest_vertex_config)/n_div
            q_stop = nearest_vertex_config + (i+1)*(q_new - nearest_vertex_config)/n_div
            jointPositions_start, _ = fk.forward(q_start)
            jointPositions_stop, _ = fk.forward(q_stop)
            collision_flag = False
            for box in obstacles_non_inflated:
                collision_list = detectCollision(jointPositions_start, jointPositions_stop, box)
                if(np.sum(collision_list) >0):
                    collision_flag = True
                    break
            if(collision_flag == True):
                break

        if(collision_flag == True):
            iterations=iterations+1
            continue

        #q_new = nearest_vertex_config + (segment_length)* unit_vec
        #q_new = q_sampled

        newNode = Node()
        newNode.q=q_new
        newNode.parent = nearest_vertex_idx
        #print(iterations)

        tree_nodes.append(newNode)



        #GOAL KA CONDITION VERIFY KARO
        if  np.linalg.norm(q_new - goal) < tolerance:
            collision_flag = False
            for i in range(n_div):
                q_start = q_new + i*(goal - q_new)/n_div
                q_stop = q_new + (i+1)*(goal - q_new)/n_div
                jointPositions_start, _ = fk.forward(q_start)
                jointPositions_stop, _ = fk.forward(q_stop)
                collision_flag = False
                for box in obstacles_non_inflated:
                    collision_list = detectCollision(jointPositions_start, jointPositions_stop, box)
                    if(np.sum(collision_list) >0):
                        collision_flag = True
                        break
                if(collision_flag == True):
                    break

            if(collision_flag==False):
                last_vertex_idx= len(tree_nodes)-1
                goalNode = Node()
                goalNode.q = goal
                goalNode.parent = last_vertex_idx
                tree_nodes.append(goalNode)
                goal_reached = True
                print("goal reached")



        #DEBUGGING KE LIYEEE
        if(iterations%500 == 0):
            node_config_list=[node.q for node in tree_nodes]
            distance = np.linalg.norm(goal-np.asarray(node_config_list), axis = 1)
            nearest_vertex_idx = np.argsort(distance)[0]
            nearest_vertex_config = node_config_list[nearest_vertex_idx]
            print(distance[nearest_vertex_idx])
            print(nearest_vertex_config)
            #print(iterations)

        iterations=iterations+1 #END WHILE


    if(goal_reached==True):
        print("len(tree_nodes)")
        print(len(tree_nodes))
        i = len(tree_nodes)-1; #index of last node in the tree (q_goal)
        path.append(tree_nodes[i].q)
        while True:
            #print("i")
            #print(i)
            i = tree_nodes[i].parent; #index of parent node of current evaluated node
            if i == 0: #when i==0 means the start goal has been reached
                path.append(tree_nodes[i].q) #Adds q_start
                #print("BREAKKKKK")
                break
            path.append(tree_nodes[i].q)
            # print("path")
            # print(np.array(path).shape)
        path.reverse()
        #path = np.array(path)
        print("path")
        print(np.array(path).shape)
    else:
        print("No Path found")
        path = None
    return path

if __name__ == '__main__':
    map_struct = loadmap("../maps/map2.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
