#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 10 13:39:04 2019

@author: leena
"""
import numpy as np
     
    
maze = [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
       0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]

#reshape list as array 20*18
my_map=np.asarray(maze).reshape(20,18)
#print(my_map)

#get neighbours for a particular node- excluding nodes outside edges and wall nodes
#will return  dictionary if list with values [node point,cost(1, if up,down,left,right and 1.14(root 2) if diagonal neighbour)]
def get_neighbours(i,j):
    final_dict={}
    diagonal=round(np.sqrt(2), 2)
    
    if not(i-1<0 or my_map[i-1][j]==1):
        final_dict["n2"]=[[i-1,j],1]
    if not(j-1<0 or my_map[i][j-1]==1):
        final_dict["n4"]=[[i,j-1],1]
    if not(j+1==len(my_map[0]) or my_map[i][j+1]==1):
        final_dict["n5"]=[[i,j+1],1]
    if not(i+1==len(my_map) or my_map[i+1][j]==1):
        final_dict["n7"]=[[i+1,j],1]
              
    if not(i-1<0 or j-1<0 or my_map[i-1][j-1]==1 or (final_dict.get("n2")==None and final_dict.get("n4")==None)):
        final_dict["n1"]=[[i-1,j-1],diagonal]
    if not(i-1<0 or j+1==len(my_map[0]) or my_map[i-1][j+1]==1 or (final_dict.get("n2")==None and final_dict.get("n5")==None)):
        final_dict["n3"]=[[i-1,j+1],diagonal]
    if not(i+1==len(my_map) or j-1<0 or my_map[i+1][j-1]==1 or (final_dict.get("n7")==None and final_dict.get("n4")==None)):
        final_dict["n6"]=[[i+1,j-1],diagonal]
    if not(j+1==len(my_map[0]) or i+1==len(my_map) or my_map[i+1][j+1]==1 or (final_dict.get("n5")==None and final_dict.get("n7")==None)):
            final_dict["n8"]=[[i+1,j+1],diagonal]
    return final_dict
    
#n1,n2,n3,n4,n5,n6,n7,n8=get_neighbours(3,4)
#print(n1,n2,n3,n4,n5,n6,n7,n8)

#initialize start and goal nodes
# start = [11, 0]
# end = [0, 13]

#get euclidean distance between input node and goal - heuristic function - h(n)
def get_h(start_node,goal_node):
    x_diff=np.square(start_node[0]-goal_node[0])
    y_diff=np.square(start_node[1]-goal_node[1])
    distance=np.sqrt(y_diff+x_diff)
    return round(distance, 2)

#after reaching goal, get parent elements from the parent's graph 
def backpropogate(parent_map,current):
    total_path=[current]
    current = parent_map[current[0]][current[1]]
    total_path.append(current)
    #print(total_path)
    while current in np.reshape(parent_map, 360).tolist() and current != None:
        current = parent_map[current[0]][current[1]]
        total_path.append(current)
    return total_path[::-1]

#a-star implementation
def impl_astar(start,end):
    #print("start,end: ",start,end)
    #map (dimension 20*18) to store g(n) values for all nodes
    #initialized to infinity
    g_map=np.full((len(my_map),len(my_map[0])),np.inf)
    #map (dimension 20*18) to store h(n) values for all nodes
    #initialized to infinity
    h_map=np.full((len(my_map),len(my_map[0])),np.inf)
    #map (dimension 20*18) to store position values of parent for all nodes
    #initialized to empty 
    #elements datatype is set as list
    parent_map=np.empty((len(my_map),len(my_map[0])),dtype=list)
    #initilize list to store visited nodes
    closedset=[]   
    #initialize list to store nodes which are not visited yet
    openset_list=[]
    
    #at first step, store start values [g=0,h=eucildean distance between start and end, append start node to openlist]
    g_map[start[0],start[1]]=0
    h_map[start[0],start[1]]=get_h(start,end)
    openset_list.append(start)
    #print(openset_dict)

    #a-star loop
    while len(openset_list) != 0:
        #get node with least cost
        min_node_cost=9999999
        min_node=0
        for node in openset_list:
            if(min_node_cost>h_map[node[0],node[1]]):
                min_node_cost=h_map[node[0],node[1]]
                min_node=node
        
        #check if goal is reached, if yes, then backpropogate
        if min_node == end:
            return backpropogate(parent_map, min_node)

        #pop openlist and add the node to visited list
        #print("min_node: ",min_node)  
        openset_list.remove(min_node)
        closedset.append(min_node)
        
        #get neighbours
        n_dict=get_neighbours(min_node[0],min_node[1])
        #check each neighbour
        for nb,nb_val in n_dict.items():
            #print(nb_val)
            #ignore neighbour if already visited
            if nb_val[0] in closedset:
                continue
            #calculte f(n)
            nb_g=min_node_cost+nb_val[1]+get_h(nb_val[0],end)
            #print(nb_g)
            #compare old and new f(n), if new is smaller, update to g_map,h_map and add parent as the min_node, add the neihbour to openlist
            if(nb_g<h_map[nb_val[0][0],nb_val[0][1]]):
                parent_map[nb_val[0][0],nb_val[0][1]]=[min_node[0],min_node[1]]
                g_map[nb_val[0][0],nb_val[0][1]]=nb_g
                h_map[nb_val[0][0],nb_val[0][1]]=nb_g+get_h(nb_val[0],end)
                if(nb_val[0] not in openset_list):
                    openset_list.append(nb_val[0])
            #print(g_map)
        #print(parent_map)

