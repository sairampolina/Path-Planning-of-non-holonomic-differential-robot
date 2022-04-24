#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 24 11:45:37 2022

@author: sairam
"""

import cv2
import numpy as np
import copy
import heapq as hp
import math

def createObstacles(canvas):
    
     
    # radius of turtlebot
    offset=11
    
    height,width,_ = canvas.shape
           
    for i in range(width): 
        for j in range(height):
            
            
            #Offset should be given as cm in input
            if ((i-200)**2+(j-800)**2-((offset+100)**2))<=0:
                canvas[j][i] = [0,255,0]

            if ((i-200)**2+(j-200)**2-((offset+100)**2))<=0:
                canvas[j][i] = [0,255,0]

            if (j<=(1000-425)+offset) and (j>=(1000-575)-offset) and (i>=25-offset) and (i<=175+offset):
                canvas[j][i] = [0,255,0]

            if (j<=(1000-425)+offset) and (j>=(1000-575)-offset) and (i>=375-offset) and (i<=625+offset):
                canvas[j][i] = [0,255,0]
                
            if (j<=(1000-200)+offset) and (j>=(1000-400)-offset) and (i>=725-offset) and (i<=875+offset):
                canvas[j][i] = [0,255,0]
                
            
            
    return canvas

start_state=[]
goal_state=[]
rpm_list=[]

def getInputs():
    
    # get valid x,y,theta_s coordinate of start point
    while True:
        
        while True:
            
            value=input("Enter the X- coordinate of Start Point: ")
            
            if float(value)<0 or float(value)>a_width:
                print("Invalid input...Re-enter a valid X-coordinate \n")
                continue
            else:
                start_state.append(float(value))
                break
        while True:
            
            value=input("Enter Y-coordinate of Start Point: ")
            
            if float(value)<0 or float(value)>a_height:
                print("Invalid Input...Re-enter a valid Y-coordinate\n")
                continue
            else:
                start_state.append(float(value))
                break
        
        while True:
            
            value=input("Enter Orientation of Robot at Start Point(+ve in counter-clockwise):\n")
            
            start_state.append(float(value))
            break
        
        
        if (canvas[a_height-int(start_state[1])][int(start_state[0])][1]==255):
            print("***Start Point Entered is inside Obstacle Space..Retry***")
            start_state.clear()
            continue
        else:
            break
    
    # get valid x,y,theta_g coordinate of goal node
    while True:
        
        while True:
            
            value=input("Enter the X- coordinate of Goal Point: ")
            
            if float(value)<0 or float(value)>a_width:
                print("Invalid input...Re-enter a valid X-coordinate \n")
                
                continue
            else:
                goal_state.append(float(value))
                break
        
        while True:
            
            value=input("Enter Y-coordinate of Goal Point: ")
            
            if float(value)<0 or float(value)>a_height:
                print("Invalid Input...Re-enter a valid Y-coordinate\n")
                continue
            else:
                goal_state.append(float(value))
                break
        
        
        if canvas[a_height-int(goal_state[1])][int(goal_state[0])][1]==255:
            print("***Goal Point Entered is inside Obstacle Space..Retry***")
            goal_state.clear()
            continue
        else:
            lrpm=input('Enter Left wheel Rpm: ')
            rrpm=input('Enter Right wheel Rpm: ')
            rpm_list.append(int(lrpm))
            rpm_list.append(int(rrpm))
            
            break

    
    return start_state,goal_state,rpm_list


def isItDuplicatenode(p,q,r,V):
    
    if (p>999 or q>999):
        return True
     
    p=round(p*2)/2
    q=round(q*2)/2
    
    if r>=360:
        r=(r%360)
   
    # print(p,q,r)
   
    if V[int(2*p)][int(2*q)][int(r//30)]==0:
        
        V[int(2*p)][int(2*q)][int(r//30)]=1
        return False
    else:
        return True

def generateNode(node,UL,UR,canvas):
    
    next_node=copy.deepcopy(node)
    
    # r = 0.033
    # L = 0.178
    
    r=3.3
    L=17.8
# =============================================================================
#     r=0.09
#     L=0.7
# =============================================================================
    
    dt = 0.2
    D=0
    X_i=next_node[0]
    Y_i=next_node[1]
    Thetai=next_node[2]
    

    Thetan =(3.14 * Thetai) / 180
    
    node_steps_list=[]
    
    node_steps_list.append([X_i,Y_i,Thetai])
    
    for i in range(0,10,1):
        
        Delta_Xn = 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Delta_Yn = 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Thetan = (r / L) * (UR - UL) * dt
        D=D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
        
        Xd=X_i+Delta_Xn
        Yd=Y_i+Delta_Yn
        Thetad=Thetai+(180*(Thetan))/3.14
        
        Thetan=(3.14 * Thetad) / 180
       
        # if angle is negative, change angle to (counter clock wise angle)
        if (Thetad<0):
            Thetad=360+Thetai
        else:
            Thetad=Thetad
       
        if (Thetad>360):
            Thetad=(Thetad%360)
       
        X_i=Xd
        Y_i=Yd
        Thetai=Thetad
        
        
        if ((Xd<=0 or Xd>=a_width) or (Yd<=0 or Yd>=a_height)):
            node_steps_list.clear()
            return False,next_node,node_steps_list,D
        else:
            if canvas[int(Yd)][int(Xd)][1]==255 :
                node_steps_list.clear()
                return False,next_node,node_steps_list,D
            else:
                # Thetan = (180 * Thetan) / 3.14
                node_steps_list.append([Xd,Yd,Thetad])
    
    print(node_steps_list)
    print('/n************************************')
        
    x_f,y_f,theta_f=node_steps_list[-1]
    
    if ((x_f>=0 and x_f<=a_width) and 
        (y_f>=0 and y_f<=a_height) and 
        (canvas[int(y_f)][int(x_f)][1] != 255) and 
        (not isItDuplicatenode(x_f,y_f,theta_f,V))):

        next_node[0],next_node[1],next_node[2]=node_steps_list[-1]   
        
        return True,next_node,node_steps_list,D 
    else:
        return False,next_node,node_steps_list,D

def c2g(coordinates1,goal_state):
    point1=np.array(copy.deepcopy(coordinates1[0:2]))
    goal_point=np.array(copy.deepcopy(goal_state[0:2]))
    
    dis=np.linalg.norm(point1-goal_point)
    
    return int(dis)       


def AStar(start_state,goal_state,canvas,UL,UR):
    open_list = []
    # closed_list key(present_node):value(parent_node)
    closed_list = {} 
    back_track_flag = False
    hp.heapify(open_list)
    c2g_i=c2g(start_state,goal_state)
    hp.heappush(open_list,[c2g_i,0,start_state,start_state])
    # 0: Total-cost,1:cost to come,2:parent node, 3: present node
    
    path_dict={}
    
    while(len(open_list)>0):
        
        # pop node from open list
        node=hp.heappop(open_list)
        
        
        # adding popped node to closed-list
        closed_list[(node[3][0],node[3][1],node[3][2])]=node[2]
       
        # check if popped node is goal node
        # if it is goal start back tracking
        p1=np.array(goal_state[0:2])
        p2=np.array(node[3][0:2])
        distance=np.linalg.norm(p1-p2)
        l=5*12
        
        if (distance**2)<=(l**2):
            back_track_flag=True
            apprx_goal=node[3]
            print(apprx_goal)
            print('************Started BackTracking**************\n')
            break
        
        del p1
        del p2
        
        
        
        
        actions=[[0, UL],[UL, 0],[UL, UL],[0, UR],[UR, 0],[UR, UR],[UL, UR],[UR, UL]]
        
      
        
        for action in actions:
            
            flag,next_node,node_steps_list,D=generateNode(node[3],action[0],action[1], canvas)
            
            present_c2c=node[1]
        

            if(flag):
                
                present_c2g=c2g(next_node,goal_state)
                
                # consider child, if it is not in closed-list
                
                if tuple(next_node) not in closed_list:
                    
                    # calculate c2c and total-cost
                    new_c2c=present_c2c+D
                    Total_cost=new_c2c+present_c2g
                    
                    
                    # if child is not in open_list add child with its  Total cost,c2c, and parent node
                    temp_list=[]
                    
                    for i in range(len(open_list)):
                        temp_list.append(open_list[i][3])
                   
                    if next_node not in temp_list:
                        hp.heappush(open_list,[Total_cost,new_c2c, node[3],next_node])
                        hp.heapify(open_list)
                        path_dict[(next_node[0],next_node[1],next_node[2])]=node_steps_list
                    
                    # if it is in open-list update costs and parent-node
                    else:
                        idx=temp_list.index(next_node) 
                        if(Total_cost<open_list[idx][0]): # Updating the cost and parent node for the child generated
                               open_list[idx][0] = Total_cost
                               open_list[idx][1] = new_c2c
                               open_list[idx][2] = node[3]
                               hp.heapify(open_list)
                               path_dict[(next_node[0],next_node[1],next_node[2])]=node_steps_list
                    temp_list.clear()
            
                

        hp.heapify(open_list)     

    if(back_track_flag):
        #Call the backtrack function
        backTrack(start_state,apprx_goal,closed_list,canvas,path_dict)
    
    else:
        print("Solution Cannot Be Found")


def backTrack(start_state,apprx_goal,closed_list,canvas,path_dict):
    
    # video_writer = cv2.VideoWriter_fourcc(*'XVID')
    # out = cv2.VideoWriter('Planning_exploration.avi',video_writer,1000,(400,250))
    
    # plotting explored vectors
    for key in path_dict.keys():
        
        sub_list=copy.deepcopy(path_dict[key])
        
       
        
        for i in range(len(sub_list)-1):
            
            list1=sub_list[i]
            list2=sub_list[i+1]
            
            X=list1[0]
            Y=list1[1]
            
            X=int(round(X))
            Y=int(round(Y))
            

            
            U=list2[0]
            V=list2[1]
            
            U=int(round(U))
            V=int(round(V))
            
            
            cv2.line(canvas,(X,Y),(U,V),(255,0,0),1)
            cv2.imshow('exploring', canvas)
            # out.write(canvas)
            cv2.waitKey(1)
    
    
    # plotting optimal path
    optimal_path=[]
    
    f_list=path_dict[tuple(apprx_goal)]
    f_list.reverse()
    
    for i in range(len(f_list)):
        optimal_path.append(f_list[i])
        
   
    
    parent=f_list[-1]
    
    
    n_list=[]
    
    while(parent!=start_state): 
        n_list=path_dict[tuple(parent)]
        n_list.reverse()
        for i in range(len(n_list)):
            optimal_path.append(n_list[i])
        parent=n_list[-1]
    
    optimal_path.append(start_state)
    print('Optimal Path generated is: \n ',optimal_path)
    
    
    for i in range(len(optimal_path)-1):
        
        list1=optimal_path[i]
        list2=optimal_path[i+1]
        
    
        X=list1[0]
        Y=list1[1]
        
        X=int(round(X))
        Y=int(round(Y))
        
        U=list2[0]
        V=list2[1]
        
        U=int(round(U))
        V=int(round(V))
        
        
        cv2.line(canvas,(X,Y),(U,V),(0,0,255),2)
        cv2.imshow('exploring', canvas)
        # out.write(canvas)
        cv2.waitKey(1)
    
    # out.release()
    print('reached goal')
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__=='__main__':
    
    canvas=np.ones((1000,1000,3),dtype='uint8')
    canvas=createObstacles(canvas)
    cv2.imshow('canvas', canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    height,width,_=canvas.shape
    a_height=height-1
    a_width=width-1
    
    start_state,goal_state,rpm_list=getInputs()
    
    # changing world co-ordinates to map-coordinates
    start_state[1]=a_height-start_state[1]
    goal_state[1]=a_height-goal_state[1]
    
    # print(start_state,"goal",goal_state,'rpm',rpm_list)
    
    UL=rpm_list[0]
    UR=rpm_list[1]
    
    # creating matrix V
    
    V=np.zeros((2000,2000,12),dtype='uint8')
    
    
    # generateNode(start_state,UL,UR,canvas)
    
    AStar(start_state,goal_state,canvas,UL,UR)
        
    
    