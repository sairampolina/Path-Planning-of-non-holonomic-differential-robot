#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 21 12:24:26 2022

@author: sairam
"""


import cv2
import numpy as np
import copy
import heapq as hp
import math


def createObstacles(canvas):
    
    
    clea=10.5  
    # radius of turtlebot
    offset=clea
    
    height,width,_ = canvas.shape
           
    for i in range(width): 
        for j in range(height):
            
            if ((i-300)**2+(j-65)**2-((40+offset)**2))<=0:
                canvas[j][i] = [0,255,0]
                
            if (j+(0.57*i)-(224.285-offset*1.151))>=0 and (j-(0.57*i)+(4.285+offset*1.151))>=0 and (i-(235+offset))<=0 and (j+(0.57*i)-(304.285+offset*1.151))<=0 and (j-(0.57*i)-(75.714+offset*1.151))<=0 and (i-(165-offset))>=0:
                canvas[j][i] = [0,255,0]

            if ((j+(0.316*i)-(76.392-offset*1.048)>=0) and (j+(0.857*i)-(138.571+offset*1.317)<=0) and (j-(0.114*i)-60.909)<=0) or ((j-(3.2*i)+(186+offset*3.352)>=0) and (j-(1.232*i)-(20.652+offset*1.586))<=0 and (j-(0.114*i)-60.909)>=0):
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

if __name__=='__main__':
    
    canvas=np.ones((250,400,3),dtype='uint8')
    canvas=createObstacles(canvas)
     
# =============================================================================
#     cv2.imshow('canvas', canvas)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
# =============================================================================
   
    height,width,_=canvas.shape
    a_height=height-1
    a_width=width-1
    
    start_state,goal_state,rpm_list=getInputs()
    
    # print(start_state,"goal",goal_state,'rpm',rpm_list)
    RPM1=rpm_list[0]
    RPM2=rpm_list[1]
    

    actions=[[0, RPM1],[RPM1, 0],[RPM1, RPM1],[0, RPM2],[RPM2, 0],[RPM2, RPM2],[RPM1, RPM2],[RPM2, RPM1]]