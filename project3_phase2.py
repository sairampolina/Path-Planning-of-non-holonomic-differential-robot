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
    
    # Rad=int(input("Enter Robot radius: "))
    clea=int(input("Enter clearence to be maintained for Robot: "))
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

if __name__=='__main__':
    
    canvas=np.ones((250,400,3),dtype='uint8')
    canvas=createObstacles(canvas)