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
    
    
    clea=5 
    # radius of turtlebot
    offset=10
    
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


if __name__=='__main__':
    
    canvas=np.ones((1000,1000,3),dtype='uint8')
    canvas=createObstacles(canvas)
    cv2.imshow('canvas', canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    
    