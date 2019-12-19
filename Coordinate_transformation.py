import numpy as np
import math
def Coordinate_transformation(points,a,b,c,d):
    pts = np.array(points)
    pts_avg = np.mean(pts,axis=0)
    z_1 = np.array([a,b,c])
    x_1 = pts[0,:] - pts_avg
    sqrtsumx_1 = math.sqrt(x_1[0]*x_1[0]+x_1[1]*x_1[1]+x_1[2]*x_1[2])
    x_1[0] = x_1[0]/sqrtsumx_1
    x_1[1] = x_1[1]/sqrtsumx_1
    x_1[2] = x_1[2]/sqrtsumx_1
    print(x_1)
    y_1 = np.array([z_1[1]*x_1[2]-x_1[1]*z_1[2],z_1[2]*x_1[0]-z_1[0]*x_1[2],z_1[0]*x_1[1]-x_1[0]*z_1[1]])
    print(y_1)
    
    x_0 = np.array([1,0,0])
    y_0 = np.array([0,1,0])
    z_0 = np.array([0,0,1])
    
    R11 = np.sum(x_1*x_0)
    R12 = np.sum(y_1*x_0)
    R13 = np.sum(z_1*x_0)
    R21 = np.sum(x_1*y_0)
    R22 = np.sum(y_1*y_0)
    R23 = np.sum(z_1*y_0)
    R31 = np.sum(x_1*z_0)
    R32 = np.sum(y_1*z_0)
    R33 = np.sum(z_1*z_0)
    R = np.array([[R11,R12,R13,pts_avg[0]],[R21,R22,R23,pts_avg[1]],[R31,R32,R33,pts_avg[2]],[0,0,0,1]])
    
    print("R=",R)
    
    return R