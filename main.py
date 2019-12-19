'''
This program demostrate a simple process for UR robot based 3D printing.
1) Exturder controller and temperature controller are not given here.
2) Path input is saved in numpy.array txt file
3) Connect UR robot by urx

'''
import traceback
import urx
import numpy as np
import serial, time
from estimate_plane import estimate_plane
from Coordinate_transformation import Coordinate_transformation

class PathManager(object):
    def __init__(self):
        self.path = None
        pass
    
    def load(self, path_file):
        '''
        return path represents by n*3 numpy.array[[x,y,z
        convert unit from millimeter to meters
        '''
        self.path = np.loadtxt(path_file) / 1000
        return self.path
    
    def get_homogeneous_coordinates(self):
        one_arr = np.ones([self.path.shape[0], 1])
        return np.hstack((self.path, one_arr))   
        
class URPrinter(object):
    def __init__(self):
        self.is_connected = False 
    
    def connect(self, ip="169.254.204.33"):
        try:                
            self.rob = urx.Robot(ip)
            self.rob.set_tcp((0.00022,-0.00265,0.12321,0,0,0))  # pre measurement
            self.rob.set_payload(0.5, (0,0,0)) 
            self.is_connected = True
        except Exception as e:
            print(traceback.format_exc())
            print("Begin simulation....")
    
    def close(self): 
        self.rob.close()
        self.is_connected = False
    
    def register_plane(self, N=3):
        '''
        return transform matrix R and tcp pose
        '''
        if not self.is_connected:
            self.R= [[ 0.67404229,-0.73868061,0.00423645,-0.21223078],
                [0.7386522,0.6740543,0.00661261,-0.30540021],
                [-0.0077402,-0.00132791,0.99996916,0.03895087],
                [0,0,0,1]]        
            self.tcp_pose = [-3,-0.811891,-0.031571]
            return self.R, self.tcp_pose            
            
        # get registered points
        points = np.ones((N,6))
        for i in range(N):
            input("Waiting to get coordinates:") 
            pose = rob.getl()  
            points[i,:] = pose
            
        self.tcp_pose = points[0,3:].tolist()
        [a,b,c,d]=estimate_plane(points) 
        self.R=Coordinate_transformation(points,a,b,c,d)
        return self.R, self.tcp_pose
    
    def prepare(self):
        pass
    
    def moveto(self, point):
        move = ','.join(map(str,point))
        programString = "movej(p["+move+"],a=1.4,v=1.04)"        
        if self.is_connected:
            self.rob.send_program(programString)   
        else:
            print(programString)
            
        
        
if __name__ == "__main__":
    pm = PathManager()
    pm.load("examples/standard.txt")
    #init UR3 and TCP
    pr = URPrinter()
    pr.connect()
    #find and register print plane
    R, tcp_pose = pr.register_plane()
    
    pr.prepare()
    #send script to UR3
    homogeneous_coordinates = pm.get_homogeneous_coordinates()
    for p in homogeneous_coordinates:
        v = np.dot(R, p.T)
        v = v.tolist()
        v = v[0:3] + pr.tcp_pose
        
        pr.moveto(v)   
    
