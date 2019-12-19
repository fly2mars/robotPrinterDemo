import numpy as np
import math
def estimate_plane(points):
    a=b=c=d=0
    pts = np.array(points)
    if len(pts) == 0:
        raise Exception('points is empty')
    pts = pts - np.mean(pts, axis=0)
    x = pts[:,0]
    y = pts[:,1]
    z = pts[:,2]
    
    xx = np.sum(x*x.transpose())
    yy = np.sum(y*y.transpose())
    zz = np.sum(z*z.transpose())
    xy = np.sum(x*y.transpose())
    xz = np.sum(x*z.transpose()) 
    yz = np.sum(y*z.transpose())
    det_x = yy*zz - yz*yz
    det_y = xx*zz - xz*xz
    det_z = xx*yy - xy*xy
    det_max = max(det_x, det_y, det_z)
    if det_max <= 0:        
        return a,b,c,d
    if det_max == det_x:
        a = det_x
        b = xz*yz - xy*zz
        c = xy*yz - xz*yy
    elif det_max == det_y:
        a = xz*yz - xy*zz
        b = det_y
        c = xy*xz - yz*xx
    else:
        a = xy*yz - xz*yy
        b = xy*xz - yz*xx
        c = det_z
    sqrtsum = math.sqrt(a*a+b*b+c*c)
    a=a/sqrtsum
    b=b/sqrtsum
    c=c/sqrtsum
    print("a={}, b={}, c={}, d={}".format(a,b,c,d))    
    return a,b,c,d
