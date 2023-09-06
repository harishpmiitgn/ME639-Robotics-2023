import numpy as np

#transfromation matrix for 6R UR5 robot

def position_UR5 (theta_1,theta_2,theta_3,theta_4,theta_5,theta_6,l1,l2,l3,l4,l5,l6,l7,l8):
    #rotation matrix base to frame 1
    R01 = np.array([[np.cos(theta_1), -np.sin(theta_1), 0],
                    [np.sin(theta_1), np.cos(theta_1), 0],
                    [0, 0, 1]])
    #translation matrix base to frame 1
    d01 = np.array([[0,0,0]]).T
    #transformation matrix base to frame 1
    T01 = np.concatenate((R01,d01),axis=1)
    #rotation matrix about x axis through pi/2
    Rx = np.array([[1,0,0],
                     [0,np.cos(np.pi/2),-np.sin(np.pi/2)],
                        [0,np.sin(np.pi/2),np.cos(np.pi/2)]])
    #rotation through theta_2 about z axis
    R12dash = np.array([[np.cos(theta_2), -np.sin(theta_2), 0],
                    [np.sin(theta_2), np.cos(theta_2), 0],
                    [0, 0, 1]])
    #rotation matrix frame 1 to frame 2
    R12 = np.matmul(Rx,R12dash)
    #translation matrix frame 1 to frame 2
    d12 = np.array([[0,-l1,0]]).T
    #transformation matrix frame 1 to frame 2
    T12 = np.concatenate((R12,d12),axis=1) 
    #rotation matrix frame 2 to frame 3
    R23 = np.array([[np.cos(theta_3), -np.sin(theta_3), 0],
                    [np.sin(theta_3), np.cos(theta_3), 0],
                    [0, 0, 1]])
    #translation matrix frame 2 to frame 3
    d23 = np.array([[-l2*np.sin(theta_3),l2*np.cos(theta_3),0]]).T
    #transformation matrix frame 2 to frame 3
    T23 = np.concatenate((R23,d23),axis=1)
    #rotation matrix frame 3 to frame 4
    R34 = np.array([[np.cos(theta_4), -np.sin(theta_4), 0],
                    [np.sin(theta_4), np.cos(theta_4), 0],
                    [0, 0, 1]])
    #translation matrix frame 3 to frame 4
    d34 = np.array([[-l4*np.sin(theta_4),l4*np.cos(theta_4),-l3]]).T
    #transformation matrix frame 3 to frame 4
    T34 = np.concatenate((R34,d34),axis=1)
    #rotation matrix about y axis through pi/2
    Ry = np.array([[np.cos(np.pi/2),0,np.sin(np.pi/2)],
                        [0,1,0],
                        [-np.sin(np.pi/2),0,np.cos(np.pi/2)]])
    #rotation through theta_5 about z axis
    R45dash = np.array([[np.cos(theta_5), -np.sin(theta_5), 0],
                    [np.sin(theta_5), np.cos(theta_5), 0],
                    [0, 0, 1]])
    #rotation matrix frame 4 to frame 5
    R45 = np.matmul(Ry,R45dash)
    #translation matrix frame 4 to frame 5
    d45 = np.array([[l6,0,l5]]).T
    #transformation matrix frame 4 to frame 5
    T45 = np.concatenate((R45,d45),axis=1)
    #rotation matrix about y axis through -pi/2
    Ry2  = np.array([[np.cos(-np.pi/2),0,np.sin(-np.pi/2)],
                        [0,1,0],
                        [-np.sin(-np.pi/2),0,np.cos(-np.pi/2)]])
    #rotation through theta_6 about z axis
    R56dash = np.array([[np.cos(theta_6), -np.sin(theta_6), 0],
                    [np.sin(theta_6), np.cos(theta_6), 0],
                    [0, 0, 1]])
    #rotation matrix frame 5 to frame 6
    R56 = np.matmul(Ry2,R56dash)
    #translation matrix frame 5 to frame 6
    d56 = np.array([[-l8,0,l7]]).T
    #transformation matrix frame 5 to frame 6
    T56 = np.concatenate((R56,d56),axis=1)

    T02 = np.matmul(T01,T12) #transformation matrix base to frame 2
    T03 = np.matmul(T02,T23) #transformation matrix base to frame 3
    T04 = np.matmul(T03,T34) #transformation matrix base to frame 4
    T05 = np.matmul(T04,T45) #transformation matrix base to frame 5
    T06 = np.matmul(T05,T56) #transformation matrix base to frame 6
    
    return T06 [0:2,3] #return position of end effector

