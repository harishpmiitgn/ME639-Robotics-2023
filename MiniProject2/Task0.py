import numpy as np

#2R configuration
#All rotations are about the current z axes.
#D-H parameters d,theta,a,alpha

l=11,11.5 # link lengths

# rotation matrix about z axis
def Rzq(q_):
    R=[[np.cos(q_), -np.sin(q_), 0],
       [np.sin(q_), np.cos(q_), 0],
       [0, 0, 1]]
    return R

# rotation matrix about y axis
def Ryq(q):
    R=[[np.cos(q), 0, np.sin(q)],
       [0, 1, 0],
       [-np.sin(q), 0, np.cos(q)]]
    return R

# rotation matrix about x axis
def Rxq(q_):

    R=[[1, 0, 0],
       [0, np.cos(q_), -np.sin(q_)],
       [0, np.sin(q_), np.cos(q_)]]
    return R

# transformation matrix
def H(R=np.identity(3), d=[0,0,0]): #default no rotation, no translation
    H1=[[R[0][0],R[0][1], R[0][2],d[0]],
       [R[1][0],R[1][1], R[1][2],d[1]],
       [R[2][0],R[2][1], R[2][2],d[2]],
       [0,0, 0,1]]
    #print(H1
    return H1

def A(d,theta,a,al):
    d1=[0,0,d]
    a1=[a,0,0]
    print(theta)
    H1=H(np.identity(3),d1)
    H2= H(Rzq(theta))
    H3=H(np.identity(3),a1)
    H4=H(Rxq(al),[0,0,0])
    Hab=(np.linalg.multi_dot([
                H(np.identity(3),d1),
                H(Rzq(theta)),
                 H(np.identity(3),a1),
                 H(Rxq(al),[0,0,0])
                 ]))
    # Ai=[
    #     [np.cos(theta),-np.sin(theta)*np.cos(al),np.sin(theta)*np.sin(al),a*np.cos(theta)],
    #     [np.sin(theta),np.cos(theta)*np.cos(al),-np.cos(theta)*np.sin(al),a*np.sin(theta)],
    #     [0,np.sin(al),np.cos(al),d],
    #     [0,0,0,1]
    # ]
    return Hab

# inverse kinematics
def inv(x,y):
    theta=np.arccos((x**2+y**2-l[0]**2-l[1]**2)/(2*l[0]*l[1])) # in radians
    q1= np.arctan2(y, x) - np.arctan2(l[1] * np.sin(theta), l[0] + l[1]* np.cos(theta)) #in radians
    q2= q1+theta
    return [q1,q2]

def endeffector(q,l):   # forward kinematics
    # transformation matrices
    q1,q2_=q[0],q[1]-q[0]   #q2_ is the relative angle
    # print("rel angles",q1,q2_)
    H01=A(0,q1,l[0],0)
    H12=A(0,q2_,l[1],0)
 
    H=[H01,H12]
    #position of end effector in 0 frame
    P=np.linalg.multi_dot([H01,H12,[0,0,0,1]])

    print("final position of the end effector with respect to the base frame : ", P[0],"i +",P[1],"j +", P[2],"k")
    #return P, H




# input angles to get the desired end effector position

# for _ in range(2):
#     x,y=eval(input("enter x,y coordinates: "))
#     q=inv(x,y)
#     endeffector(q,l)



q=np.pi/2,np.pi/2
print("for q1=pi/2,q2=pi/2 (q1,q2 are absolute angles)")
endeffector(q,l)

q=np.pi,np.pi
print("for q1=pi,q2=pi (q1,q2 are absolute angles)")
endeffector(q,l)