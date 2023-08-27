# 2R-manipulator
ME 639 - Introduction to robotics, Mini project, 2023 (IIT Gandhinagar)


## Task 0
Derivation of the 6 key equations derived with regard to the kinematics, dynamics, and statics of the elbow manipulator.

## Task 1
Python simulation with visualization that implements a trajectory in the  controller that makes the robot follow an arbitrary end-tip trajectory.

Here I used inverse kinematics. I first decided the trajectory in planar cartesian coordinate (x,y) and from that calculated $θ_1$; , $θ_2$; and created an animation.

When we include dynamics in visualization, there will be external forces acting on the robot. When we neglect dynamics and consider only kinematics, we analyze the motion of the manipulator without considering the forces and torques involved which affect the controlling of the motors.

## Task 2
In Python visualization, I considered the trajectory of the end tip in the normal direction to the wall so that when the end tip of the manipulator touches the wall, it will exert normal force on the wall.

## Task 3
In this task, I showed an animation of a 2R manipulator end tip acting as a virtual spring.

x<sub>0</sub> and y<sub>0</sub> is the mean position.

When the end tip will go away from the mean position, a force will pull the end tip towards it like spring. For visualization, I recorded the $θ_1$ and $θ_2$ 
values at each point and stored them in an array and the end tip will travel through the path towards the mean position following these angles values from the array in reverse order.

## Task 4
In this, I showed the workspace of the end tip of the manipulator i.e. the space where the end tip can reach.

$θ_1$ = (35&deg; , 145&deg;)

$θ_2$ = (35&deg; , 145&deg;)

So using forward kinematics we can find the end tip location at each possible pair of $θ_1$ and $θ_2$. 

