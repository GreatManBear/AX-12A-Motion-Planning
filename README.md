# AX-12A-Motion-Planning

Basic motion planning function for the AX-12A robot in Matlab with obstacle avoidance. It relies on the gradient descent algorithm to converge between two points. Obstacles have to be modeled with spheres, cylinders, and planes. 

As with most gradient descent algorithms, convergence and obstacle avoidance is not guaranteed. Tuning the accuracy and alpha values of the motion planning algorithm can fix this.

Robitics ToolBox Version 8 must be installed to view simulation.

Run main.m to demo motionplanning.
