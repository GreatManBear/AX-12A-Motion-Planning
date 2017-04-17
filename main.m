%Run this to demo all motion planning

close all
clear all

%Initialize All Theta(q) to 0
q1=0;
q2=0;
q3=0;
q4=pi/2;

%DH Table of AX12 Robot
DH=[pi/2 -55 q1 142;
    0 172 q2 0;
    pi/2 23 q3 0;
    0 5 q4 285];
    
%Create a ax12 robot with 0 delta
myrobot=myax12([0 0 0 0]);

setupobstacle;

tau = rep([pi/10 pi/10 pi/2 pi/2],myrobot_prep,{obs{1}}) % This tests the torque for the cylinder obstacle

%Motion Planning to go from point p1 to p2 to p3

%Changes points to change desired locations in MM
p1 = [0 -200 5]';
p2 = [200 -20 20]'; 
p3 = [-220 220 40]';

%Generates desired angles of points
q1 = inverse_ax12(myrobot,p1);
q2 = inverse_ax12(myrobot,p2);
q3 = inverse_ax12(myrobot,p3);


qref1 = motionplan(q1, q2, 0, 10, myrobot, obs, 0.011)
qref2 = motionplan(q2, q3, 0, 10, myrobot, obs, 0.04) 

t1 = linspace(0, 10, 300);
t2 = linspace(0, 10, 300);
q_first_path = ppval(qref1, t1)';
q_second_path = ppval(qref2, t2)';


%Ploting motion of robot
figure

hold on;
axis([-400 400 -400 400 0 400]);
view(-32,50);
plotobstacle(obs);
plot(myrobot, q_first_path);
plot(myrobot, q_second_path);
hold off;