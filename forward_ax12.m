%Uses Forward Kinetics to Create Homonogenous Transformation Matrix H04

function [ H ] = forward_ax12(myrobot,joint)
%Extracts links from the robot
l1=myrobot.link{1,1};
l2=myrobot.link{2,1};
l3=myrobot.link{3,1};
l4=myrobot.link{4,1};

%Creates i-1 to i frame
H01=createH(l1.alpha,l1.A,joint(1),l1.D);
H12=createH(l2.alpha,l2.A,joint(2),l2.D);
H23=createH(l3.alpha,l3.A,joint(3),l3.D);
H34=createH(l4.alpha,l4.A,joint(4),l4.D);

%Uses i-1 to i frames from i=1 to 4, to create H matrix for 0 to 4
H=H01*H12*H23*H34;

end




