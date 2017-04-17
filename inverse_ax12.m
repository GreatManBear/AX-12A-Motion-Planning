%Uses Inverse Kinematics to Generate Orientation (q) of all links

function [q] = inverse_ax12(myrobot,H)

%Extracts links from robot
l1=myrobot.link{1,1};
l2=myrobot.link{2,1};
l3=myrobot.link{3,1};
l4=myrobot.link{4,1};

%Extracts (X,Y,Z) 
xc=H(1,1);
yc=H(2,1);
zc=H(3,1);

%Theta1 is q1
theta1=atan2(yc,xc)+atan2(l4.A^2,real(sqrt(xc^2+yc^2-(l4.A^2))));

r=-l1.A^2 + real(sqrt(xc^2+yc^2-l4.A^2));
C=real((sqrt(l3.A^2+l4.D^2)));
D=(r^2+(zc-l1.D)^2-l2.A^2-C^2)/(2*l2.A*C);

phi=atan2(real((sqrt(1-D^2))),D);

%Theta2 is q2
theta2=atan2(zc-l1.D,-l1.A+real(sqrt(xc^2+yc^2-l4.A^2)))+atan2(C*sin(phi),l2.A+C*cos(phi));

%Theta3 is q3
theta3=pi/2-phi-atan2(l3.A,l4.D);

%Theta1, Theta2, Theta3, pi/2 -- as desribed in lab manual
q=[theta1;theta2;theta3;pi/2];

end