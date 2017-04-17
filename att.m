%Calculates tau for the attractive force between a given point to the desired location

function [ tau ] = att( q, qf, myrobot )

%Compute forward kinematics for current position of origin o_i
%Extracts links from the robot
l1=myrobot.link{1,1};
l2=myrobot.link{2,1};
l3=myrobot.link{3,1};
l4=myrobot.link{4,1};

H0i=zeros(4,4,4);

%Creates i-1 to i frame
H01=createH(l1.alpha,l1.A,q(1),l1.D);
H12=createH(l2.alpha,l2.A,q(2),l2.D);
H23=createH(l3.alpha,l3.A,q(3),l3.D);
H34=createH(l4.alpha,l4.A,q(4),l4.D);

%Uses i-1 to i frames from i=1 to 6, to create H matrix for 0 to 6
H0i(:,:,1)=H01;
H0i(:,:,2)=H01*H12;
H0i(:,:,3)=H01*H12*H23;
H0i(:,:,4)=H01*H12*H23*H34;

Oi_q=zeros(3,4);
for i=1:4   
    Oi_q(:,i)=H0i(1:3,4,i);
end

%Compute the Jacobian matrix

Jv = zeros(3,4,4);

z0_j_1= zeros(3,1);
O0_j_1= zeros(3,1);
for i=1:4
    for j=1:i
        if (j==1)
            z0_j_1=[0;0;1];
            O0_j_1=[0;0;0];
        else
            z0_j_1(:,1)=H0i(1:3,3,j-1);
            O0_j_1=Oi_q(:,j-1);
        end
        Jv(:,j,i)=cross(z0_j_1,(Oi_q(:,i)-O0_j_1));
    end
end

%Compute forward kinematics for current position of origin o_i_qf
%Creates i-1 to i frame
H01=createH(l1.alpha,l1.A,qf(1),l1.D);
H12=createH(l2.alpha,l2.A,qf(2),l2.D);
H23=createH(l3.alpha,l3.A,qf(3),l3.D);
H34=createH(l4.alpha,l4.A,qf(4),l4.D);

%Uses i-1 to i frames from i=1 to 6, to create H matrix for 0 to 6
H0i(:,:,1)=H01;
H0i(:,:,2)=H01*H12;
H0i(:,:,3)=H01*H12*H23;
H0i(:,:,4)=H01*H12*H23*H34;

Oi_qf=zeros(3,4);
for i=1:4   
    Oi_qf(:,i)=H0i(1:3,4,i);
end

zeta=1;

%Compute the attractive force (Fatt)
Fatt=zeros(3,4);

%Sums the forces
for i=1:4
    Fatt(:,i) = -zeta * (Oi_q(:,i) - Oi_qf(:,i));
end

%Calculates the torque (tau)
tau=zeros(4,1);
for i=1:4
    tau = tau + Jv(:,:,i)'*Fatt(:,i);
end

norm_tau=norm(tau);

for i=1:4
    tau(i) = tau(i)/norm_tau;
end 

tau=tau';

end

