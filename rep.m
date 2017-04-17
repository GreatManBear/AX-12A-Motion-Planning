%Calculates tau for the repulsive force between a given point and obstacles passed into the function

function [ tau ] = rep( q, myrobot, obs )

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

%Uses i-1 to i frames from i=1 to 6, to create H matrix for 0 to 4
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

%Compute the replusive force (Frep)
Frep=zeros(3,4);

length=size(obs);
%Loops to add the forces for each obstacle
for i=1:length(1,2)
    %Calculalte for b (closest point to obs)
    b=zeros(3,4);
    oiq_b=zeros(3,4);
    poiq=zeros(3,4);
    grad_p=zeros(3,4);
    for k=1:4
        switch obs{i}.type
            case 'cyl' %If the object is a cylinder
                if (Oi_q(3,k)<obs{i}.h)
                    r=sqrt((Oi_q(1,k)-obs{i}.c(1,1))^2+(Oi_q(2,k)-obs{i}.c(2,1))^2);
                    b(1,k)=obs{i}.c(1,1)+ obs{i}.R * (Oi_q(1,k)-obs{i}.c(1,1)) / r;
                    b(2,k)=obs{i}.c(2,1)+ obs{i}.R * (Oi_q(2,k)-obs{i}.c(2,1)) / r;
                    b(3,k)=Oi_q(3,k);
                elseif (sqrt((Oi_q(1,k)-obs{i}.c(1,1))^2+(Oi_q(2,k)-obs{i}.c(2,1))^2)<obs{i}.R)
                    b(1,k)=Oi_q(1,k);
                    b(2,k)=Oi_q(2,k);
                    b(3,k)=obs{i}.h;              
                else
                    r=sqrt((Oi_q(1,k)-obs{i}.c(1,1))^2+(Oi_q(2,k)-obs{i}.c(2,1))^2);
                    b(1,k)=obs{i}.c(1,1)+ obs{i}.R * (Oi_q(1,k)-obs{i}.c(1,1)) / r;
                    b(2,k)=obs{i}.c(2,1)+ obs{i}.R * (Oi_q(2,k)-obs{i}.c(2,1)) / r;
                    b(3,k)=obs{i}.h;             
                end
            case 'sph' %If the object is a sphere
                r=sqrt((Oi_q(1,k)-obs{1}.c(1,1))^2+(Oi_q(2,k)-obs{i}.c(2,1))^2+(Oi_q(3,k)-obs{i}.c(3,1))^2);
                b(1,k)=obs{i}.c(1,1)+ obs{i}.R * (Oi_q(1,k)-obs{i}.c(1,1)) / r;
                b(2,k)=obs{i}.c(2,1)+ obs{i}.R * (Oi_q(2,k)-obs{i}.c(2,1)) / r;
                b(3,k)=obs{i}.c(3,1)+ obs{i}.R * (Oi_q(3,k)-obs{i}.c(3,1)) / r;
            case 'plane' %If the object is a sphere
                b(1,k)=Oi_q(1,k);
                b(2,k)=Oi_q(2,k);
                b(3,k)=0;
        end
        oiq_b(:,k)= Oi_q(:,k)-b(:,k);
        grad_p(:,k)=oiq_b(:,k)/norm(oiq_b(:,k));
        poiq(:,k)=norm(oiq_b(:,k));
    end

    eta=1;
    %Summing the forces
    for j=1:4
        if (poiq(:,j)>obs{i}.rho0)
            Frep(:,j) = Frep(:,j) + 0;
        else
            Frep(:,j) = Frep(:,j) + (eta * (1/norm(oiq_b(:,j)) - (1/obs{i}.rho0)) * (1/(norm(oiq_b(:,j))^2)) * grad_p(:,j));
        end
    end  
end

%Calculates the torque (tau)
tau=zeros(4,1);
for i=1:4
    tau = tau + Jv(:,:,i)'*Frep(:,i);
end

norm_tau=norm(tau);
if(norm_tau==0)
    tau=zeros(4,1);
else
    for i=1:4
        tau(i) = tau(i)/norm_tau;
    end 
end
tau=tau';

end
