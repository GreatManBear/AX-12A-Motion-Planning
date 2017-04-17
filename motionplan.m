%This motion planning function works for all obstacles (no obstacles, 1 obstacle, or multiple obstacles)
function [ qref ] = motionplan(q0,qf,t1,t2,myrobot,obs,accur)

alphaK=1;
alphaKa=0.015;
alphaKr=0.01;

q=q0;

N=1;
q_new=zeros(4,N+1);
q_new(:,1)=q0;

doRun=1;
while((doRun==1)&&(N<800))
    if ((norm(q_new(1:3,N)-qf(1:3))<accur))
       doRun=0;
    else
        tau_att=alphaKa * (att(q(:,N),qf,myrobot)/norm(att(q(:,N),qf,myrobot)))';
        
        if(size(obs)==0) 
            tau_rep=zeros(4,1);
        elseif(norm(rep(q(:,N),myrobot,obs))==0)
            tau_rep=zeros(4,1);
        else
            tau_rep=alphaKr * (rep(q(:,N),myrobot,obs)/norm(rep(q(:,N),myrobot,obs)))';
        end
        
        q_k_1=q(:,N) + alphaK*( tau_att + tau_rep);
        
        q_new=zeros(4,N+1);

        for i=1:N
            q_new(1:4,i)=q(1:4,i);
        end
        
        q_new(:,N+1)=q_k_1;
        
        q=q_new;
        N=N+1;
    end
end
q(4,:)=linspace(q0(4,1),qf(4,1),N);

q=q';

t = linspace(t1,t2,size(q,1));

qref = spline(t,q'); % defines a spline object with interpolation

t =linspace(0,10,40);% times in t and interpolation values the columns of q

qref=spline(t,ppval(qref,t));

end