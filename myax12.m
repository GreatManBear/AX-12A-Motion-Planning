%Creates a Robot with 4 links with values of a given DH table and a delta
%offset

function [myrobot] = myax12(delta)
q1=0;
q2=0;
q3=0;
q4=pi/2;


 DH=[pi/2 -55 q1 142;
     0 172 q2 0;
    pi/2 (23+delta(1)) q3 0;
    0 (5+delta(2)) q4 (285+delta(3))];

%Create Links
l1 = link([DH(1,1),DH(1,2),DH(1,3),DH(1,4),0],'standard');
l2 = link([DH(2,1),DH(2,2),DH(2,3),DH(2,4),0],'standard');
l3 = link([DH(3,1),DH(3,2),DH(3,3),DH(3,4),0],'standard');
l4 = link([DH(4,1),DH(4,2),DH(4,3),DH(4,4),0],'standard');

%Groups links
links = {l1 l2 l3 l4};

%Creates Robot object
myrobot = robot(links);
myrobot.name = 'AX-12A';

end