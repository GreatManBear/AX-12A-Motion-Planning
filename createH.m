%Helper function to create H Matrix for i-1 to i frame

function [ Hi ] = createH(alphai,ai,thetai,di)

Hi = [cos(thetai) (-sin(thetai)*cos(alphai)) sin(thetai)*sin(alphai) ai*cos(thetai);
    sin(thetai) cos(thetai)*cos(alphai) -cos(thetai)*sin(alphai) ai*sin(thetai);
    0 sin(alphai) cos(alphai) di;
    0 0 0 1];
    
end
