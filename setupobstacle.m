% Obstacle1: Floor
obs{1}.R = 800;
obs{1}.c = [0;0;0];
obs{1}.rho0 = 50;
obs{1}.h = 0;
obs{1}.type = 'plane';

% Obstacle2: Cylinder
obs{2}.R = 30;
obs{2}.c = [165; -165];
obs{2}.rho0 = 50;
obs{2}.h = 100;
obs{2}.type = 'cyl';
 
% Obstacle3: Cylinder
obs{3}.R = 70;
obs{3}.c = [0; 130];
obs{3}.rho0 = 50;
obs{3}.h = 100;
obs{3}.type = 'cyl';