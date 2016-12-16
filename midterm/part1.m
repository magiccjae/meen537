% A0 = [1 0 0 0; 0 1 0 0; 0 0 1 2; 0 0 0 1];
% A1 = [0 0 1 0; 1 0 0 0; 0 1 0 2; 0 0 0 1];
% A2 = [sqrt(3)/2 0 1/2 0; 1/2 0 -sqrt(3)/2 0; 0 1 0 0; 0 0 0 1];
% 
% A3 = [1 0 0 0; 0 0 -1 0; 0 1 0 4; 0 0 0 1];
% 
% A4 = [sqrt(3)/2 0 1/2 sqrt(3); 1/2 0 -sqrt(3)/2 1; 0 1 0 0; 0 0 0 1];
% 
% A0*A1*A2*A3*A4;
% 
% jacobian1 = [-4 0 sqrt(3) 0; 0 2*sqrt(3) 0 0; 0 4 0 -2; 0 1 0 -1; 0 0 1/2 0; 1 0 -sqrt(3)/2 0];
% R_21 = [0 1 0; 0 0 1; 1 0 0];
% rot = [R_21 zeros(3); zeros(3) R_21];
% rot*jacobian1;
% 

clear L
clear
close all
%%%%%%%%%%%%% theta, d, a, alpha, revolute or prismatic, offset
L(1) = Link([ 0 	4       0         pi/2    0        pi/2], 'standard');
L(2) = Link([ 0     0       0         pi/2    0          0 ], 'standard');
L(3) = Link([ 0     4       0         pi/2    0          0 ], 'standard');
L(4) = Link([ 0     0       2         pi/2    0          0 ], 'standard');


%% defining the robot now
robot = SerialLink(L, 'name', 'midterm1', ...
    'manufacturer', 'JL robotics');

% some useful poses
qz = [0 pi/6 0 pi/6]; % zero angles, L shaped pose

% robot.plot(qz);
% view(90,0);
robot.A([1 2 3 4],qz);
robot.A([1 2], qz);
A1 = robot.A(1,qz);
A2 = robot.A(2,qz);
A3 = robot.A(3,qz);
A4 = robot.A(4,qz);
A1*A2;
A1*A2*A3;
A1*A2*A3*A4;

A_e_to_2 = A2*A3*A4
my_skew = skew(A_e_to_2(1:3,4))
R_e_to_2 = A_e_to_2(1:3,1:3)
J_o2_in_2 = [R_e_to_2 my_skew*R_e_to_2; zeros(3) R_e_to_2]*robot.jacobn(qz)
R_2_to_0 = A1(1:3,1:3)
J_o2_in_0 = [R_2_to_0 zeros(3); zeros(3) R_2_to_0]*J_o2_in_2