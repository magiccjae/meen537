clear all;
clc;

%syms a1 a2 theta1 theta2
syms theta1 theta2
%th  d  a  alpha type offset

% define the following DH params for simplicity:
a1 = 1
a2 = 1
L(1) = Link([0 0 a1 -pi/2]);
L(2) = Link([0 0 a2 0]);
robot =  SerialLink(L, 'name', 'hw3_problem2');

%part a)i)
T = inv(robot.fkine([0,0]));
z0_2 = T(1:3,3);
o0_2 = T(1:3,4);
A2 = inv(robot.A(2, [0,0]));
z1_2 = A2(1:3,3);
o1_2 = A2(1:3,4);

nJn = [cross(z0_2, -o0_2), cross(z1_2, -o1_2);
        z0_2,              z1_2]

%part a)ii)
J = robot.jacob0([0,0]);
H = robot.fkine([0, 0]);
R = H(1:3, 1:3)'; %the transpose changes this to be from the base frame to the end effector
nJn_compare = [R zeros(3,3);
               zeros(3,3), R]*J
   
%this is equal to the jacobian in part i)


%part b)i)
q = [0, pi/4]';
F = [-1, 0, 0]';

J = robot.jacob0(q);
tau = J(1:3,:)'*F

%part b)ii)
q = [0, pi/2]';
F = [-1, 0, 0]';

J = robot.jacob0(q);
tau = J(1:3,:)'*F

%part b)iii)
q = [pi/4, pi/4]';
F = [-1, -1, 0]';

J = robot.jacob0(q);
tau = J(1:3,:)'*F

%part b)iv)
q = [0, 0]';
F = [0, 0, 1]';

J = robot.jacob0(q);
tau = J(1:3,:)'*F

%part b)v)
q = [0, 0]';
F = [1, 0, 0]';

J = robot.jacob0(q);
tau = J(1:3,:)'*F


