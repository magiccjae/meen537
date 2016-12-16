%p4_17 homework problem
%
clear all;
clc;

syms a1 a2 a3 theta1 theta2 theta3
%     th  d  a  alpha type offset

L(1) = Link([0 a1 0 pi/2]);
L(2) = Link([0 0 a2 0]);
L(3) = Link([0 0 a3 0]);

robot =  SerialLink(L, 'name', 'p4_17');

%grabbing the relevant transformations, and then using the z-axis and the
%translation from them to calculate the Jacobian
T_01 = robot.A(1, [theta1, theta2, theta3]);
T_02 = robot.A(1, [theta1, theta2, theta3])*robot.A(2, [theta1, theta2, theta3]);
T_03 = robot.fkine([theta1, theta2, theta3]);
z01 = T_01(1:3,3);
o_01 = T_01(1:3,4);
z02 = T_02(1:3,3);
o_02 = T_02(1:3,4);
o_03 = T_03(1:3,4);

jacob = simplify([cross([0, 0, 1]', o_03) cross(z01, o_03-o_01) cross(z02, o_03-o_02)])

% I used these to check my calculations above
%jacob_check = robot.jacob0([theta1, theta2, theta3]);
%jacob_check = simplify(jacob(1:3,:))

%calculating the determinant now
simplify(det(jacob))

