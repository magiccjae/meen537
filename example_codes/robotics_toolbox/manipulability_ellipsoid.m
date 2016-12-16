clear all;
clc;
close all; 

% importing the model for Baxter
run mdl_baxter;
baxter = right;

%look at comparing singular or eigen values for two different configurations ...
svd(baxter.jacob0([0, 0, 0, -pi/4, pi/2, pi/2, pi/4]))
svd(baxter.jacob0(zeros(1,7)))
pause

%creating joint configurations to test with
q = zeros(1,7);
q1 = [0, pi/4,pi/2, pi/4, 0, -pi/2, 0];

%finding the Jacobian and extracting the first three rows for q
J = baxter.jacob0(q);
J_vel = J(1:3,:);

%finding the manipulability matrix for J(q)
[vec, val] = eig(J_vel*J_vel')

%finding the Jacobian and extracting the first three rows for q1
J2 = baxter.jacob0(q1);
J_vel2 = J2(1:3,:);

%finding the manipulability matrix for J(q1)
[vec2, val2] = eig(J_vel2*J_vel2')


%finding the end effector position for baxter in position q
pose = baxter.fkine(q);
center = pose(1:3,4);
baxter.plot(q);
hold on;

%the scale can help you plot the ellipsoid more easily.
scale = 1.0;

%this plots the manipulability ellipsoid for q, we could do the same thing
%for q1, but would have to make our own function to plot it.
ellipsoid(center(1), center(2), center(3), val(1,1)*scale, val(2,2)*scale, val(3,3)*scale);p
