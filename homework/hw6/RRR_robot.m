%% Problem 2
clear;
close all;
clc;

length = 0.4;
Izz = 0.01;
%   theta d a alpha
L(1) = Link([ 0     0   length  0], 'standard');
L(2) = Link([ 0     0   length  0], 'standard');
L(3) = Link([ 0     0   length  0], 'standard');
L(1).m = 1;
L(2).m = 1;
L(3).m = 1;
L(1).r = [-length/2; 0; 0];
L(2).r = [-length/2; 0; 0];
L(3).r = [-length/2; 0; 0];
I = Izz*eye(3);
L(1).I = I;
L(2).I = I;
L(3).I = I;
L(1).G = 0;
L(2).G = 0;
L(3).G = 0;
L(1).Jm = 0;
L(2).Jm = 0;
L(3).Jm = 0;

rrr = SerialLink(L(1:3), 'name', 'RRR');
% rrr.base = [1 0 0 0;
%             0 0 -1 0;
%             0 1 0 0;
%             0 0 0 1];

qz = [0 0 0];
rrr.plot(qz);

q = [pi/4, pi/4, pi/4];
qd = [pi/6, -pi/4, pi/3];
qdd = [-pi/6, pi/3, pi/6];
figure(1)
rrr.plot(q)

g = [0; -9.81; 0];
% Part (b)
tau_rtb = rrr.rne(q, qd, qdd, g)

% Part (a)
g = [0; 9.81; 0];
tau_mine = my_rne(q,qd,qdd,g,rrr,length,L)

% Part (c)
inertia_rtb = rrr.inertia(q)
qd = [0 0 0];
g = [0; 0; 0];
inertia_mine = zeros(3);
inertia_mine(1,:) = my_rne(q,qd,[1;0;0],g,rrr,length,L);
inertia_mine(2,:) = my_rne(q,qd,[0;1;0],g,rrr,length,L);
inertia_mine(3,:) = my_rne(q,qd,[0;0;1],g,rrr,length,L);
inertia_mine
