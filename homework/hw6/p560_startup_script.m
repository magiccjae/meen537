%%
clear all
clc;

%define the robotics toolbox Puma 560 arm
mdl_puma560;

%set the Coulomb friction terms to zero to help with numerical simulation
p560 = p560.nofriction;

%load the torque profile and open the simulink model
load puma560_torque_profile.mat
open sl_puma_hw6

%%
% part (a)
q = out.get('q_sim');
q_dot = out.get('qd_sim');
q_ddot = out.get('qdd_sim');
global t_sim;
t_sim = out.get('t_sim');

global tau;
global tau_time;
global p560;
tau = torque;
tau_time = time;

x_0 = [0 0 0 0 0 0 0 0 0 0 0 0];
t_range = [time(1) time(end)];
[t, x] = ode45(@eom, time, x_0);

qdd_accel = p560.accel(x(:,7:12),x(:,1:6),interp1(tau_time,tau,t));
figure(2); hold on;
plot(t_sim, q_ddot);
plot(t, qdd_accel,'--');
title('comparison');
% % part (b)
% global tau_sample;
% global p560;
% tau_sample = interp1(time,torque,t_sim);
% 
% x_0 = [0 0 0 0 0 0 0 0 0 0 0 0];
% t_range = [t_sim(1) t_sim(end)];
% [t, x] = ode45(@eom, t_range, x_0);
% 
% qdd_accel = p560.accel(x(:,7:12),x(:,1:6),interp1(t_sim,tau_sample,t));
% 
% figure(2); hold on;
% plot(t_sim, q_ddot);
% plot(t, qdd_accel,'--');