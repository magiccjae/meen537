clear;
close all;
clc;

run ~/a_jae_stuff/classes/meen537/robotics_tools/startup_rvc.m
run ~/a_jae_stuff/classes/meen537/vision_tools/startup_rvc.m

%define the robotics toolbox Puma 560 arm
mdl_puma560;

%set the Coulomb friction terms to zero to help with numerical simulation
p560 = p560.nofriction;

%load the torque profile and open the simulink model
open sl_puma_hw7


%%

q = out.get('q_sim');
q_dot = out.get('qd_sim');
q_ddot = out.get('qdd_sim');
global t_sim;
t_sim = out.get('t_sim');
