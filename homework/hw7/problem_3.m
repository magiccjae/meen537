clear; close all; clc;

run ~/a_jae_stuff/classes/meen537/robotics_tools/startup_rvc.m
run ~/a_jae_stuff/classes/meen537/vision_tools/startup_rvc.m

load('hw7_prob3.mat');

cam = CentralCamera('default');
T1 = cam.estpose(P1,p1)
T2 = cam.estpose(P2,p2)
T3 = cam.estpose(P3,p3)

