clear;
close all;
clc;

run ~/a_jae_stuff/classes/meen537/robotics_tools/startup_rvc.m
run ~/a_jae_stuff/classes/meen537/vision_tools/startup_rvc.m

%define the robotics toolbox Puma 560 arm
mdl_puma560;

%set the Coulomb friction terms to zero to help with numerical simulation
p560 = p560.nofriction;
p560 = p560.perturb(0.15);

%%
% Feed Forward
ff_error = q_ff_desired.Data-q_ff_actual.Data;
figure; hold on
subplot(3,2,1)
plot(q_ff_desired.Time, ff_error(:,1));
title('Feed Forward tracking error: Joint1');

subplot(3,2,2)
plot(q_ff_desired.Time, ff_error(:,2));
title('Feed Forward tracking error: Joint2');

subplot(3,2,3)
plot(q_ff_desired.Time, ff_error(:,3));
title('Feed Forward tracking error: Joint3');

subplot(3,2,4)
plot(q_ff_desired.Time, ff_error(:,4));
title('Feed Forward tracking error: Joint4');

subplot(3,2,5)
plot(q_ff_desired.Time, ff_error(:,5));
title('Feed Forward tracking error: Joint5');

subplot(3,2,6)
plot(q_ff_desired.Time, ff_error(:,6));
title('Feed Forward tracking error: Joint6');

%%
% Computed Torque
q_ct_actual = out.get('q_ct_actual');
q_ct_desired = out.get('q_ct_desired');

ct_error = q_ct_desired.Data-q_ct_actual.Data;
figure; hold on
subplot(3,2,1)
plot(q_ct_desired.Time, ct_error(:,1));
title('Computed Torque tracking error: Joint1');

subplot(3,2,2)
plot(q_ct_desired.Time, ct_error(:,2));
title('Computed Torque tracking error: Joint2');

subplot(3,2,3)
plot(q_ct_desired.Time, ct_error(:,3));
title('Computed Torque tracking error: Joint3');

subplot(3,2,4)
plot(q_ct_desired.Time, ct_error(:,4));
title('Computed Torque tracking error: Joint4');

subplot(3,2,5)
plot(q_ct_desired.Time, ct_error(:,5));
title('Computed Torque tracking error: Joint5');

subplot(3,2,6)
plot(q_ct_desired.Time, ct_error(:,6));
title('Computed Torque tracking error: Joint6');
