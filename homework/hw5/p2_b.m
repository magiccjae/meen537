clear; close all;
storage = load('desired_accel.mat');
joint_angles = storage.q;
time = storage.t;

%calling the file as a function that returns a left and right arm
[left, right] = mdl_baxter('sim');

q_d0 = zeros(1,7);
q_dd0 = zeros(1,7);
q_d = zeros(1001,7);
q_dd = zeros(1001,7);
time_step = 0.01;
% central difference equations to get velocity and acceleration from joint
% angles
for i=2:length(time)-1
    q_d(i,:) = joint_angles(i+1,:)-joint_angles(i,:)/time_step;
    q_dd(i,:) = joint_angles(i+1,:)-2*joint_angles(i,:)+joint_angles(i-1,:)/time_step^2;
end

joint_tau = zeros(7,length(time));
for i=1:length(time)
    M = left.inertia(joint_angles(i,:));
    C = left.coriolis(joint_angles(i,:),q_d(i,:));
    G = left.gravload(joint_angles(i,:));
    joint_tau(:,i) = M*q_dd(i,:)'+C*q_d(i,:)'+G';
end

%%
plot(joint_tau');
xlabel('time');
ylabel('joint torque');
xlim([0 1001]);