clear all;
clc;

%joint angles, velocities and accelerations as defined in homework
q_0 = [pi/4, pi/4, pi/4]';
qd_0 = [pi/6, -pi/4, pi/3]';
qdd_0 = [-pi/6, pi/3, pi/6]';
g = [0; 9.81; 0];

%get the robot model
run three_link_full;

%to just get the total torques we can run this function
[f, tau] = rne_hw(q_0, qd_0, qdd_0, three_link, g);
%to get the tau we care about on the actuators, we have to grab the last
%entry in each column (or the z-direction torque)
tau = tau(end,:)'  %this agrees with three_link.rne(q_0', qd_0', qdd_0') except for a negative which means that our convention for positive torque direction was different.


%to get inertia matrix, we can call our RNE function multiple times to get
%each column. Note that we are setting gravity and joint velocity to zero
%to find this correctly.
qdd = zeros(three_link.n,1);
D = zeros(three_link.n);
for i=1:1:three_link.n
    qdd_buf = qdd;
    qdd_buf(i) = 1;
    [D_f, D_tau] = rne_hw(q_0, zeros(three_link.n,1), qdd_buf, three_link, zeros(3,1));
    D(:,i) = D_tau(end,:)';
end

%to get gravity torques we can pass in all zeros except for position and
%gravity vector - gravity was not required for homework, but this is just 
%to show how it works
[g_f, g_tau] = rne_hw(q_0, zeros(three_link.n,1), zeros(three_link.n,1), three_link, g);

%we can compare our RNE and toolbox's and they agree exactly
D
three_link.inertia(q_0')


