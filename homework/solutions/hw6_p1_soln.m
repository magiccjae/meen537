%run the script I gave you 
run p560_startup_script.m;

%get data from simulink simulation
out = sim('sl_puma_hw6');

%get simulink data for comparison
q_s = out.get('q_sim');
qd_s = out.get('qd_sim');
qdd_s = out.get('qdd_sim');
t_s = out.get('t_sim');

%now run ode45 simulation
n_dof = 6
[t, x] = ode45(@eom_puma, [0, 10], zeros(n_dof*2, 1), odeset, ...
    p560.nofriction(), torque, time);

%pull out the joint angles and velocities from the ode45 simulation
q_ode = x(:,7:end);
qd_ode = x(:,1:6);

%now plot the ode45 and simulink results for each joint
figure()
for i=1:1:6
    subplot(6,1,i);
    plot(t,q_ode(:,i), 'b', t_s, q_s(:,i), 'r--');
    ylabel('radians');
end
legend('ODE results', 'Simulink results');
xlabel('time(s)')