function [f, tau] = rne_hw(q, qd, qdd, robot, g)

alpha_0 = zeros(3,1);
omega_0 = zeros(3,1);
ac_0 = zeros(3,1);
ae_0 = zeros(3,1);

%start by doing the forward recursion to get accelerations and velocities
[ac, ae, alphas, omegas] = get_accel(robot, ac_0, ae_0, alpha_0, omega_0, q, qd, qdd);

f_end = zeros(3,1);
tau_end = zeros(3,1);

%then can do the backward recursion to get reaction forces and torques at
%the joints
[f, tau] = get_forces(robot, ac, alphas, omegas, f_end, tau_end, q, g);

end
