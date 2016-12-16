function xdot = eom(t,x)
% % keyboard
% global tau_sample;
% global t_sim;
% global p560;
% 
% tau_now = interp1(t_sim,tau_sample,t);
% xdot(7:12,1) = x(1:6);
% xdot(1:6) = p560.accel(x(7:12)',x(1:6)',tau_now);

% keyboard
global tau;
global tau_time;
global p560;

tau_now = interp1(tau_time,tau,t);
xdot(7:12,1) = x(1:6);
xdot(1:6) = p560.accel(x(7:12)',x(1:6)',tau_now);

end
