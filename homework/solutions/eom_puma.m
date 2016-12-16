function xdot = eom_puma(t, x, robot, torque, time)
%first define the # of degrees of freedom
n = length(x)/2;

%make a zero vector for the derivatives
xdot = zeros(2*n, 1);

%calculate the current applied torque based on the torque and time vector
%that we passed in for simulation
tau = interp1(time, torque, t);

%assign the joint velocities as the derivatives of the joint angles
xdot(n+1:end) = x(1:n);

%calculate and assign the joint accelerations
qdd = robot.accel(x(n+1:end)', x(1:n)', tau);
xdot(1:n) = qdd;

end