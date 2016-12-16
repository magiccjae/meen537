function [f, tau] = get_forces(robot, ac, alphas, omegas, f_end, tau_end, q, g)

%initializing variables for reaction forces and torques at joints
f = zeros(3, robot.n);
tau = zeros(3, robot.n);

%doing the backward recursion now starting from the end effector and going
%the other direction towards the base
for i=robot.n:-1:1

    %finding the current transformation
    T_cur_to_zero = robot.A([1:1:i], q);
   
    %if first pass through, we assign the torque at the previous point to
    %be whatever was passed in, otherwise, it is the last force and torque
    %we calculated. "Previous" in this case refers to a joint that is more
    %distal (or further from the base) and make more sense in terms of the
    %order of the recursion instead of the sequential order of the links.
    if i == robot.n
        f_prev = f_end;
        tau_prev = tau_end;
        T_prev_to_zero = T_cur_to_zero;
    else
        T_prev_to_zero = robot.A([1:1:i+1], q);
        f_prev = f(:,i+1);
        tau_prev = tau(:,i+1);
    end
    
    %getting other needed transformations
    T_prev_to_cur = inv(T_cur_to_zero)*T_prev_to_zero;
    R_prev_to_cur = T_prev_to_cur(1:3,1:3);
    R_zero_to_cur = T_cur_to_zero(1:3,1:3)';

    
    %calcuating the forces and torques using equations from class. These
    %are both 3x1 vectors. The torque in z-direction is what we generally 
    %care about since that is what our actuators would have to do. We again
    %assumed that the COM was along the x-direction only and half the
    %distance of the DH parameter "a". This could be more general
    f(:,i) = R_prev_to_cur*f_prev + robot.links(i).m*ac(:,i) - ...
        robot.links(i).m*R_zero_to_cur*g;
    tau(:,i) = R_prev_to_cur*tau_prev - cross(f(:,i), [robot.links(i).a/2; 0; 0]) + ...
        cross(R_prev_to_cur*f_prev, [-robot.links(i).a/2; 0; 0]) + ...
        robot.links(i).I*alphas(:,i) + cross(omegas(:,i), robot.links(i).I*omegas(:,i));
        
end