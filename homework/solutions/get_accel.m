function [ac, ae, alphas, omegas] = get_accel(robot, ac_0, ae_0, alpha_0, ...
    omega_0, q, qd, qdd)

%initializing the variables to be zero where each column corresponds to a
%new link or position on the link (i.e. center or end)
omegas = zeros(3, robot.n);
alphas = zeros(3, robot.n);
ae = zeros(3, robot.n);
ac = zeros(3, robot.n);

for i=1:1:robot.n
    %find the current transform back to zero
    T_cur_to_zero = robot.A([1:1:i], q);

    %if first time through, there is some funny business such as defining
    %the initial z direction or transformation from from 0 to frame 0.
    if i == 1
        T_prev_to_zero = eye(4);
        z = [0; 0; 1];
        omega_prev = omega_0;
        alpha_prev = alpha_0;
        ae_prev = ae_0;
        ac_prev = ac_0;
    
    %otherwise, we are finding the previous z-direction and the previous
    %transform back to zero which we need along with setting some
    %convenience variables to make the next lines easier to read/shorter
    else
        T_prev_to_zero = robot.A([1:1:i-1],q);
        z = T_prev_to_zero(1:3,3);
        omega_prev = omegas(:, i-1);
        alpha_prev = alphas(:, i-1);
        ae_prev = ae(:, i-1);
        ac_prev = ac(:, i-1);
    end
    
    %getting all the necessary rotation matrices
    T_prev_to_cur = inv(T_cur_to_zero)*T_prev_to_zero;
    R_prev_to_cur = T_prev_to_cur(1:3,1:3);
    R_zero_to_cur = T_cur_to_zero(1:3,1:3)';

    %calculating the omega and alpha terms based on equations from class
    omegas(:,i) = R_prev_to_cur*omega_prev + R_zero_to_cur*z*qd(i);
    alphas(:,i) = R_prev_to_cur*alpha_prev + R_zero_to_cur*z*qdd(i) + ...
        cross(omegas(:,i), R_zero_to_cur*z*qd(i));
    
    %the distance to the COM here is defined has half of DH parameter "a",
    %this could be much more general. Otherwise, we are just calculating
    %the linear acceleration at the COM and the link end here.
    ac(:,i) = R_prev_to_cur*ae_prev + cross(alphas(:,i), [robot.links(i).a/2; 0; 0]) + ...
        cross(omegas(:,i), cross(omegas(:,i), [robot.links(i).a/2; 0; 0]));
    ae(:,i) = R_prev_to_cur*ae_prev + cross(alphas(:,i), [robot.links(i).a; 0; 0]) + ...
        cross(omegas(:,i), cross(omegas(:,i), [robot.links(i).a; 0; 0]));
end

end