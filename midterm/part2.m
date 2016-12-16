clear L
clear
close all
%%%%%%%%%%%%% theta, d, a, alpha, revolute or prismatic, offset
L(1) = Link([ 0     2       0          0      0          0 ], 'standard');
L(2) = Link([ 0 	2       0         pi/2    0        pi/2], 'standard');
L(3) = Link([ 0     0       0         pi/2    0          0 ], 'standard');
L(4) = Link([ 0     4       0         pi/2    0          0 ], 'standard');
L(5) = Link([ 0     0       2         pi/2    0          0 ], 'standard');

% another robot defined to easily get jacobian for the second to the last
% DH frame
M(1) = Link([ 0     2       0          0      0          0 ], 'standard');
M(2) = Link([ 0 	2       0         pi/2    0        pi/2], 'standard');
M(3) = Link([ 0     0       0         pi/2    0          0 ], 'standard');
M(4) = Link([ 0     4       0         pi/2    0          0 ], 'standard');


%% defining the robot now
robot = SerialLink(L, 'name', 'midterm1', ...
    'manufacturer', 'JL robotics');
robot1 = SerialLink(M,'name', 'helper', ...
    'manufacturer', 'JL robotics');

% some useful poses
qz = [0 0 pi/6 0 pi/6]; % zero angles, L shaped pose

figure(1); hold on;
robot.plot(qz);
% obstacle sphere
[x,y,z] = sphere;
surf(x,y+3,z+2);
% goal position
goal_position = [0; 2; 4];
scatter3(goal_position(1),goal_position(2),goal_position(3),100,'r','p','filled');
view(90,0);

%%========== method 2 ==========
% starting configuration with qz
k = 1;
gain = 1;
kappa = [gain*eye(3); zeros(3)];
threshold = 0.1;
time_step = 0.5;
q = qz;
error = 100;
iteration = 0;
while error > threshold
    J = robot.jacob0(q);
    J_A = [eye(3) zeros(3); zeros(3) zeros(3)]*J;
    current_fk = robot.fkine(q);
    current_position = current_fk(1:3,4);
    q_dot = J_A'*inv(J_A*J_A' + k^2*eye(6))*(kappa*(goal_position-current_position));
    q = q + q_dot'*time_step;
    %     robot.plot(q);
    iteration = iteration+1;
    error = norm(current_position-goal_position)
end
desired_position = goal_position;
num_iteration = iteration;
final_joint_angles = q;
qf = final_joint_angles;    % goal joint angles

q = qz;
[T,all] = robot.fkine(qf);
% only the last two DH frames(frame 4, 5) are possible to collide
f4_goal = all(1:3,4,4);
f5_goal = all(1:3,4,5);
threshold = 0.3;
alpha = 0.02;      % parameter for gradient descent
% anim = Animate('movie');
while norm(q-qf) > threshold
    [T,all] = robot.fkine(q);
    f4_current = all(1:3,4,4);
    f5_current = all(1:3,4,5);
    
    d = 2;      % parameter for attractive force
    zeta = 1;       % parameter for attractive force
    
    % calculating attractive force
    if norm(f4_current-f4_goal) <= d
        att_4 = -zeta*(f4_current-f4_goal);
    else
        att_4 = -d*zeta*(f4_current-f4_goal)/norm(f4_current-f4_goal);
    end
    
    if norm(f5_current-f5_goal) <= d
        att_5 = -zeta*(f5_current-f5_goal);
    else
        att_5 = -d*zeta*(f5_current-f5_goal)/norm(f5_current-f5_goal);
    end
    
    % calculating repulsive force
    rho0 = 3;       % parameter for repulsive force
    eta = 1;        % parameter for repulsive force
    % calculating the shortest distance between the sphere and origins of the last two DH frames
    % and get a point on the sphere that is closest to the origins of the last
    % two DH frames
    min_4 = 10;
    min_5 = 10;
    b_4 = zeros(3,1);
    b_5 = zeros(3,1);
    for i=1:21
        for j=1:21
            a_point = [x(i,j); y(i,j)+3; z(i,j)+2];
            temp_dist = norm(f4_current - a_point);
            temp_dist2 = norm(f5_current - a_point);
            if temp_dist <= min_4
                min_4 = temp_dist;
                b_4 = a_point;
            end
            if temp_dist2 <= min_5
                min_5 = temp_dist2;
                b_5 = a_point;
            end
        end
    end
    
    rep_4 = zeros(3,1);
    rep_5 = zeros(3,1);
    if min_4 <= rho0
        rep_4 = eta*(1/min_4 - 1/rho0)*1/(min_4^2)*((f4_current - b_4)/min_4);
    end
    if min_5 <= rho0
        rep_5 = eta*(1/min_5 - 1/rho0)*1/(min_5^2)*((f5_current - b_5)/min_5);
    end
    
    % calculating jacobian for the points of interst
    jacobian_5 = robot.jacob0(q);
    jacobian_4 = zeros(6,5);
    jacobian_4(:,1:4) = robot1.jacob0(q(1:4));
    jv_4 = jacobian_4(1:3,:);
    jv_5 = jacobian_5(1:3,:);
    
    tau_att_4 = jv_4'*att_4;
    tau_att_5 = jv_5'*att_5;
    tau_rep_4 = jv_4'*rep_4;
    tau_rep_5 = jv_5'*rep_5;
    
    tau = tau_att_4 + tau_att_5 + tau_rep_4 + tau_rep_5;
    
    q = q + alpha*(tau/norm(tau))';
    robot.plot(q);
%     anim.add();
end

