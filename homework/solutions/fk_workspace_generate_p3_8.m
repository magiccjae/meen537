run p3_8

%number of tests
num_tests = 100000;

%generating random joint angles with joint limits
jt_angles = random('unif', -pi, pi/2, 6, num_tests);

%making emtpy vector to store positions
positions = zeros(3, num_tests);

%calculating FK for each set of random joint angles
for i=1:1:num_tests
    FK = robot.fkine(jt_angles(:,i));
    positions(:, i) = FK(1:3, 4);
    i
end

%calculating a subset for visualization
subset = positions(:, find(positions(1,:)> 0 & positions(2,:)> 0));
    
%plotting results of either full set or subset
%scatter3(positions(1,:), positions(2,:), positions(3,:), 1, sqrt(positions(1,:).^2+positions(2,:).^2+positions(3,:).^2))
scatter3(subset(1,:), subset(2,:), subset(3,:), 1, sqrt(subset(1,:).^2+subset(2,:).^2+subset(3,:).^2))
colormap(jet)

%adding robot to plot for scale
hold on
robot.plot(jt_angles(:,1)')
