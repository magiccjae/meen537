clear;
close all;

a1 = 0.4;
a2 = 0.4;
a3 = 0.4;
%   theta d a alpha
L(1) = Link([ 0     0   a1  0], 'standard');
L(2) = Link([ 0     0   a2  0], 'standard');
L(3) = Link([ 0     0   a3  0], 'standard');


threelink = SerialLink(L, 'name', 'three link', ...
		       'comment', 'from Spong, Hutchinson, Vidyasagar');
threelink.base = [1 0 0 0;
                  0 0 -1 0;
                  0 1 0 0;
                  0 0 0 1];
              
syms q1 q2 q3
qz = [q1 q2 q3];
% threelink.plot(qz);

T_1 = threelink.A(1,qz);
T_c1 = T_1;
T_c1(1:3,4) = T_c1(1:3,4)/2;
T_c1;

temp_t2 = threelink.A(2,qz);
T_2 = T_1*temp_t2;
temp_c2 = temp_t2;
temp_c2(1:3,4) = temp_c2(1:3,4)/2;
T_c2 = T_1*temp_c2;

temp_t3 = threelink.A(3,qz);
T_3 = T_2*temp_t3;
temp_c3 = temp_t3;
temp_c3(1:3,4) = temp_c3(1:3,4)/2;
T_c3 = T_2*temp_c3;

z = [0; 0; 1];
jacobian_c1 = [cross(z,T_c1(1:3,4)) zeros(3,2); z zeros(3,2)];
jacobian_c2 = [cross(z,T_c2(1:3,4)) cross(z,T_c2(1:3,4)-T_1(1:3,4)) zeros(3,1); z z zeros(3,1)];
jacobian_c3 = [cross(z,T_c3(1:3,4)) cross(z,T_c3(1:3,4)-T_1(1:3,4)) cross(z,T_c3(1:3,4)-T_2(1:3,4)); z z z];

syms Ixx Iyy Izz q_dot1 q_dot2 q_dot3
q_dot = [q_dot1; q_dot2; q_dot3];

mass = 1;
K_rot = 1/2*q_dot.'*(jacobian_c1(4:6,:).'*[Ixx 0 0; 0 Iyy 0; 0 0 Izz]*jacobian_c1(4:6,:)...
                     + jacobian_c2(4:6,:).'*[Ixx 0 0; 0 Iyy 0; 0 0 Izz]*jacobian_c2(4:6,:)...
                     + jacobian_c3(4:6,:).'*[Ixx 0 0; 0 Iyy 0; 0 0 Izz]*jacobian_c3(4:6,:))*q_dot;

K_trans = 1/2*q_dot.'*(mass*jacobian_c1(1:3,:).'*jacobian_c1(1:3,:) + mass*jacobian_c2(1:3,:).'*jacobian_c2(1:3,:)...
                       + mass*jacobian_c3(1:3,:).'*jacobian_c3(1:3,:))*q_dot;
K_total = K_rot+K_trans;

D = jacobian_c1(4:6,:).'*[Ixx 0 0; 0 Iyy 0; 0 0 Izz]*jacobian_c1(4:6,:)...
                     + jacobian_c2(4:6,:).'*[Ixx 0 0; 0 Iyy 0; 0 0 Izz]*jacobian_c2(4:6,:)...
                     + jacobian_c3(4:6,:).'*[Ixx 0 0; 0 Iyy 0; 0 0 Izz]*jacobian_c3(4:6,:)...
                     + mass*jacobian_c1(1:3,:).'*jacobian_c1(1:3,:) + mass*jacobian_c2(1:3,:).'*jacobian_c2(1:3,:)...
                     + mass*jacobian_c3(1:3,:).'*jacobian_c3(1:3,:);
                

for k=1:3
    for i=1:3
        for j=1:3
            c(i,j,k) = 1/2 * (diff(D(k,j),qz(i)) + diff(D(k,i),qz(j)) - diff(D(i,j),qz(k)));
        end
    end
end

gravity = [0; 9.81; 0];
P = gravity'*T_c1(1:3,4) + gravity'*T_c2(1:3,4) + gravity'*T_c3(1:3,4);
g1 = diff(P,q1);
g2 = diff(P,q2);
g3 = diff(P,q3);
g = [g1; g2; g3];
% Plug into the EOM equation
D
c
g