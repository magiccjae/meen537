function tau = my_rne(q,qd,qdd,g,rrr,length,L)
T1 = rrr.A(1,q);
T2 = rrr.A(2,q);
T3 = rrr.A(3,q);
R1 = T1(1:3,1:3);
R2 = T2(1:3,1:3);
R3 = T3(1:3,1:3);

R0_1 = R1;
R0_2 = R1*R2;
R0_3 = R1*R2*R3;

z = [0; 0; 1]; %For all links

w_0 = [0;0;0];
alpha0 = [0;0;0];
ae_0 = [0;0;0];


w_1 = R1'*w_0 + R0_1'*z*qd(1);
w_2 = R2'*w_1 + R0_2'*z*qd(2);
w_3 = R3'*w_2 + R0_3'*z*qd(3);

alpha1 = R1'*alpha0 + R0_1'*z*qdd(1) + cross(w_1,R0_1'*z*qd(1));
alpha2 = R2'*alpha1 + R0_2'*z*qdd(2) + cross(w_2,R0_2'*z*qd(2));
alpha3 = R3'*alpha2 + R0_3'*z*qdd(3) + cross(w_3,R0_3'*z*qd(3));

r0_1 = [length; 0; 0];
r1_2 = [length; 0; 0];
r2_3 = [length; 0; 0];

ae_1 = R1'*ae_0 + cross(alpha1,r0_1) + cross(w_1,cross(w_1,r0_1));
ae_2 = R2'*ae_1 + cross(alpha2,r1_2) + cross(w_2,cross(w_2,r1_2));
ae_3 = R3'*ae_2 + cross(alpha3,r2_3) + cross(w_3,cross(w_3,r2_3));

r1_c1 = r0_1/2;
r2_c2 = r1_2/2;
r3_c3 = r2_3/2;

ac_0 = [0;0;0];
ac_1 = R1'*ae_0 + cross(alpha1,r1_c1) + cross(w_1,cross(w_1,r1_c1));
ac_2 = R2'*ae_1 + cross(alpha2,r2_c2) + cross(w_2,cross(w_2,r2_c2));
ac_3 = R3'*ae_2 + cross(alpha3,r3_c3) + cross(w_3,cross(w_3,r3_c3));

g_temp = g;
f_4 = [0;0;0];
f_3 = L(3).m*ac_3 - L(3).m*R0_3'*g_temp;
f_2 = R2*f_3 + L(2).m*ac_2 - L(2).m*R0_2'*g_temp;
f_1 = R1*f_2 + L(1).m*ac_1 - L(1).m*R0_1'*g_temp;

tau_4 = [0;0;0];
tau_3 = -cross(f_3,r3_c3) + cross(R3*f_4,-r2_3/2) + L(3).I*alpha3 + cross(w_3, L(3).I*w_3);
tau_2 = R2*tau_3 - cross(f_2,r2_c2) + cross(R2*f_3,-r1_2/2) + L(2).I*alpha2 + cross(w_1, L(2).I*w_1);
tau_1 = R1*tau_2 - cross(f_1,r1_c1) + cross(R1*f_2,-r0_1/2) + L(1).I*alpha1 + cross(w_0, L(1).I*w_0);

tau = [tau_1(3) tau_2(3) tau_3(3)];
end