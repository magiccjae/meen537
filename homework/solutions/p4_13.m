%after running startup_rvc.m
syms psi phi theta d_psi d_theta d_phi

%defining intermediate variables for rotation
Rz1 = rotz(psi);
Ry = roty(theta);
Rz2 = rotz(phi);
R = Rz1*Ry*Rz2;

%taking the analytical derivative using the product and chain rule
dR_dt = diff(R, psi)*d_psi + diff(R, theta)*d_theta + diff(R, phi)*d_phi

%omega as defined by the book
omega = [0; 0; d_psi]+Rz1*[0; d_theta; 0]+ Rz1*Ry*[0; 0; d_phi]

%using omega to calculate the derivative of the rotation matrix
dR_dt_test = skew(omega)*R

%if the two are equal, we should be able to subtract them and simplify
simplify(dR_dt - dR_dt_test)

