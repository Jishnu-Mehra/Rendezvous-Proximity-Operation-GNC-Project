function [dv1, dv2] = cw_target_test(x0, xf, tf, A)

% Initial state
r0 = x0(1:3);
v0 = x0(4:6);

% Desired final state
rf = xf(1:3);
vf = xf(4:6);

% CW state transition matrix
Phit = expm(A*tf);

Prr = Phit(1:3,1:3);
Prv = Phit(1:3,4:6);
Pvr = Phit(4:6,1:3);
Pvv = Phit(4:6,4:6);

% Required departure velocity
% rf = Prr*r0 + Prv*v_depart
v_depart = Prv \ (rf - Prr*r0);

% First burn: dv1 = v_depart - v0
dv1 = v_depart - v0;

% Predicted arrival velocity
% v_arrive = Pvr*r0 + Pvv*v_depart
v_arrive = Pvr*r0 + Pvv*v_depart;

% Final burn: dv2 = vf - v_arrive
dv2 = vf - v_arrive;

end