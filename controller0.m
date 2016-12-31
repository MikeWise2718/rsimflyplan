function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

m = params.mass;
g = params.gravity; 
l = params.arm_length;
I = params.I;
Iinv = params.invI;


% Thrust


% Calculate p,q,r (body frame ang vel)
% from state.rot and state.omega
phi = state.rot(1);
tht = state.rot(2);
psi = state.rot(3);
dphi = state.omega(1);
dtht = state.omega(2);
dpsi = state.omega(3);

kpv = 100;
kdv = 20;
kav = 0.5;

kpt = 100;
kdt = 1;

erv = des_state.vel-state.vel;
erp = des_state.pos-state.pos;

ddr1des = kav*des_state.acc(1) + kdv*erv(1) + kpv*erp(1);
ddr2des = kav*des_state.acc(2) + kdv*erv(2) + kpv*erp(2);
ddr3des = kav*des_state.acc(3) + kdv*erv(3) + kpv*erp(3);

p = dphi - tht*dpsi;
q = dtht + phi*dpsi;
r = dpsi - tht*dphi;

% calc phides,thetades from des_state.acc and  state.rot
phides = (1/g)*(ddr1des*sin(psi) - ddr2des*cos(psi));
thtdes = (1/g)*(ddr1des*cos(psi) + ddr2des*sin(psi));
psides = des_state.yaw;

% see page 6, eq 14a and 14b
% calc pdes,qdes,rdes from page 6 eq 15a,15b,16a,16b
pdes = 0;
qdes = 0;
rdes = des_state.yawdot;

% Now get this from pg6. eq 13 (or before maybe
F = m*g + m*ddr3des;



u21 = kpt*(phides-phi) + kdt*(pdes-p);
u22 = kpt*(thtdes-tht) + kdt*(qdes-q);
u23 = kpt*(psides-psi) + kdt*(rdes-r);

% Moment
% Now get this from p5 equ 10.
M = [u21;u22;u23];

% =================== Your code ends here ===================

end
