%% rpo_setup.m
% RPO GNC simulation parameters
% Frame: LVLH
% x: radial, y: along-track, z: cross-track
% Units: m, m/s, m/s^2, s

%% 1. Orbit and CW dynamics

mu = 3.986004418e14;
Re = 6378137;
rt = Re + 450e3;

n = sqrt(mu/rt^3);
T = 2*pi/n;

A = [0       0    0     1     0   0;
     0       0    0     0     1   0;
     0       0    0     0     0   1;
     3*n^2   0    0     0   2*n   0;
     0       0    0  -2*n     0   0;
     0       0 -n^2    0     0   0];

B = [zeros(3,3); eye(3)];

%% 2. LQR hold controller

Qw = diag([ones(1,3)/100^2, ones(1,3)/0.5^2]);
Rw = eye(3)/0.01^2;

K = lqr(A,B,Qw,Rw);

%% 3. Sensor and navigation parameters

dt = 1;

sigma_z = 1;
q = 1e-9;
thr_err = 0.02;

P0pos = 5;
P0vel = 0.05;

%% 4. Discrete Kalman filter

M = expm([A B; zeros(3,9)]*dt);

Phi = M(1:6,1:6);
Gam = M(1:6,7:9);

Qc = B*(q*eye(3))*B';

V = expm([-A Qc; zeros(6) A']*dt);

Qd = V(7:12,7:12)'*V(1:6,7:12);
Qd = (Qd + Qd')/2;

H = [eye(3), zeros(3)];

R = sigma_z^2*eye(3);

P0 = diag([P0pos^2*ones(1,3), ...
           P0vel^2*ones(1,3)]);

%% 5. Rendezvous guidance

rf = [0; -1000; 0];
vf = zeros(3,1);

X_hold = [rf; vf];

t_burn = 100;
t_burn0 = 60;

t_tr = 0.75*T;
t_transfer = t_tr;

% Planned burn timing
% Transfer time measured between burn midpoints

t_depart = t_burn0 + t_burn/2;
t_arrival = t_depart + t_tr;

% First burn
t_burn1_start = t_burn0;
t_burn1_end = t_burn1_start + t_burn;

% Midcourse burn, centered halfway through transfer
t_mid = t_depart + t_tr/2;

t_mid_start = t_mid - t_burn/2;
t_mid_end = t_mid + t_burn/2;

% Arrival burn
t_burn2_start = t_arrival - t_burn/2;
t_burn2_end = t_arrival + t_burn/2;

% CW targeting conditioning check
if abs(sin(n*t_tr)) < 0.2
    warning('Transfer duration may be near a CW targeting singularity.');
end

%% 6. Initial conditions

if isfile('handoff.mat')
    S = load('handoff.mat','x0');
    x0 = S.x0;
else
    x0 = [0; -10000; 0; 0; 0; 0];
end

rng(1);

xh0 = x0 + chol(P0,'lower')*randn(6,1);

%% 7. Simulation settings

t_end = t_burn2_end + 4000;

%% 8. Display mission timing

fprintf('\nRPO guidance timing:\n');
fprintf('Orbital period:       %.1f s\n',T);
fprintf('Transfer duration:    %.1f s\n',t_tr);
fprintf('Burn duration:        %.1f s\n',t_burn);
fprintf('First burn start:     %.1f s\n',t_burn1_start);
fprintf('Midcourse burn start: %.1f s\n',t_mid_start);
fprintf('Arrival burn start:   %.1f s\n',t_burn2_start);
fprintf('Simulation end:       %.1f s\n\n',t_end);