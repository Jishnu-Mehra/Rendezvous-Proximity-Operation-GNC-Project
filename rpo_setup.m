%% rpo_setup.m  --  RUN THIS FIRST (or set it as the Simulink model's InitFcn callback)
% Creates every parameter used by the scripts AND the Simulink blocks in the base
% workspace. One place to change a number = it changes everywhere.
% Frame: LVLH, x radial, y along-track, z cross-track.  Units: m, m/s, m/s^2, s.

%% Orbit and CW design model
mu = 3.986004418e14;  Re = 6378137;  rt = Re + 450e3;
n  = sqrt(mu/rt^3);   T  = 2*pi/n;
A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1;
     3*n^2 0 0 0 2*n 0; 0 0 0 -2*n 0 0; 0 0 -n^2 0 0 0];
B = [zeros(3); eye(3)];

%% LQR hold controller (week 1)
Qw = diag([1/100^2*ones(1,3), 1/0.5^2*ones(1,3)]);
Rw = eye(3)/0.01^2;
K  = lqr(A, B, Qw, Rw);

%% Sensor, noise, sample time, truth thrust error (week 2)
dt = 1;            % s   sensor + filter + control sample period
sigma_z = 1;       % m   relative-position noise (placeholder: cite a source later)
q = 1e-9;          % (m/s^2)^2*s  white-acceleration PSD used for Q
thr_err = 0.02;    % truth thrust gain error, unknown to the filter
P0pos = 5;  P0vel = 0.05;

%% Kalman filter matrices (discrete)
M   = expm([A B; zeros(3,9)]*dt);
Phi = M(1:6,1:6);  Gam = M(1:6,7:9);                 % zero-order-hold discretization
Qc  = B*(q*eye(3))*B';
V   = expm([-A Qc; zeros(6) A']*dt);                 % Van Loan
Qd  = (V(7:12,7:12)')*V(1:6,7:12);  Qd = (Qd + Qd')/2;
H   = [eye(3) zeros(3)];                             % position-only sensor
R   = sigma_z^2*eye(3);
P0  = diag([P0pos^2*ones(1,3), P0vel^2*ones(1,3)]);

%% Guidance parameters (week 3)
Tb       = 30;          % s   burn pulse length (a = dv/Tb) until the week-6 thruster model
t_burn0  = 60;          % s   centre of the first burn
t_tr     = 0.75*T;      % s   CW transfer time. AVOID multiples of T/2 (singular, see cw_two_impulse)
rf = [0; -1000; 0];     % m   target point: 1 km behind the target on V-bar
vf = zeros(3,1);
if abs(sin(n*t_tr)) < 0.2
    warning('t_tr is close to a singular CW transfer time (multiple of T/2).');
end

%% Initial condition: Phase 0 handoff if available, otherwise ideal 10 km V-bar
if isfile('handoff.mat')
    S = load('handoff.mat', 'x0');  x0 = S.x0;
else
    x0 = [0; -10000; 0; 0; 0; 0];
end
rng(1);
xh0 = x0 + chol(P0,'lower')*randn(6,1);              % initial estimate = truth + error
t_end = t_burn0 + t_tr + Tb/2 + 4000;                % s   simulation length
