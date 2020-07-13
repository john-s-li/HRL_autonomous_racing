%% main.m
% Hybrid Robotics Lab
% Author : Johnathon Li

clear
close all
clc

% Path Appending --------------------------------------
addpath('apiFiles/casadi-linux-matlabR2014b-v3.5.1')
addpath('Classes')
addpath('Utilities')

%% Vehicle Model and Track Processing
vehicleParams;
track = Track(0.8); % initialize the track with width

% Test the curvature function (check with Ugo's function: PASSED)
% for s = 1:17
%     str = ['k(',num2str(s),') = ', num2str(get_curvature(s,track))];
%     disp(str);
% end

%% Optimization Set Up

% Library Imports -------------------------------------
import casadi.*

n = 6; % num_states
d = 2; % num_controls

N = 10; % NMPC Horizon

opti = Opti();

% x = MX.sym('x',n, N); % States : [vx, vy, wz, e_psi, s, e_y] , should be N+1 but CASADI complains..
% u = MX.sym('u',d, N); % Controls : [delta, accel]

x = opti.variable(6, N);
u = opti.variable(2,N);
T = opti.variable(); % The final time (goal is to minimize over horizon)

delta = u(1,:);
accel = u(2,:);

vx = x(1,:); % velocity in x-axis wrt/vehicle frame
vy = x(2,:); % velocity in y-axis wrt/vehicle frame
wz = x(3,:); % yaw rate wrt/track frame
e_psi = x(4,:); % headeing error wrt/track frame
s = x(5,:); % arc length on centerline
e_lat = x(6,:); % later error wrt/track centerline

%% Define the ODE f(x,u) for dx = f(x,u)

% Tire Split Angles (Negative due to our coordinate frame)
alpha_f = delta - atan2( vy + lf * wz, vx );
alpha_r = -atan2( vy - lf * wz , vx);

F_nf = lf/(lf+lr)*m*g; % Front Tire Normal Load
F_nr = lr/(lf+lr)*m*g; % Rear Tire Normal Load

F_yf = F_nf*Df*sin( Cf * atan2(1, Bf*alpha_f)); % Front Tire Lateral Force
F_yr = F_nr*Dr*sin( Cr * atan2(1, Br*alpha_r)); % Rear Tire Lateral Force

% ODEs
% NOTE: for now, keep k(s) constant (sym variables don't work with logical operators)

dvx = 1/m.*(accel - F_yf.*sin(delta) + m.*(wz.*vy));
dvy = 1/m.*(F_yr + F_yf.*sin(delta) - m.*vx.*wz);
dwz = 1/Iz.*(F_yf.*lf.*cos(delta) - F_yr.*lr); % No torque vectoring 
de_psi = wz - (vx.*cos(e_psi) - vy.*sin(e_psi))./(1-get_curvature(s,track)*e_lat)*get_curvature(s,track);
ds = (vx.*cos(e_psi)-vy.*sin(e_psi))./(1 - get_curvature(s,track)*e_lat);
de_lat = vx.*sin(e_psi) + vy.*cos(e_psi);

f = @(x,u) [dvx; dvy; dwz; de_psi; ds; de_lat];

%% Define constraints
dt = T/N;

% RK4 integration
s_tot = 0;
for i = 1:N-1
   k1 = f(x(:,i), u(:,i));
   k2 = f(x(:,i) + dt/2*k1, u(:,i));
   k3 = f(x(:,i) + dt/2*k2, u(:,i));
   k4 = f(x(:,i) + dt*k3, u(:,i));
   x_next = x(:,i) + dt/6*(k1 + 2*k2 + 2*k3 +k4);
   opti.subject_to(x(:,i+1) == x_next)
   s_tot = s_tot + x(5,i);
end

% Path constraints
opti.subject_to(e_lat <= track.width);
% Input an obstacle avoidance constraint later

% Friction constraints
mu = 0.7; % avg friction coefficient for roads (assume rear wheel drive
opti.subject_to(F_yf.^2 <= (mu.*F_nf).^2);
opti.subject_to(F_yr.^2 + (accel./2).^2 <= (mu.*F_nr).^2)

% Input Box Constraints
opti.subject_to(-0.5 <= delta <= 0.5);
opti.subject_to(-1 <= accel <= 1);

% Boundary constraints
% opti.subject_to(x(:,1) == 0); % Initial Conditions
opti.set_initial(x,0);

% Miscellaneous Constraints
opti.subject_to(T >= 0);
opti.subject_to(ds >= 0); % no going backwards

% Objective function
opti.minimize(-s_tot)

%% Optimization 
opti.solver('ipopt');

sol = opti.solve();




















