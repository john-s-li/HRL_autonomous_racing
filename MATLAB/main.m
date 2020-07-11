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

% Test the curvature function (check with Ugo's function: PASS)
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

x = MX.sym('x',n, N); % States : [vx, vy, wz, e_psi, s, e_y] , should be N+1 but CASADI complains..
u = MX.sym('u',d, N); % Controls : [delta, accel]

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

% ISSUE: ds, de_psi, and de_lat depend on curvature k(s)
% when trying to use a function to do k(s), does not work with CASADI MX.sym

dvx = 1/m.*(accel - F_yf.*sin(delta) + m.*(wz.*vy));
dvy = 1/m.*(F_yr + F_yf.*sin(delta) - m.*vx.*wz);
dwz = 1/Iz.*(F_fy.*lf.*cos(delta) - F_yr.*lr); % No torque vectoring 
de_psi = wz - (vx.*cos(e_psi) - vy.*sin(e_psi))/(1-get_curvature(s,track)*e_lat)*get_curvature(s,track);
ds = (vx.*cos(e_psi)-vy.*sin(e_psi))./(1 - get_curvature(s,track)*e_lat);
de_lat = vx.*sin(e_psi) + vy.*cos(e_psi);

ode = [dvx; dvy; dwz; de_psi; ds; de_lat];

f = Function('f',{x,u},{ode});

% Jun's Idea: make k(s) constant in open loop MPC
% So...what I can do...optimize the system with constant k(s), apply
% optimal control in simulation, then see where vehicle is, then change
% k(s) as needed (this will require some projection onto the
% centerline...see Liniger's code for this)




























