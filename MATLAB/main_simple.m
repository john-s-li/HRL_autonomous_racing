%% main.m
% Hybrid Robotics Lab
% Author : Johnathon Li

clear
close all
clc

% Path Appending --------------------------------------
addpath('apiFiles/casadi-linux-matlabR2014b-v3.5.1') % THIS IS ONLY FOR UBUNTU
addpath('Classes')
addpath('Utilities')

%% Vehicle Model and Track Processing
vehicleParams;
track = Track(0.8); % initialize the track with width

%% Optimization Set Up
% NOTE: use bicycle dynamics in curvilinear frame (no tire model in dynamics)
% Reference: Brunke M.S. Disseration

% Library Imports -------------------------------------
import casadi.*

n = 4; % num_states
d = 2; % num_controls

N = 10; % NMPC Horizon

opti = Opti();

% Variables for Opti() stack (these will be denotes in all caps)
X = opti.variable(n, N + 1); % States : [s, e_lat, e_psi, v]
U = opti.variable(d, N); % Controls : [accel, delta]

ACCEL = U(1,:);
DELTA = U(2,:);
    
S = X(1,:);
E_LAT = X(2,:);
E_PSI = X(3,:);
V = X(4,:);

%% Build the ODE f(x,u) for dx = f(x,u) for CASADI Opti() stack 

x = MX.sym('x',n,1);
u = MX.sym('u',d,1);

accel = u(1);
delta = u(2);

s = x(1);
e_lat = x(2);
e_psi = x(3);
v = x(4);

% Define the ODEs
beta = atan((lr/(lf+lr))*tan(delta));
k = 1/4; % curvature for a circle with constant radius = 4 (Counter-CW)

ds = v*(cos(e_psi + beta))/(1 - e_lat*k);
de_lat = v*sin(e_psi + beta);
de_psi = (v/lf)*sin(beta) - k*ds;
dv = accel;

f_vec = [ds; de_lat; de_psi; dv];
f = Function('f',{x,u},{f_vec}); % Build CASADI function object

%% Dynamics Constraints
dt = 0.1;

% Create an integrator to discretize the system
intg_options = struct;
intg_options.tf = dt;
intg_options.simplify = true;
intg_options.number_of_finite_elements = 4; % not sure what this is...

% DAE Problem structure
dae = struct;
dae.x = x;
dae.p = u;
dae.ode = f(x,u);

intg = integrator('intg','rk', dae, intg_options);

x0 = [0; 0; 0; 1.0]; % Initial Conditions

res = intg('x0',x0);
x_next = res.xf;

F = Function('F',{x,u},{x_next},{'x','u'},{'x_next'});

for i=1:N
   opti.subject_to(X(:,i+1) == F(X(:,i),U(:,i))); 
end

%% State Constraints
opti.subject_to(-track.width <= E_LAT <= track.width);
opti.subject_to(-2 <= V <= 2); 

%% Input Box Constraints
opti.subject_to(-0.5 <= DELTA <= 0.5);
opti.subject_to(-1 <= ACCEL <= 1);

%% Miscellaneous Constraints
BETA = atan((lr/(lf+lr))*tan(DELTA));
DS = V(1:N).*(cos(E_PSI(1:N) + BETA))./(1 - E_LAT(1:N)*k);

opti.subject_to(DS > 0); % no going backwards

%% Initial Conditions
opti.set_initial(V(1), 0.5); % Bicycle Model ill-defined for slow velocities

%% Objective function
opti.minimize(-S(end));

%% Optimization 
opti.solver('ipopt');
% opti.callback(@(i) plot(opti.debug.value(S)))
opti.callback(@(i) display(opti.debug.value(X)))
sol = opti.solve();
sol.value(X)
sol.value(U)