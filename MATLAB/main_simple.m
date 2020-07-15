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
vehParams = vehicleParams();
track = Track(0.8); % initialize the track with width

g =  vehParams.g;
m  = vehParams.m;
lf = vehParams.lf;
lr = vehParams.lr;
Iz = vehParams.Iz;
Df = vehParams.Df;
Cf = vehParams.Cf;
Bf = vehParams.Bf;
Dr = vehParams.Dr;
Cr = vehParams.Cr;
Br = vehParams.Br;

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
k = 1/-4; % curvature for a circle with constant radius = 4 (Counter-CW)

ds = v*(cos(e_psi + beta))/(1 - e_lat*k);
de_lat = v*sin(e_psi + beta);
de_psi = (v/lf)*sin(beta) - k*ds;
dv = accel;

f_vec = [ds; de_lat; de_psi; dv];
f = Function('f',{x,u},{f_vec}); % Build CASADI function object

%% Dynamics Constraints
dt = 0.1;

for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = f(X(:,k),         U(:,k));
   k2 = f(X(:,k)+dt/2*k1, U(:,k));
   k3 = f(X(:,k)+dt/2*k2, U(:,k));
   k4 = f(X(:,k)+dt*k3,   U(:,k));
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

%% State Constraints
opti.subject_to(-track.width <= E_LAT <= track.width);
% opti.subject_to(-4 <= V <= 4); 

%% Input Box Constraints
opti.subject_to(-0.5 <= DELTA <= 0.5);
opti.subject_to(-1 <= ACCEL <= 1);

%% Miscellaneous Constraints
BETA = atan((lr/(lf+lr))*tan(DELTA));
DS = V(1:N).*(cos(E_PSI(1:N) + BETA))./(1 - E_LAT(1:N)*k);

opti.subject_to(DS > 0); % no going backwards
opti.subject_to(ACCEL >= 0);

%% Initial Conditions
opti.set_initial(X(:,1),[0.0;0.0;0.0;1.0]); % Bicycle Model ill-defined for slow velocities
opti.subject_to(X(:,1) == [0.0;0.0;0.0;1.0]); % Must have this to make sure first column does NOT vary

%% Objective function
opti.minimize(-sum(S));

%% Run the simulation
x_curv = [0.0; 0.0; 0.0; 1.0];
u_curv = [0.0; 0.0];

x_log = x_curv;
u_log = u_curv;

print_debug = 0;

c_opt = 0;

x0 = repmat(x_curv,1,N+1);
u0 = repmat(u_curv,1,N);

while (x_curv(1) <= track.trackLength)
    
    opti.set_initial(X, x0); % Bicylce Model and Pacejka Tyre model ill-defined 
                             % for slow velocities
    opti.set_initial(U, u0);
    opti.subject_to(X(:,1) == x0(:,1));
    %opti.subject_to(U(:,1) == u0(:,1));
    
    % NOTE: need all IC lines. Opti() is weird in this sense...
     
    % These below are helpful debugging commands
    % opti.callback(@(i) display(opti.debug.value(X))) % Print out values @ each iteration
    % opti.debug.g_describe(7) % This is to help look at infeasibilities
    
    opti.solver('ipopt');
    sol = opti.solve();
    
    if print_debug
        opti.debug.value(X);
        opti.debug.value(U);
    end
    
    x_opt = sol.value(X);
    u_opt = sol.value(U)
    
    c_opt = c_opt + 1;
    str = ['Optimizations complete: ', num2str(c_opt)];
    disp(str)
    
    % Extract the first optimal control input
    u_NMPC = u_opt(:,1);
    u_log = [u_log, u_NMPC]; 
    
    % Apply optimal control to system
    x_curv = vehicleSim(x_curv, u_NMPC, dt, vehParams, track);
    x_log = [x_log, x_curv];
        
    x0 = x_opt;
    u0 = u_opt;
end

% % opti.callback(@(i) plot(opti.debug.value(S)))
% opti.callback(@(i) display(opti.debug.value(X)))
% sol = opti.solve();
% opti.debug.value(X)
% opti.debug.value(U)

disp('Race Complete')

