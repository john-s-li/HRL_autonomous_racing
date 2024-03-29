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

import casadi.*

%% Vehicle Model and Track Processing
disp('Loading Vehicle Parameters and Building Track ----------')
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

% Test the curvature function (check with Ugo's function: PASSED)
% for s = 1:17
%     str = ['k(',num2str(s),') = ', num2str(get_curvature(s,track))];
%     disp(str);
% end

%% Build the ODE f(x,u) for dx = f(x,u) for CASADI Opti() stack 
disp('Building CASADI ODE function object -----------')
functionBuilder;

%% Optimization Set Up
disp('Setting up CASADI Opti() Class ----------')

n = 6; % num_states
d = 2; % num_controls

dt = 0.1; % Discretization Time
N = 10; % NMPC Horizon

opti = Opti();

% Optimizer options
opts = struct;
opts.print_header = false;
opts.print_iteration = false;

opti.solver('ipopt')

% Variables for Opti() stack (these will be denotes in all caps)
X = opti.variable(n, N + 1); % States : [vx, vy, wz, e_psi, s, e_y]
U = opti.variable(d, N); % Controls : [delta, accel]

DELTA = U(1,:);
ACCEL = U(2,:);

VX = X(1,:);
VY = X(2,:);
WZ = X(3,:);    
E_PSI = X(4,:);
S = X(5,:);
E_LAT = X(6,:);

% RK4 integration for dynamics constraints
for i = 1:N
   k1 = f(X(:,i), U(:,i));   
   k2 = f(X(:,i) + dt/2*k1, U(:,i));
   k3 = f(X(:,i) + dt/2*k2, U(:,i));
   k4 = f(X(:,i) + dt*k3, U(:,i));
   x_next = X(:,i) + dt/6*(k1 + 2*k2 + 2*k3 +k4);
   opti.subject_to(X(:,i+1) == x_next)
end

% To increase feasability, can introduce slack variables (only for equality constraints)

% State constraints
opti.subject_to(-track.width <= E_LAT <= track.width);
opti.subject_to(-4 <= VX <= 4);
% TODO: Input an obstacle avoidance constraint later

% nput Box Constraints
opti.subject_to(-0.5 <= DELTA <= 0.5);
opti.subject_to(-1 <= ACCEL <= 1);

% Friction constraints
mu = 0.7; % avg friction coefficient for roads (assume rear wheel drive

% Redefine equations for optimization variable constraints
ALPHA_F = DELTA - atan( (VY(1:N) + lf * WZ(1:N)) ./ VX(1:N));
ALPHA_R = - atan( ( VY(1:N) - lf * WZ(1:N) ) ./ VX(1:N));

F_YF = F_nf*Df*sin( Cf * atan(Bf*ALPHA_F)); % Front Tire Lateral Force
F_YR = F_nr*Dr*sin( Cr * atan(Br*ALPHA_R)); % Rear Tire Lateral Force

opti.subject_to(F_YF.^2 <= (mu.*F_nf).^2);
opti.subject_to(F_YR.^2 + (ACCEL./2).^2 <= (mu.*F_nr).^2)

% Miscellaneous Constraints
DS = (VX.*cos(E_PSI)-VY.*sin(E_PSI))./(1 - get_curvature(S,track).*E_LAT);
opti.subject_to(DS > 0); % no going backwards

% Objective function
opti.minimize(-sum(S));

%% Run the simulation
disp('Running Simulation ---------')
% Initial conditions
x_curv = [1.0; 0.0; 0.0; 0.0; 0.0; 0.0]; % States : [vx, vy, wz, e_psi, s, e_y]
u_curv = [0.0; 0.0]; % Controls : [delta, accel]

x_log = x_curv;
u_log = u_curv;

print_debug = 0;

c_opt = 0;

while (x_curv(5) <= track.trackLength)
    
    x0 = repmat(x_curv,1,N+1);
    u0 = repmat(u_curv,1,N);
    
    opti.set_initial(X, x0); % Bicylce Model and Pacejka Tyre model ill-defined for slow velocities
    opti.set_initial(U, u0);
    opti.subject_to(X(:,1) == x0(:,1));
    %opti.subject_to(U(:,1) == u0(:,1));
    
    % NOTE: need all IC lines. Opti() is weird in this sense...
     
    % These below are helpful debugging commands
    % opti.callback(@(i) display(opti.debug.value(X))) % Print out values @ each iteration
    % opti.debug.g_describe(7) % This is to help look at infeasibilities
    
    sol = opti.solve();
    
    if print_debug
        opti.debug.value(X);
        opti.debug.value(U);
    end
    
    x_opt = sol.value(X);
    u_opt = sol.value(U);
    
    c_opt = c_opt + 1;
    str = ['Optimizations complete: ', num2str(c_opt)];
    disp(str)
    
    % Extract the first optimal control input
    u_NMPC = u_opt(:,1);
    u_log = [u_log, u_NMPC]; 
    
    % Apply optimal control to system
    x_curv = vehicleSim(x_curv, u_NMPC, dt, vehParams, track);
    x_log = [x_log, x_curv];
    
    % Update the optimal control for next otpimization sequence
    u_curv = u_opt(:,1); 
  
end
disp('Race Completed ----------')
