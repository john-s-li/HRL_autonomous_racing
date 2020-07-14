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

N = 5; % NMPC Horizon

opti = Opti();

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

%% Build the ODE f(x,u) for dx = f(x,u) for CASADI Opti() stack 

x = MX.sym('x',n,1);
u = MX.sym('u',d,1);

delta = u(1);
accel = u(2);

vx = x(1); % velocity in x-axis wrt/vehicle frame
vy = x(2); % velocity in y-axis wrt/vehicle frame
wz = x(3); % yaw rate wrt/track frame
e_psi = x(4); % heading error wrt/track frame
s = x(5); % arc length on centerline
e_lat = x(6); % later error wrt/track centerline

% Tire Split Angles (Negative due to our coordinate frame)
alpha_f = - delta + atan2( vy + lf * wz, vx );
alpha_r = - atan2( vy - lf * wz , vx);

F_nf = lf/(lf+lr)*m*g; % Front Tire Normal Load
F_nr = lr/(lf+lr)*m*g; % Rear Tire Normal Load

F_yf = F_nf*Df*sin( Cf * atan2(1, Bf*alpha_f)); % Front Tire Lateral Force
F_yr = F_nr*Dr*sin( Cr * atan2(1, Br*alpha_r)); % Rear Tire Lateral Force

% ODEs
% NOTE: for now, keep k(s) constant (sym variables don't work with logical operators)

dvx = 1/m*(m*accel - F_yf*sin(delta) + m*(wz*vy));
dvy = 1/m*(F_yr + F_yf*sin(delta) - m*vx(1)*wz);
dwz = 1/Iz*(F_yf*lf*cos(delta) - F_yr*lr); % No torque vectoring 
ds = (vx*cos(e_psi)-vy*sin(e_psi))/(1 - get_curvature(s,track)*e_lat);
de_psi = wz - get_curvature(s,track)*ds;
de_lat = vx*sin(e_psi) + vy*cos(e_psi);

f_vec = [dvx; dvy; dwz; de_psi; ds; de_lat];
f = Function('f',{x,u},{f_vec}); % Build CASADI function object

%% Dynamics Constraints
dt = 0.1;

% RK4 integration for dynamics constraints
for i = 1:N-1
   k1 = f(X(:,i), U(:,i));   
   k2 = f(X(:,i) + dt/2*k1, U(:,i));
   k3 = f(X(:,i) + dt/2*k2, U(:,i));
   k4 = f(X(:,i) + dt*k3, U(:,i));
   x_next = X(:,i) + dt/6*(k1 + 2*k2 + 2*k3 +k4);
   opti.subject_to(X(:,i+1) == x_next)
end

%% Path constraints
%opti.subject_to(E_LAT <= track.width);
% Input an obstacle avoidance constraint later

%% Friction constraints
mu = 0.7; % avg friction coefficient for roads (assume rear wheel drive

ALPHA_F = - DELTA + atan2( VY(1:N) + lf * WZ(1:N), VX(1:N));
ALPHA_R = atan2( VY(1:N) - lf * WZ(1:N) , VX(1:N));

F_YF = F_nf*Df*sin( Cf * atan2(1, Bf*ALPHA_F)); % Front Tire Lateral Force
F_YR = F_nr*Dr*sin( Cr * atan2(1, Br*ALPHA_R)); % Rear Tire Lateral Force

opti.subject_to(F_YF.^2 <= (mu.*F_nf).^2);
opti.subject_to(F_YR.^2 + (ACCEL./2).^2 <= (mu.*F_nr).^2)

%% State Constraints
opti.subject_to(-2 <= E_LAT <= 2);
opti.subject_to(-2 <= VX <= 2);

opti.subject_to(-1 <= VY <= 1);

%% Input Box Constraints
opti.subject_to(-0.5 <= DELTA <= 0.5);
opti.subject_to(-1 <= ACCEL <= 1);

%% Miscellaneous Constraints
DS = (VX.*cos(E_PSI)-VY.*sin(E_PSI))./(1 - get_curvature(S,track).*E_LAT);

opti.subject_to(S >= 0); 
opti.subject_to(DS > 0); % no going backwards

%% Initial Conditions
opti.set_initial(VX, 1.0); % Bicylce Model and Pacejka Tyre model ill-defined for slow velocities

%% Objective function
opti.minimize(-S(end));

%% Optimization 
opti.solver('ipopt');
% opti.callback(@(i) plot(opti.debug.value(S)))
opti.callback(@(i) display(opti.debug.value(X)))
sol = opti.solve();



















