%% functionBuilder.m
% File to build the CASADI function object
import casadi.*

n = 6; % num_states
d = 2; % num_controls

x = MX.sym('x',n,1);
u = MX.sym('u',d,1);

delta = u(1);
accel = u(2);

vx = x(1); % velocity in x-axis wrt/vehicle frame
vy = x(2); % velocity in y-axis wrt/vehicle frame
wz = x(3); % yaw rate wrt/track frame
e_psi = x(4); % headeing error wrt/track frame
s = x(5); % arc length on centerline
e_lat = x(6); % lateral error wrt/track centerline

% Tire Split Angles (Negative due to our coordinate frame)
alpha_f = delta - atan( (vy + lf * wz) / vx );
alpha_r = - atan( (vy - lf * wz) / vx);

F_nf = lf/(lf+lr)*m*g; % Front Tire Normal Load
F_nr = lr/(lf+lr)*m*g; % Rear Tire Normal Load

F_yf = F_nf*Df*sin(Cf * atan(Bf*alpha_f)); % Front Tire Lateral Force
F_yr = F_nr*Dr*sin(Cr * atan(Br*alpha_r)); % Rear Tire Lateral Force

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