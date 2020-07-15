%% dynamicsTest.m
% Script to ensure that curvilinear dynamics are okay
clear
clc
close all

addpath('Classes')
addpath('Utilities')

vehicleParams; % Load vehicle params
track = Track(0.8); % Make a track object

vx_ref = 0.8;

tspan = [0 20];
x0 = [1.0; 0; 0; 0.2; 0; 0.3]; 

opts = odeset('RelTol', 1e-3);

[t,x] = ode45(@(t,x)curv_bicycle_dyn(t,x,vx_ref,lr,lf,m,g,Bf,Br,Cf,Cr,Df,Dr,Iz, track), ...
                                     tspan, x0, opts);

vx = x(:,1);
vy = x(:,2);
wz = x(:,3);
e_psi = x(:,4);
s = x(:,5);
e_lat = x(:,6);

plot(t,vx,'DisplayName','$v_x$')
hold on
plot(t,vy,'DisplayName','$v_y$')
plot(t,wz,'DisplayName','$\omega_z$')
plot(t,s,'DisplayName','$s$')
plot(t,e_lat,'DisplayName','$e_{y}$')
plot(t,e_psi,'DisplayName','$e_{\psi}$')
hold off
xlabel('time $t$','Interpreter','latex')
title('Dynamic Bicycle Dynamics with PID Control','Interpreter','latex')
legend('show','Interpreter','latex','FontSize',14)
grid on

function [dx] = curv_bicycle_dyn(t,x,vx_ref,lr,lf,m,g,Bf,Br,Cf,Cr,Df,Dr,Iz,track)
    vx = x(1); % velocity in x-axis wrt/vehicle frame
    vy = x(2); % velocity in y-axis wrt/vehicle frame
    wz = x(3); % yaw rate wrt/track frame
    e_psi = x(4); % headeing error wrt/track frame
    s = x(5); % arc length on centerline
    e_lat = x(6); % later error wrt/track centerline

    % PID Control -----------------------------------------------
    K_v = 1.5;
    K_psi = 0.6;
    K_lat = 0.9;
    
    delta = -K_psi*(e_psi) - K_lat*(e_lat); % Drive to zero
    accel = K_v*(vx_ref - vx);
    % -----------------------------------------------------------
    
    % Tire Split Angles (Negative due to our coordinate frame)
    alpha_f = delta - atan( (vy + lf * wz) / vx );
    alpha_r = - atan( (vy - lf * wz) / vx);

    F_nf = lf/(lf+lr)*m*g; % Front Tire Normal Load
    F_nr = lr/(lf+lr)*m*g; % Rear Tire Normal Load

    F_yf = F_nf*Df*sin( Cf * atan(Bf*alpha_f)); % Front Tire Lateral Force
    F_yr = F_nr*Dr*sin( Cr * atan(Br*alpha_r)); % Rear Tire Lateral Force

    % ODEs
    % NOTE: for now, keep k(s) constant (sym variables don't work with logical operators)

    dvx = 1/m*(m*accel - F_yf*sin(delta) + m*(wz*vy));
    dvy = 1/m*(F_yr + F_yf*sin(delta) - m*vx(1)*wz);
    dwz = 1/Iz*(F_yf*lf*cos(delta) - F_yr*lr); % No torque vectoring 
    ds = (vx*cos(e_psi)-vy*sin(e_psi))/(1 - get_curvature(s,track)*e_lat);
    de_psi = wz - get_curvature(s,track)*ds;
    de_lat = vx*sin(e_psi) + vy*cos(e_psi);
    
    dx = [dvx;
          dvy;
          dwz;
          de_psi;
          ds;
          de_lat];
      
    % str = ['ODE call done and t = ', num2str(t)];
    % disp(str)
end