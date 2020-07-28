%% computeJacobianFunctions.m
% Function to generate MATLAB functions for the linearized dynamics

clc
clear
close all

addpath('Classes')

% load vehicle parameters
vehParams = vehicleParams();
classTrack = Track(0.8);

lf = vehParams.lf; 
lr = vehParams.lr;
Bf = vehParams.Bf;
Br = vehParams.Br;
Cf = vehParams.Cf;
Cr = vehParams.Cr;
Df = vehParams.Df;
Dr = vehParams.Dr;
Iz = vehParams.Iz;
m = vehParams.m;
g = vehParams.g;

syms vx vy wz s e_lat e_psi k accel delta

x = [vx; vy; wz; s; e_lat; e_psi];
u = [accel; delta];

[F_yf, F_yr, ~ , ~ ] = tireSplitForces(x,u,vehParams);

dvx = 1/m*(m*accel - F_yf*sin(delta) + m*(wz*vy));
dvy = 1/m*(F_yr + F_yf*cos(delta) - m*vx*wz);
dwz = 1/Iz*(F_yf*lf*cos(delta) - F_yr*lr); 
ds = (vx*cos(e_psi)-vy*sin(e_psi))/(1 - k*e_lat);
de_lat = vx*sin(e_psi) + vy*cos(e_psi);
de_psi = wz - k*ds;

f = [dvx;
     dvy;
     dwz;
     ds;
     de_lat;
     de_psi];

A = jacobian(f,x);
B = jacobian(f,u);

% generate the functions and save them
if ~exist('gen','dir')
   mkdir('gen') 
end

matlabFunction(A, 'File', 'gen/A_gen', 'Vars', {x,u,k});
matlabFunction(B, 'File', 'gen/B_gen', 'Vars', {x,u,k});