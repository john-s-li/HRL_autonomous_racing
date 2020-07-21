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

syms s e_lat e_psi v k accel delta

x = [s; e_lat; e_psi; v];
u = [accel; delta];

beta = atan((lr/(lf+lr))*tan(delta));

ds = v*(cos(e_psi + beta))/(1 - e_lat*k);
de_lat = v*sin(e_psi + beta);
de_psi = (v/lf)*sin(beta) - k*ds;
dv = accel;

f = [ds;
      de_lat;
      de_psi;
      dv];

A = jacobian(f,x);
B = jacobian(f,u);

% generate the functions and save them
if ~exist('gen','dir')
   mkdir('gen') 
end

matlabFunction(A, 'File', 'gen/A_gen', 'Vars', {x,u,k});
matlabFunction(B, 'File', 'gen/B_gen', 'Vars', {x,u,k});




