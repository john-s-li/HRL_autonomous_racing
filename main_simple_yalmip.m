%% main_simple_yalmip

clc;
clear all;
close all;

% addpath
addpath('Classes')
addpath('Utilities')

% size for system state and input
nX = 4;
nU = 2;

% simulation time
M = 10 ;

% MPC horizon 
N = 5;

% discrete time
dt = 0.2;

% constraint & cost
constraints = [];
cost = 0;

% Weight matrix
Q = -1;
R = eye(2);

% closed-loop trajectory
x_nmpc = zeros(nX,M+1);
u_nmpc = zeros(nU,M);

% initial condition 
x0 = [0; 0; 0; 1]; 

% assign initial condition
x_nmpc(:,1) = x0;

for i = 1:M
    x0 = x_nmpc(:,i);
    
    [feas, x_ftoc, u_ftoc] = solve_ftoc(Q, R, N, nX, nU, x0, dt);  
    
    x_nmpc(:,i+1) = x_ftoc(:,2);
    u_nmpc(:,i) = u_ftoc(:,1);
    
end


