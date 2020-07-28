clc; clear; close all;

user_is_john = 1;

if user_is_john
    % Add YALMIP and IPOPT Paths (for John)...comment out if necessary
    addpath(genpath('/home/johnathon/Documents/MATLAB/tbxmanager/toolboxes/yalmip/R20200116/all/YALMIP-master'))
    addpath(genpath('/home/johnathon/Documents/MATLAB/Ipopt'))
end

% Mandatory path appending
addpath('Classes')
addpath('Utilities')
addpath('gen')

% Load vehicle and racetrack parameters
vehParams = vehicleParams();
classTrack = Track(0.8);

% Size of state and controls
nX = 4;
nU = 2;

% MPC Horizon
N = 6;

% Time Discretization
dt = 0.2;

% Constraint and Cost
constraints = [];
cost = 0;

% Weight Matrices
Q = 1;
R = eye(2);

% Set initial condition
x0 = [0.0; 0.2; 0.0; 1.0];

% Assign the initial condition for the closed loop trajectory
x_curv = x0; % curv = curvilinear

% Assessment Variables
c_opt = 0;
tot_time = 0;

% Logging and animation
x_log = x0;
u_log = [];

% Run the simulation and optimization
while (x_curv(1) <= classTrack.trackLength)
    
    tic;
    % Run optimization
    [feas, x_ftoc, u_ftoc] = solve_ftoc(Q, R, N, nX, nU, x_curv, dt, vehParams, classTrack);
    t_end = toc;
    
    x_ftoc;
    u_ftoc;
    
    if feas ~= true
        disp('Infeasibility Encountered')
        break;
    end

    c_opt = c_opt + 1;
    fprintf('Optimization Num = %d and Solver Time Elapsed = %0.4f \n', c_opt, t_end);
    
    % Extract the first optimal control input
    u_opt = u_ftoc(:,1);
    u_log = [u_log, u_opt];
    
    % Apply optimal control to system
    x_curv_next = vehicleSim(x_curv, u_opt, dt, vehParams, classTrack, 1);
    x_log = [x_log, x_curv_next];
       
    % Update the time
    tot_time = tot_time + dt;
    
    fprintf('Track Progress = %0.4f and Simulation Time = %0.4f \n', x_curv_next(1), tot_time)
    fprintf('\n')
            
    % Update initial conditions for next iteration
    x_curv = x_curv_next;
end

if feas == true
    str1 = ['Race Completed in time = ', num2str(tot_time),' seconds'];
    disp(str1)
    str2 = ['Optimizations complete: ', num2str(c_opt)];
    disp(str2)
end

% Plot the results
statePlot(x_log, u_log, dt)