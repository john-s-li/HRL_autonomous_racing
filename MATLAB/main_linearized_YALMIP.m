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
N = 7;
 
% Time Discretization
dt = 0.1;

% Constraint and Cost
constraints = [];
cost = 0;

% Weight Matrices
Q = diag([100]);
R = diag([1,10]); % Need a way to smooth the steering angle...check dynamics?

% Set initial condition 
x0 = [0.0; 0.0; 0.0; 1.0];
u0 = [0.0; 0.0];

% Set the initial optimal trajectory and control sequence (from experiments)
% Horizon should still remain on the straight segment at the beginning
% Run main_simple_YALMIP with R = 0 set in vehicleDynamics.m and unsuppress
% output to see what the values approximately be

s_predict = 0:dt:dt*N;
x_opt = [s_predict; x0(2)*ones(1,N+1); x0(3)*ones(1,N+1); x0(4)*ones(1,N+1)];
     
u_opt = zeros(2,N);

% Assign the initial condition for the closed loop trajectory
x_curv = x0; % curv = curvilinear
u_curv = u0;

% Assessment Variables
c_opt = 0;
tot_time = 0;

% Logging and animation
x_log = x0;
u_log = [];
x_pred_log = [x0];
x_traj_log = containers.Map;
x_traj_log('1') = x_opt;
u_traj_log = containers.Map;
u_traj_log('1') = u_opt;

% Run the simulation and optimization
while (x_curv(1) <= classTrack.trackLength)
     
    tic;
    % Run optimization with linearized dynamics
    [feas, x_opt, u_opt] = solve_linearized_ftoc(Q, R, N, nX, nU, x_opt, u_opt, ...
                                                 x_curv, u_curv, dt, vehParams, classTrack);
    t_end = toc;
        
    x_opt;
    u_opt;
                
    if feas ~= true
        disp('Infeasibility Encountered')
        break;
    end
       
    c_opt = c_opt + 1;
    fprintf('Optimization Num = %d and Solver Time Elapsed = %0.4f \n', c_opt, t_end);
        
    % Extract the first optimal control input
    u_curv = u_opt(:,1);
    
    % Apply optimal control to non-linear system
    x_curv_next = vehicleSim(x_curv, u_curv, dt, vehParams, classTrack);
            
    % Log plotting variables
    x_log = [x_log, x_curv_next];
    x_pred_log = [x_pred_log x_opt(:,2)];
    x_traj_log(num2str(c_opt+1)) = x_opt;
    u_traj_log(num2str(c_opt+1)) = u_opt;
    u_log = [u_log, u_curv];
    
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

% Animation
plotLog(x_log, x_traj_log, vehParams, classTrack, 0.2)

% Plot the results
statePlot(x_log, x_pred_log, u_log, dt)
