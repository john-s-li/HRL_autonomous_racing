%% vehicleSim.m
% Simulation for vehicle

function [x_curv_next] = vehicleSim(x, u, dt, vehParams,track)
    
    % Simulate the vehicle state evolution over time
    
    % Discretization Params
    deltaT = 0.001;
    % x_glob_next = zeros(size(x_glob,1));
    x_curv_next = zeros(size(x));
    
    % Extract state and input values
    accel = u(1); delta = u(2);
    vx = x(1); vy = x(2); wz = x(3); s = x(4); e_lat = x(5); e_psi = x(6);
    
    i = 0; % Initialize the counter
    while (i*(deltaT) <= dt)
        dx = vehicleDynamics(0,x,u,vehParams,track);
        
        dvx = dx(1);
        dvy = dx(2);
        dwz = dx(3);
        ds = dx(4);
        de_lat = dx(5);
        de_psi = dx(6);
        
        vx = vx + deltaT*dvx;
        vy = vy + deltaT*dvy;
        wz = wz + deltaT*dwz;
        s = s + deltaT*ds;
        e_lat = e_lat + deltaT*de_lat;
        e_psi = e_psi + deltaT*de_psi;

        i = i + 1;
    end
    
    % Some noise variables for robustness (implement xyz vector noise later)
    upper = 0.01;
    lower = 0.01;
    nx = lower + rand()*(upper-lower);
    ny = lower + rand()*(upper-lower);
    nz = lower + rand()*(upper-lower);
    
    x_curv_next(1) = vx + nx;
    x_curv_next(2) = vy + ny;
    x_curv_next(3) = wz + nz;
    x_curv_next(4) = s;
    x_curv_next(5) = e_lat;
    x_curv_next(6) = e_psi;   
end