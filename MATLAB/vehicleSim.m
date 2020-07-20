%% vehicleSim.m
% Simulation for vehicle

function [x_curv_next] = vehicleSim(x, u, dt, vehParams, track)
    
    % Simulate the vehicle state evolution over time
    s = x(1);
    e_lat = x(2);
    e_psi = x(3);
    v = x(4);

    % Discretization Params
    deltaT = 0.001;
    
    x_curv_next = zeros(size(x,1),1);
    
    i = 0; % Initialize the counter
    while (i*(deltaT) <= dt)
        
        dx = vehDynamics(0,x,u,vehParams,track);
    
        ds = dx(1);
        de_lat = dx(2);
        de_psi = dx(3);
        dv = dx(4);

        s = s + deltaT*ds;
        e_lat = e_lat + deltaT*de_lat;
        e_psi = e_psi + deltaT*de_psi;
        v = v + deltaT*dv;

        i = i + 1;
    end
        
    % Some noise variables for robustness (implement xyz vector noise later)
    upper = 0.05;
    lower = 0.05;
    nv = lower + rand()*(upper-lower);
    
    v = v + nv;
    
    x_curv_next(1) = s;
    x_curv_next(2) = e_lat;
    x_curv_next(3) = e_psi;
    x_curv_next(4) = v;
    
end