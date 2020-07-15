%% vehicleSim.m
% Simulation for vehicle

function [x_curv_next] = vehicleSim(x, u, dt, vehParams, track)

    lf = vehParams.lf;
    lr = vehParams.lr;
    
    % Simulate the vehicle state evolution over time
    
    % Discretization Params
    deltaT = 0.001;
    
    x_curv_next = zeros(size(x,1),1);

    % Extract state and input values
    delta = u(2); accel = u(1);
    
    s = x(1);
    e_lat = x(2);
    e_psi = x(3);
    v = x(4);
    
    i = 0; % Initialize the counter
    while (i*(deltaT) <= dt)
        beta = atan((lr/(lf+lr))*tan(delta));

        ds = v*(cos(e_psi + beta))/(1 - e_lat*get_curvature(s,track));
        de_lat = v*sin(e_psi + beta);
        de_psi = (v/lf)*sin(beta) - get_curvature(s,track)*ds;
        dv = accel;

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