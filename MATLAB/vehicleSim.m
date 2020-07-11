%% vehicleSim.m
% Simulation for vehicle

function [x_curv_next, x_glob_next] = vehicleSim(x, x_glob, u, dt, track)
    % Simulate the vehicle state evolution over time
    
    % Discretization Params
    deltaT = 0.001;
    x_glob_next = zeros(size(x_glob,1));
    x_curv_next = zeros(size(x,1));

    % Extract state and input values
    delta = u(1); accel = u(2);
    psi = x_glob(4); X = x_glob(5); Y = x_glob(6);
    vx = x(1); vy = x(2); wz = x(3); e_psi = x(4); s = x(5); e_lat = x(6);
    
    i = 0; % Initialize the counter
    while (i*(deltaT) <= dt)
        
        % Tire Split Angle
        alpha_f = delta - atan2( vy + lf * wz, vx );
        alpha_r = -atan2( vy - lf * wz , vx);

        F_nf = lf/(lf+lr)*m*g; % Front Tire Normal Load
        F_nr = lr/(lf+lr)*m*g; % Rear Tire Normal Load

        F_yf = F_nf*Df*sin( Cf * atan2(1, Bf*alpha_f)); % Front Tire Lateral Force
        F_yr = F_nr*Dr*sin( Cr * atan2(1, Br*alpha_r)); % Rear Tire Lateral Force

        dvx = 1/m.*(accel - F_yf.*sin(delta) + m.*(wz.*vy));
        dvy = 1/m.*(F_yr + F_yf.*sin(delta) - m.*vx.*wz);
        dwz = 1/Iz.*(F_fy.*lf.*cos(delta) - F_yr.*lr); % No torque vectoring 
        de_psi = wz - (vx.*cos(e_psi) - vy.*sin(e_psi))/(1-get_curvature(s,track)*e_lat)*get_curvature(s,track);
        ds = (vx.*cos(e_psi)-vy.*sin(e_psi))./(1 - get_curvature(s,track)*e_lat);
        de_lat = vx.*sin(e_psi) + vy.*cos(e_psi);

        % Update the states for next iteration
        X = X + deltaT*(vx.*cos(psi) - vy.*sin(psi));
        Y = Y + deltaT*d(vy.*cos(psi) + vx.*sin(psi));
        psi = psi + deltaT*(wz);
        
        vx = vx + deltaT*dvx;
        vy = vy + deltaT*dvy;
        wz = wz + deltaT*dwz;
        e_psi = e_psi + deltaT*de_psi;
        s = s + deltaT*ds;
        e_lat = e_lat + deltaT*de_lat;

        i = i + 1;
    end
    
    % Populate the state vectors to return
    x_glob_next(1) = vx;
    x_glob_next(2) = vy;
    x_glob_next(3) = wz;
    x_glob_next(4) = psi;
    x_glob_next(5) = X;
    x_glob_next(6) = Y;
    
    % Some noise variables for robustness (implement xyz vector noise later)
    upper = 0.05;
    lower = 0.05;
    nx = lower + rand()*(upper-lower);
    ny = lower + rand()*(upper-lower);
    nz = lower + rand()*(upper-lower);
    
    x_curv_next(1) = vx + nx;
    x_curv_next(2) = vy + ny;
    x_curv_next(3) = wz + nz;
    x_curv_next(4) = e_psi;
    x_curv_next(5) = s;
    x_curv_next(6) = e_lat;
    
end