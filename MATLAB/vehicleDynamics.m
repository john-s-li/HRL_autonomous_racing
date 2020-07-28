%% vehicleDynamics.m

function dx = vehicleDynamics(t,x,u,vehParams,track)

    % Param t = time only needed if doing ODE simulation
    % Otherwise, input t = 0

    lf = vehParams.lf; 
    lr = vehParams.lr;
    Iz = vehParams.Iz;
    m = vehParams.m;

    vx = x(1);
    vy = x(2);
    wz = x(3);
    s = x(4);
    e_lat = x(5);
    e_psi = x(6);
    
    accel = u(1);
    delta = u(2);

    k = get_curvature(s, track);

    [F_yf, F_yr, ~ , ~ ] = tireSplitForces(x,u,vehParams);

    dvx = 1/m*(m*accel - F_yf*sin(delta) + m*(wz*vy));
    dvy = 1/m*(F_yr + F_yf*cos(delta) - m*vx*wz);
    dwz = 1/Iz*(F_yf*lf*cos(delta) - F_yr*lr); 
    ds = (vx*cos(e_psi)-vy*sin(e_psi))/(1 - k*e_lat);
    de_lat = vx*sin(e_psi) + vy*cos(e_psi);
    de_psi = wz - k*ds;
    
    dx = [dvx;
          dvy;
          dwz;
          ds;
          de_lat;
          de_psi];
end