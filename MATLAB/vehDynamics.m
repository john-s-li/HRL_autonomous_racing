%% vehDynamics.m

function dx = vehDynamics(t,x,u,vehParams,track)

    lf = vehParams.lf;
    lr = vehParams.lr;

    s = x(1);
    e_lat = x(2);
    e_psi = x(3);
    v = x(4);
    
    accel = u(1);
    delta = u(2);

    beta = atan((lr/(lf+lr))*tan(delta));
    k = get_curvature(s,track); 
    ds = v*(cos(e_psi + beta))/(1 - e_lat*k);
    de_lat = v*sin(e_psi + beta);
    de_psi = (v/lf)*sin(beta) - k*ds;
    dv = accel;
    
    dx = [ds;
          de_lat;
          de_psi;
          dv];
end

function k = get_curvature(s, track)
    
    s = mod(s, track.trackLength);
    
    % Heaviside Function (MUST be updated based on track's specification...see Track.m)
    % Will parametrize later when source is to be published
    c = -1;
    k = c*(heaviside(s-3) - heaviside(s-4.57)) + ...
        c*(heaviside(s-6.57) - heaviside(s-8.1416)) + ...
        c*(heaviside(s-14.1416) - heaviside(s-15.7124)) + ...
        c*(heaviside(s-17.7124) - heaviside(s-19.2832));
end