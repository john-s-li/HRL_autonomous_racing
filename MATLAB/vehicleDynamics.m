%% vehicleDynamics.m

function dx = vehicleDynamics(t,x,u,vehParams,track)

    lf = vehParams.lf;
    lr = vehParams.lr;

    s = x(1);
    e_lat = x(2);
    e_psi = x(3);
    v = x(4);
    
    accel = u(1);
    delta = u(2);

    beta = atan((lr/(lf+lr))*tan(delta));
    
    k = get_curvature(s, track);
       
    ds = v*(cos(e_psi + beta))/(1 - e_lat*k);
    de_lat = v*sin(e_psi + beta);
    de_psi = (v/lf)*sin(beta) - k*ds;
    dv = accel;
    
    dx = [ds;
          de_lat;
          de_psi;
          dv];
end
