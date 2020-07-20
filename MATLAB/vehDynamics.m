%% vehDynamics.m

function dx = vehDynamics(t,x,u,vehParams,track,optimize_flag)

    lf = vehParams.lf;
    lr = vehParams.lr;

    s = x(1);
    e_lat = x(2);
    e_psi = x(3);
    v = x(4);
    
    accel = u(1);
    delta = u(2);

    beta = atan((lr/(lf+lr))*tan(delta));
    
    if optimize_flag
        k = 1/-4;
    else
        k = get_curvature(s, track);
    end
   
    ds = v*(cos(e_psi + beta))/(1 - e_lat*k);
    de_lat = v*sin(e_psi + beta);
    de_psi = (v/lf)*sin(beta) - k*ds;
    dv = accel;
    
    dx = [ds;
          de_lat;
          de_psi;
          dv];
end
