%% tireSplitForces.m
% Compute the tire slip angles

function [F_yf, F_yr, F_nf, F_nr] = tireSplitForces(x,u,vehParams)

    g =  vehParams.g;
    m  = vehParams.m;
    lf = vehParams.lf;
    lr = vehParams.lr;
    Df = vehParams.Df;
    Cf = vehParams.Cf;
    Bf = vehParams.Bf;
    Dr = vehParams.Dr;
    Cr = vehParams.Cr;
    Br = vehParams.Br;
    
    % Extract state and input values
    accel = u(1); delta = u(2);
    vx = x(1); vy = x(2); wz = x(3); s = x(4); e_lat = x(5); e_psi = x(6);
    
    % Tire Split Angles (Negative due to our coordinate frame)
    frame = -1;
    alpha_f = frame * (atan( (vy + lf * wz) / abs(vx) ) - delta);
    alpha_r = frame * atan( (vy - lf * wz) / abs(vx) );

    F_nf = lf/(lf+lr)*m*g; % Front Tire Normal Load
    F_nr = lr/(lf+lr)*m*g; % Rear Tire Normal Load

    F_yf = F_nf*Df*sin( Cf * atan(Bf*alpha_f) ); % Front Tire Lateral Force
    F_yr = F_nr*Dr*sin( Cr * atan(Br*alpha_r) ); % Rear Tire Lateral Force
end