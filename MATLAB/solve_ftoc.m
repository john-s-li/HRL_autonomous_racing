function [feas, x_ftoc, u_ftoc] = solve_ftoc(Q, R, N, nX, nU, x0, dt, vehParams, track)

g =  vehParams.g;
m  = vehParams.m;
lf = vehParams.lf;
lr = vehParams.lr;
Iz = vehParams.Iz;
Df = vehParams.Df;
Cf = vehParams.Cf;
Bf = vehParams.Bf;
Dr = vehParams.Dr;
Cr = vehParams.Cr;
Br = vehParams.Br;

mu = 0.7; % Friction coefficient for dry road

% Optimizations Variables
x = sdpvar(nX, N+1); % [vx; vy; wz; s; e_psi; e_lat]
u = sdpvar(nU, N);   % [accel; delta]

% constraint & cost
constraints = [];
cost = 0;

for i = 1:N
    % dynamics constraint
    vx = x(1,i);
    vy = x(2,i);
    wz = x(3,i);
    s  = x(4,i);
    e_psi = x(5,i);
    e_lat = x(6,i);
    
    accel = u(1,i);
    delta = u(2,i);

    % Tire Split Angles (Negative due to our coordinate frame)
    alpha_f = delta - atan( (vy + lf * wz) / vx );
    alpha_r = - atan( (vy - lf * wz) / vx);

    F_nf = lf/(lf+lr)*m*g; % Front Tire Normal Load
    F_nr = lr/(lf+lr)*m*g; % Rear Tire Normal Load

    F_yf = F_nf*Df*sin(Cf * atan(Bf*alpha_f)); % Front Tire Lateral Force
    F_yr = F_nr*Dr*sin(Cr * atan(Br*alpha_r)); % Rear Tire Lateral Force

    % ODEs
    % NOTE: for now, keep k(s) constant (sym variables don't work with logical operators)

    dvx = 1/m*(m*accel - F_yf*sin(delta) + m*(wz*vy));
    dvy = 1/m*(F_yr + F_yf*sin(delta) - m*vx(1)*wz);
    dwz = 1/Iz*(F_yf*lf*cos(delta) - F_yr*lr); % No torque vectoring 
    ds = (vx*cos(e_psi)-vy*sin(e_psi))/(1 - get_curvature(s,track)*e_lat);
    de_psi = wz - get_curvature(s,track)*ds;
    de_lat = vx*sin(e_psi) + vy*cos(e_psi);

    % use discrete time integration
    vx_next = vx + dt*dvx; 
    vy_next = vy + dt*dvy;
    wz_next = wz + dt*dwz;
    s_next = s + dt*ds;
    e_psi_next = e_psi + dt*de_psi;
    e_lat_next = e_lat + dt*de_lat;
    
    % Define contraints
    constraints = [constraints;
        % Initial Condition
        x(:,1) == x0;
        % State Constraint
        -track.width <= e_lat <= track.width;
        -4 <= vx <= 4;
        % Friction Constraint
        F_yf^2 + (accel/2)^2 <= (mu*F_nf)^2;
        F_yr^2 + (accel/2)^2 <= (mu*F_nr)^2;
        % Input Constraint
        -1 <= accel <= 1;
        -0.5 <= delta <= 0.5;
        % Dynamics Constraint
        x(:,i+1) == [vx_next; vy_next; wz_next; s_next; e_psi_next; e_lat_next]];
    
    cost = cost + s'*Q*s + u(:,i)'*R*u(:,i);
end

options = sdpsettings('verbose',0,'solver','ipopt');
diagnostics = optimize(constraints, cost, options);

% assign solutions
if diagnostics.problem == 0
    feas = true;
    x_ftoc = value(x);
    u_ftoc = value(u);
    JOpt = value(cost);  
else
    feas = false;
    x_ftoc = [];
    u_ftoc = [];
    JOpt = [];
end

end