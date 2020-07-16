function [feas, x_ftoc, u_ftoc] = solve_ftoc(Q, R, N, nX, nU, x0, dt)

% parameters
vehParams = vehicleParams();
track = Track(0.8); % initialize the track with width

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

% variable
x = sdpvar(nX, N+1); % [s; e_lat; e_psi; v]
u = sdpvar(nU, N);   % [accel; delta]

% constraint & cost
constraints = [];
cost = 0;

for i = 1:N
    % dynamics constraint
    s = x(1,i);
    e_lat = x(2,i);
    e_psi = x(3,i);
    v = x(4,i);
    accel = u(1,i);
    delta = u(2,i);

    beta = atan((lr/(lf+lr))*tan(delta));
    k = 1/-4; % curvature for a circle with constant radius = 4 (Counter-CW)
    ds = v*(cos(e_psi + beta))/(1 - e_lat*k);
    de_lat = v*sin(e_psi + beta);
    de_psi = (v/lf)*sin(beta) - k*ds;
    dv = accel;

    s_next = s + ds*dt;
    e_lat_next = e_lat + de_lat*dt;
    e_psi_next = e_psi + de_psi*dt;
    v_next = v + dv*dt;
    
    constraints = [constraints;
        % initial condition
        x(:,1) == x0;
        % state constraint
        -track.width <= x(2,i) <= track.width;
        % input constraint
        -1 <= u(1,i) <= 1;
        -0.5 <= u(2,i) <= 0.5;
        % dynamics constraint
        x(:,i+1) == [s_next; e_lat_next; e_psi_next; v_next]];
    
    cost = cost + x(1,i)'*Q*x(1,i) + u(:,i)'*R*u(:,i);
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