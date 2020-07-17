%% solve_ftoc.m
% Optimization script using YALMIP + IPOPT
function [feas, x_ftoc, u_ftoc] = solve_ftoc(Q, R, N, nX, nU, x0, dt, vehParams, track)

lf = vehParams.lf;
lr = vehParams.lr;

% variable
x = sdpvar(nX, N+1); % [s; e_lat; e_psi; v]
u = sdpvar(nU, N);   % [accel; delta]
% s_psi = sdpvar(1,N+1); % Slack variable

% S = 10;

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
        % Initial Condition
        x(:,1) == x0;
        % State Constraint
        % -track.width <= x(2,i) + s_psi(i) <= track.width;
        -track.width <= e_psi <= track.width;
        % Input Constraint
        -1 <= accel <= 1;
        -0.5 <= delta <= 0.5;
        % Miscellaneous Constraints
        ds >= 0;
        s >= 0;
        % Dynamics Constraint
        x(:,i+1) == [s_next; e_lat_next; e_psi_next; v_next]];
    
    cost = cost + x(1,i)'*Q*x(1,i) + u(:,i)'*R*u(:,i); % + s_psi(i)'*S*s_psi(i);
end

options = sdpsettings('verbose',0,'solver','ipopt');
diagnostics = optimize(constraints, cost, options);

% assign solutions
if diagnostics.problem == 0
    feas = true;
    x_ftoc = value(x);
    u_ftoc = value(u);
    % slack = value(s_psi);
    JOpt = value(cost);  
else
    feas = false;
    x_ftoc = [];
    u_ftoc = [];
    JOpt = [];
end

end