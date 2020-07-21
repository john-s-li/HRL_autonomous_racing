%% solve_ftoc.m
% Optimization script using YALMIP + IPOPT
function [feas, x_ftoc, u_ftoc] = solve_ftoc(Q, R, N, nX, nU, x0, dt, vehParams, track)

    % variable
    x = sdpvar(nX, N+1); % [s; e_lat; e_psi; v]
    u = sdpvar(nU, N);   % [accel; delta]
    % s_psi = sdpvar(1,N+1); % Slack variable

    % constraint & cost
    constraints = [];
    cost = 0;

    for i = 1:N
        % dynamics constraint
        s = x(1,:);
        e_lat = x(2,i);
        e_psi = x(3,i);
        v = x(4,i);
        
        accel = u(1,i);
        delta = u(2,i);
        
        dx = vehicleDynamics(0,x(:,i),u(:,i),vehParams,track); 
        
        x_next = x(:,i) + dt*dx;
        
        constraints = [constraints;
            % Initial Condition
            x(:,1) == x0;
            % State Constraint
            % -track.width <= x(2,i) + s_psi(i) <= track.width;
            -track.width/2 <= e_lat <= track.width/2;
            -4 <= v <= 4;
            % Input Constraint
            -1 <= accel <= 1;
            -0.5 <= delta <= 0.5;
            % Dynamics Constraint
            x(:,i+1) == x_next];

        cost = cost - x(1,i)'*Q*x(1,i) + u(:,i)'*R*u(:,i); % + s_psi(i)'*S*s_psi(i);
    end

    options = sdpsettings('verbose',0,'debug',1,'solver','gurobi');
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