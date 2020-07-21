%% solve_linearized_ftoc.m
% Optimization script using YALMIP + IPOPT
function [feas, x_ftoc, u_ftoc] = solve_linearized_ftoc(Q, R, N, nX, nU, x_opt, u_opt, x_curv, u_curv, dt, vehParams, track)

    % variable
    x = sdpvar(nX, N+1); % [s; e_lat; e_psi; v]
    u = sdpvar(nU, N);   % [accel; delta]
    % s_psi = sdpvar(1,N+1); % Slack variable
    
    x0 = x_curv;
    u0 = u_curv;
    
    % constraint & cost
    constraints = [];
    cost = 0;
    
    % extend u_opt
    u_opt = [u_opt u_opt(:,end)];
    
    for i = 1:N
        % dynamics constraint
        s = x(1,:);
        e_lat = x(2,i);
        e_psi = x(3,i);
        v = x(4,i);
        
        accel = u(1,i);
        delta = u(2,i);
        
        % Use the linearization points ahead of car bc car has already
        % moved with one optimal input applied
        k = get_curvature(x_opt(1,i+1), track);
        
        if i == 1
             dx = A_gen(x0, u0, k)*x(:,i) + ...
                  B_gen(x0, u0, k)*u(:,i);
        else
            dx = A_gen(x_opt(:,i+1),u_opt(:,i),k)*x(:,i) + ...
                 B_gen(x_opt(:,i+1),u_opt(:,i),k)*u(:,i);   
        end
       
        x_next = x(:,i) + dt*dx;
        
        constraints = [constraints;
            % Initial Condition
            x(:,1) == x0;
            % State Constraint
            % -track.width <= x(2,i) + s_psi(i) <= track.width;
            -track.width <= e_lat <= track.width;
            -4 <= v <= 4;
            % Input Constraint
            -1 <= accel <= 1;
            -0.5 <= delta <= 0.5;
            % Dynamics Constraint
            x(:,i+1) == x_next];

        cost = cost - s(i)'*Q*s(i) + u(:,i)'*R*u(:,i); % May need to linearize this also
    end

    options = sdpsettings('verbose',0,'debug',1,'solver','ipopt');
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