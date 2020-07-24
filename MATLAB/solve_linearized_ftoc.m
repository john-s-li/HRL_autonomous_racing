%% solve_linearized_ftoc.m
% Optimization script using YALMIP + IPOPT
function [feas, x_ftoc, u_ftoc] = solve_linearized_ftoc(Q, R, N, nX, nU, x_opt, u_opt, x_curv, u_curv, dt, vehParams, track)

    % variable
    x = sdpvar(nX, N+1); % [s; e_lat; e_psi; v]
    u = sdpvar(nU, N);   % [accel; delta]
    % s_psi = sdpvar(1,N+1); % Slack variable
      
    % extend u_opt
    u_opt = [u_opt(:,2:end) u_opt(:,end)];
    
    % constraint & cost
    constraints = [];
    cost = 0;
    
    k_max = 0.5/(vehParams.lf + vehParams.lr);
    k_min = -0.5/(vehParams.lf + vehParams.lr);
    
    for i = 1:N
        % dynamics constraint
        s = x(1,:);
        e_lat = x(2,i);
        e_psi = x(3,i);
        v = x(4,i);
        
        accel = u(1,i);
        delta = u(2,i);
        
        k_car = tan(delta)/(vehParams.lf + vehParams.lr);
        
        % Use the linearization points ahead of car bc car has already
        % moved with one optimal input applied
        k = get_curvature(x_opt(1,i+1), track);
        
        if i == 1
            dx = A_gen(x_curv, u_opt(:,i), k)*x(:,i) + ...
                 B_gen(x_curv, u_opt(:,i), k)*u(:,i);
        else
            dx = A_gen(x_opt(:,i+1),u_opt(:,i),k)*x(:,i) + ...
                 B_gen(x_opt(:,i+1),u_opt(:,i),k)*u(:,i);   
        end
       
        x_next = x(:,i) + dt*dx;
        
        constraints = [constraints;
            % Initial Condition
            x(:,1) == x_curv;
            % State Constraint
            -track.width <= e_lat <= track.width;
            0 <= v <= 4.0;
            -pi/2 <= e_psi <= pi/2;
            % Input Constraint
            -1 <= accel <= 1;
            -0.5 <= delta <= 0.5;
            % Dynamics Constraint
            x(:,i+1) == x_next];

        % IDEA: smooth out the controls somehow...do a low pass filter like
        % Quan!
        cost = cost + e_lat*100*e_lat + u(:,i)'*R*u(:,i);
    end
    cost = cost - (s(end)-x_curv(1))*1.0;

    options = sdpsettings('verbose',0,'debug',1,'solver','ipopt');
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