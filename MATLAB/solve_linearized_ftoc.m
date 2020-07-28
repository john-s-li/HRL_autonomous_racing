%% solve_linearized_ftoc.m
% Optimization script using YALMIP + IPOPT

function [feas, x_ftoc, u_ftoc] = solve_linearized_ftoc(Q, R, N, nX, nU, x_opt, u_opt, x_curv, u_curv, dt, vehParams, track)

    lf = vehParams.lf; 
    lr = vehParams.lr;
    Bf = vehParams.Bf;
    Br = vehParams.Br;
    Cf = vehParams.Cf;
    Cr = vehParams.Cr;
    Df = vehParams.Df;
    Dr = vehParams.Dr;
    Iz = vehParams.Iz;
    m = vehParams.m;
    g = vehParams.g;

    % variable
    x = sdpvar(nX, N+1); % [vx; vy; wz; s; e_lat; e_psi]
    u = sdpvar(nU, N);   % [accel; delta]
      
    % extend u_opt (need a more intelligent way to do this later)
    u_opt = [u_opt(:,2:end) u_opt(:,end)];
    
    % constraint & cost
    constraints = [];
    cost = 0;
    
    x_ref = [1.0; 0.0; 0.0; 0.0; 0.0; 0.0]; 
    
    % k_max = 0.5/(vehParams.lf + vehParams.lr);
    % k_min = -0.5/(vehParams.lf + vehParams.lr);
    
    for i = 1:N
        % dynamics constraint
        vx = x(1,i);
        vy = x(2,i);
        wz = x(3,i);
        s = x(4,i);
        e_lat = x(5,i);
        e_psi = x(6,i);
        
        accel = u(1,i);
        delta = u(2,i);
        
        % k_car = tan(delta)/(vehParams.lf + vehParams.lr);
        
        % Use the linearization points ahead of car bc car has already
        % moved with one optimal input applied
        k = get_curvature(x_opt(4,i+1), track);
        
        if i == 1
            dx = A_gen(x_curv, u_opt(:,i), k)*x(:,i) + ...
                 B_gen(x_curv, u_opt(:,i), k)*u(:,i);
        else
            dx = A_gen(x_opt(:,i+1),u_opt(:,i),k)*x(:,i) + ...
                 B_gen(x_opt(:,i+1),u_opt(:,i),k)*u(:,i);   
        end
       
        x_next = x(:,i) + dt*dx;
        
        % Friction Constraints
        mu = 0.7; % avg friction coefficient for roads (assume rear wheel drive)
        [F_yf, F_yr, F_nf, F_nr] = tireSplitForces(x(:,i),u(:,i),vehParams);
        
        constraints = [constraints;
            % Initial Condition
            x(:,1) == x_curv;
            % State Constraint
            -track.width <= e_lat <= track.width;
            0 <= vx <= 4.0;
            -pi/2 <= e_psi <= pi/2;
            % Input Constraint
            -1 <= accel <= 1;
            -0.5 <= delta <= 0.5;
            % Friction Constraint
            (F_yf)^2 + (accel/2)^2 <= (mu*F_nf)^2;
            (F_yr)^2 + (accel/2)^2 <= (mu*F_nr)^2;
            % Dynamics Constraint
            x(:,i+1) == x_next];
          
        cost = cost + (x(:,i)-x_ref)'*Q*(x(:,i)-x_ref) + u(:,i)'*R*u(:,i);
        
%         % Penalize large change in inputs
%         if i ~= N
%            cost = cost + (u(2,i+1) - u(2,i))*10*(u(2,i+1) - u(2,i));
%         end
        
    end
    cost = cost - (x(4,end)-x_curv(4))*10*(x(4,end)-x_curv(4));

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