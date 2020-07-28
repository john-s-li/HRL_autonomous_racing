%% statePlot.m
% Function to plot the state progression of vehicle on race track

function statePlot(x,x_pred,u,dt)

    if isscalar(dt)
        % For optimization plotting (i.e. MPC)
        x_t = 0:dt:dt*(size(x,2)-1);
        u_t = 0:dt:dt*(size(u,2)-1);
    else
        % For PID or other non-optimization plotting purposes
        x_t = dt;
        u_t = dt(:,1:end-1);
    end
    
    c1 = [0, 0.4470, 0.7410];
    c2 = [0.8500, 0.3250, 0.0980];
    c3 = [0.4660, 0.6740, 0.1880];
    c4 = [0.3010, 0.7450, 0.9330];
    
    s = x(1,:);
    e_lat = x(2,:);
    e_psi = x(3,:);
    v = x(4,:);
     
    accel = u(1,:);
    delta = u(2,:);
    
    figure()
    
    % Plot states
    % -------------------------------------------------------------------------------
    subplot(311)
    plot(x_t,s,'color',c4,'DisplayName','$s$','LineWidth',2)
    hold on
    if ~isempty(x_pred)
        s_p = x_pred(1,:);
        plot(x_t,s_p,'--','color',c4,'DisplayName','$s_p$','LineWidth',2)
    end
    hold off
    xlabel('time $t$','Interpreter','latex')
    title('Dynamic Bicycle Curviilinear Abscissa','Interpreter','latex')
    legend('show','Interpreter','latex','FontSize',14)
    xlim([0 x_t(end)])
    grid on
    
    subplot(312)
    plot(x_t,e_lat,'color',c2,'DisplayName','$e_{y}$','LineWidth',2)  
    hold on
    plot(x_t,e_psi,'color',c3,'DisplayName','$e_{\psi}$','LineWidth',2)
    plot(x_t,v,'color',c4,'DisplayName','$v$','LineWidth',2)
   
    if ~isempty(x_pred)
        e_lat_p = x_pred(2,:);
        e_psi_p = x_pred(3,:);
        v_p = x(4,:);
        
        plot(x_t,e_lat_p,'--','color',c2,'DisplayName','$e_{y,p}$','LineWidth',2)
        plot(x_t,e_psi_p,'--','color',c3,'DisplayName','$e_{\psi,p}$','LineWidth',2)
        plot(x_t,v_p,'--','color',c4,'DisplayName','$v_p$','LineWidth',2)
    end
    
    hold off
    
    % Add vertical dashed lines 
%     for i = 1:length(x_t)
%         xline(x_t(i),'-.', 'HandleVisibility','off');
%     end
    
    xlabel('time $t$','Interpreter','latex')
    title('Dynamic Bicycle Dynamics','Interpreter','latex')
    legend('show','Interpreter','latex','FontSize',14)
    xlim([0 x_t(end)])
    grid on
    
    % Plot controls
    % ---------------------------------------------------------------------------------
    subplot(313)
    % Plot this in discretized form    
    stairs(u_t,accel,'DisplayName','$a$','LineWidth',2)
    hold on
    stairs(u_t,delta,'DisplayName','$\delta$','LineWidth',2)
    
    % Add vertical dashed lines 
%     for i = 1:length(x_t)
%         xline(x_t(i),'-.','HandleVisibility','off');
%     end
%     
    hold off
    xlabel('time $t$','Interpreter','latex')
    title('Dynamic Bicycle Optimal Control Sequence','Interpreter','latex')
    legend('show','Interpreter','latex','FontSize',14)
    grid on
    xlim([0 x_t(end)])
end