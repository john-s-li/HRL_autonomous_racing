%% statePlot.m
% Function to plot the state progression of vehicle on race track

function statePlot(x,u,dt)

    x_t = 0:dt:dt*(size(x,2)-1);
    u_t = 0:dt:dt*(size(u,2)-1);
    
    s = x(1,:);
    e_lat = x(2,:);
    e_psi = x(3,:);
    v = x(4,:);

    accel = u(1,:);
    delta = u(2,:);
    
    subplot(211)
    plot(x_t,s,'DisplayName','$s$','LineWidth',2)
    hold on
    plot(x_t,e_lat,'DisplayName','$e_{y}$','LineWidth',2)
    plot(x_t,e_psi,'DisplayName','$e_{\psi}$','LineWidth',2)
    plot(x_t,v,'DisplayName','$v$','LineWidth',2)
    hold off
    xlabel('time $t$','Interpreter','latex')
    title('Simplified Bicycle Dynamics','Interpreter','latex')
    legend('show','Interpreter','latex','FontSize',14)
    grid on
    
    subplot(212)
    plot(u_t,accel,'DisplayName','$a$','LineWidth',2)
    hold on
    plot(u_t,delta,'DisplayName','$\delta$','LineWidth',2)
    hold off
    xlabel('time $t$','Interpreter','latex')
    title('Dynamic Bicycle Optimal Control Sequence','Interpreter','latex')
    legend('show','Interpreter','latex','FontSize',14)
    grid on
    
end