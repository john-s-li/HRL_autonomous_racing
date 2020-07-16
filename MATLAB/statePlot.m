%% statePlot.m
% Function to plot the state progression of vehicle on race track

function statePlot(x,u,dt)

    x_t = 0:dt:dt*(size(x,2)-1);
    u_t = 0:dt:dt*(size(u,2)-1);
    
    vx = x(1,:);
    vy = x(2,:);
    wz = x(3,:);
    s = x(4,:);
    e_psi = x(5,:);
    e_lat = x(6,:);
    
    accel = u(1,:);
    delta = u(2,:);
    
    figure();
    subplot(211)
    plot(x_t,vx,'DisplayName','$v_x$')
    hold on
    plot(x_t,vy,'DisplayName','$v_y$')
    plot(x_t,wz,'DisplayName','$\omega_z$')
    plot(x_t,s,'DisplayName','$s$')
    plot(x_t,e_lat,'DisplayName','$e_{y}$')
    plot(x_t,e_psi,'DisplayName','$e_{\psi}$')
    hold off
    xlabel('time $t$','Interpreter','latex')
    title('Dynamic Bicylce Model States with FTOC','Interpreter','latex')
    legend('show','Interpreter','latex','FontSize',14)
    grid on
    
    subplot(212)
    plot(u_t,accel,'DisplayName','$a$')
    hold on
    plot(u_t,delta,'DisplayName','$\delta$')
    hold off
    xlabel('time $t$','Interpreter','latex')
    title('Dynamic Bicycle Dynamics Optimal Control Sequence','Interpreter','latex')
    legend('show','Interpreter','latex','FontSize',14)
    grid on
    
end