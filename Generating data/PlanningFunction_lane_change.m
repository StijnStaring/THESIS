function [result] = PlanningFunction_lane_change(current_states)

    global  width_road x_sol_prev lam_prev theta planner_lane_change iteration vx_desired
    disp('This is the current state')
    current_states
    
%   current_states(3) = 80/3.6;
    % Solving the optimization problem by calling the CasADi function,
    % previously implemented and loaded as a global variable  
    
    [X,U,T,x_sol_prev,lam_prev] = planner_lane_change(current_states,theta,width_road,vx_desired,x_sol_prev,lam_prev);
    X = full(X);
    U = full(U);
    T_sol = full(T);
    gain = 5;
    result = U(:,1)*gain;
%     result = [0.01,0.6*pi/180]';
    
    disp('simulation completed!')
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figures to visualize opti
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    x_sol =X(1,:);
    y_sol = X(2,:);
    Vx_sol = X(3,:);
    Vy_sol = X(4,:);
    yaw_sol = X(5,:); 
    r_sol = X(6,:);
    time_sol = linspace(0,T_sol,max(size(x_sol)));
    throttle = U(1,:);
    delta = U(2,:)*180/pi;
   
    % States
    figure(1)
    subplot(2,3,1)
    plot(time_sol,x_sol,'LineWidth',1)
    xlabel('t [s]','fontsize',12)
    ylabel('x [m]','fontsize',12)
    legend(int2str(iteration))
    grid on
    
    hold on

    subplot(2,3,2)
    plot(time_sol,y_sol,'LineWidth',1)
    xlabel('t [s]','fontsize',12)
    ylabel('y [m]','fontsize',12)
    legend(int2str(iteration))
    grid on
    
    hold on

    subplot(2,3,3)
    plot(time_sol,Vx_sol,'LineWidth',1)
    xlabel('t [s]','fontsize',12)
    ylabel('vx [m/s]','fontsize',12)
    legend(int2str(iteration))
    grid on
    
    hold on

    subplot(2,3,4)
    plot(time_sol,Vy_sol,'LineWidth',1)
    xlabel('t [s]','fontsize',12)
    ylabel('vy [m/s]','fontsize',12)
    legend(int2str(iteration))
    grid on
    
    hold on

    subplot(2,3,5)
    plot(time_sol,yaw_sol,'LineWidth',1)
    xlabel('t [s]','fontsize',12)
    ylabel('yaw [°]','fontsize',12)
    legend(int2str(iteration))
    grid on
    
    hold on

    subplot(2,3,6)
    plot(time_sol,r_sol,'LineWidth',1)
    xlabel('t [s]','fontsize',12)
    ylabel('r [°/s]','fontsize',12)
    legend(int2str(iteration))
    grid on
    
    hold on
    
    % Path
    figure(2)
    plot(x_sol,y_sol,'LineWidth',1)
    xlabel('x [m]','fontsize',12)
    ylabel('y [m]','fontsize',12)
    legend(int2str(iteration))
    grid on
    
    hold on
    
    %Controls
    figure(3)
    subplot(1,2,1)
    plot(time_sol(1:end-1),delta,'LineWidth',1)
    xlabel('time [s]','fontsize',12)
    ylabel('delta [°]','fontsize',12)
    legend(int2str(iteration))
    grid on
    
    hold on
    
    subplot(1,2,2)
    plot(time_sol(1:end-1),throttle,'LineWidth',1)
    xlabel('time [s]','fontsize',12)
    ylabel('throttle [-]','fontsize',12)
    legend(int2str(iteration))
    grid on
    
    hold on
    
    iteration = iteration + 1;
    
    
end