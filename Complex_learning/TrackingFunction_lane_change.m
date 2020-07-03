function [result] = TrackingFunction_lane_change(current_states)

    global  x_sol_prev lam_prev tracking_lane_change iteration T_pl N data T_MPC iter_expected plot_MPC
    
%     disp('This is the current state')
%     disp('-----------------------------')
%     disp(current_states)
%     fprintf('\n iteration: %i',iteration)

% The jump is needed because the reference has a sampling of 0.025 s and
% the mpc is called every 0.1 s
    jump = T_MPC/T_pl;
    current_ref = zeros(12,N);
    current_ref(1,1:N) = data.x(int64(jump*(iteration-1)+2):int64(N+(iteration-1)*jump+1));
    current_ref(2,1:N) = data.y(int64(jump*(iteration-1)+2):int64(N+(iteration-1)*jump+1));
    current_ref(3,1:N) = data.vx(int64(jump*(iteration-1)+2):int64(N+(iteration-1)*jump+1));
    current_ref(4,1:N) = data.vy(int64(jump*(iteration-1)+2):int64(N+(iteration-1)*jump+1));
    current_ref(5,1:N) = data.psi(int64(jump*(iteration-1)+2):int64(N+(iteration-1)*jump+1));
    current_ref(6,1:N) = data.psi_dot(int64(jump*(iteration-1)+2):int64(N+(iteration-1)*jump+1));
    current_ref(7,1:N) = data.throttle(int64(jump*(iteration-1)+2):int64(N+(iteration-1)*jump+1));
    current_ref(8,1:N) = data.delta(int64(jump*(iteration-1)+2):int64(N+(iteration-1)*jump+1));
    current_ref(9,1:N) = data.throttle_dot(int64(jump*(iteration-1)+1):int64(N+(iteration-1)*jump));
    current_ref(10,1:N) = data.delta_dot(int64(jump*(iteration-1)+1):int64(N+(iteration-1)*jump));
    current_ref(11,1:N) = data.ax(int64(jump*(iteration-1)+2):int64(N+(iteration-1)*jump+1));
    current_ref(12,1:N) = data.ay(int64(jump*(iteration-1)+2):int64(N+(iteration-1)*jump+1));
    
% Solving the optimization problem by calling the CasADi function
%     fprintf('\n time spend in CasADi optimization: ')
%     tic
    [X,U,x_sol_prev,lam_prev] = tracking_lane_change(current_states,current_ref,x_sol_prev,lam_prev);
%     toc
    X = full(X);
    U = full(U);
    result = U(:,1);
%     fprintf('\n')
   
    
    if iteration > iter_expected
        fprintf('\n Unexpected amount of iterations!')
        fprintf('\n Expected: %i and got %i \n',iter_expected, iteration)
    end
        
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figures to visualize opti
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if plot_MPC == 1
        
        x_sol =X(1,:);
        y_sol = X(2,:);
        vx_sol = X(3,:);
        vy_sol = X(4,:);
        psi_sol = X(5,:); 
        psi_dot_sol = X(6,:);
        throttle_sol = X(7,:);
        delta_sol = X(8,:);
%         ax_sol = X(9,:);
%         ay_sol = X(10,:);
        
        time_sol = data.time(int64(jump*(iteration-1)+1):int64(N+(iteration-1)*jump+1));
%         time_sol = (iteration-1)*T_pl:T_pl:(iteration-1)*T_pl+N*T_pl;
        throttle_dot_sol = U(1,:);
        delta_dot_sol = U(2,:);

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
        plot(time_sol,vx_sol,'LineWidth',1)
        xlabel('t [s]','fontsize',12)
        ylabel('vx [m/s]','fontsize',12)
        legend(int2str(iteration))
        grid on

        hold on

        subplot(2,3,4)
        plot(time_sol,vy_sol,'LineWidth',1)
        xlabel('t [s]','fontsize',12)
        ylabel('vy [m/s]','fontsize',12)
        legend(int2str(iteration))
        grid on

        hold on

        subplot(2,3,5)
        plot(time_sol,psi_sol,'LineWidth',1)
        xlabel('t [s]','fontsize',12)
        ylabel('yaw [°]','fontsize',12)
        legend(int2str(iteration))
        grid on

        hold on

        subplot(2,3,6)
        plot(time_sol,psi_dot_sol,'LineWidth',1)
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
        
        figure(3)
        subplot(1,2,1)
        plot(time_sol,delta_sol.*180/pi,'LineWidth',1)
        xlabel('time [s]','fontsize',12)
        ylabel('delta [°]','fontsize',12)
        legend(int2str(iteration))
        grid on

        hold on

        subplot(1,2,2)
        plot(time_sol,throttle_sol,'LineWidth',1)
        xlabel('time [s]','fontsize',12)
        ylabel('throttle [-]','fontsize',12)
        legend(int2str(iteration))
        grid on
        
        hold on
%         figure(4)
%         subplot(1,2,1)
%         plot(time_sol,ax_sol,'LineWidth',1)
%         xlabel('time [s]','fontsize',12)
%         ylabel('ax [m/s²]','fontsize',12)
%         legend(int2str(iteration))
%         grid on
% 
%         hold on
% 
%         subplot(1,2,2)
%         plot(time_sol,ay_sol,'LineWidth',1)
%         xlabel('time [s]','fontsize',12)
%         ylabel('ay [m/s²]','fontsize',12)
%         legend(int2str(iteration))
%         grid on
    
        figure(5)
        subplot(1,2,1)
        plot(time_sol(1:end-1),delta_dot_sol.*180/pi,'LineWidth',1)
        xlabel('time [s]','fontsize',12)
        ylabel('delta_dot [°/s]','fontsize',12)
        legend(int2str(iteration))
        grid on

        hold on

        subplot(1,2,2)
        plot(time_sol(1:end-1),throttle_dot_sol,'LineWidth',1)
        xlabel('time [s]','fontsize',12)
        ylabel('throttle_dot [1/s]','fontsize',12)
        legend(int2str(iteration))
        grid on    
        
        hold on
        
    end


% Do not comment this
iteration = iteration + 1;
        
end