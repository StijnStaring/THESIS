function data = get_data(file)
    % This function is for defining the data parameters in a structure
    % 'data'. 
    % Read data
    read = csvread(file,1);
    % Struct data
    data = struct;
    time_d = read(:,1);
    x_d = read(:,2);
    y_d = read(:,3);
    vx_d =read(:,4);
    vy_d = read(:,5);
    ax_d = read(:,6); % total acc
    ay_d = read(:,7); % total acc
    jx_d = read(:,8);
    jy_d = read(:,9);
    psi_d = read(:,10);
    psi_dot_d = read(:,11);
    psi_ddot_d = read(:,12);
    throttle_d = read(:,13);
    delta_d = 16.96*read(:,14);
    throttle_dot_d = read(:,15);
    delta_dot_d = 16.96*read(:,16);
    aty_d = read(:,17);
    any_d = read(:,18);
    atx_d = read(:,19);
    anx_d = read(:,20);
    
    data.time = time_d;
    data.x = x_d;
    data.y = y_d;
    data.vx = vx_d;
    data.vy = vy_d;
    data.ax = ax_d;
    data.ay = ay_d;
    data.jx = jx_d;
    data.jy = jy_d;
    data.psi = psi_d;
    data.psi_dot = psi_dot_d;
    data.psi_ddot = psi_ddot_d;
    data.throttle = throttle_d;
    data.delta = delta_d;
    data.throttle_dot = throttle_dot_d;
    data.delta_dot = delta_dot_d;
    data.aty = aty_d;
    data.any = any_d;
    data.atx = atx_d;
    data.anx = anx_d;
    
 
    % plotting
    %Postion vs time
    figure('name', 'pos')
    subplot(1,2,1)
    plot(data.time,data.x,'LineWidth',1.0)
    title('X [m]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('Position X [m]','fontsize',12)
    legend('data\_pathx')


    subplot(1,2,2)
    plot(data.time,data.y,'LineWidth',1.0)
    title('Y [m]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('Position Y [m]','fontsize',12)
    legend('data\_pathy')

    % Velocity vs time
    figure('name', 'vel')
    subplot(1,2,1)
    plot(data.time,data.vx,'LineWidth',1.0)
    title('vx [m/s] local axis','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('Velocity x [m/s]','fontsize',12)
    legend('data\_vx')
    

    subplot(1,2,2)
    plot(data.time,data.vy,'LineWidth',1.0)
    title('vy [m/s] local axis','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('Velocity y [m/s]','fontsize',12)
    legend('calc\_vy')


     %Yaw and Yaw rate 
    figure('name', 'yaws')
    subplot(1,2,1)
    plot(data.time,data.psi*180/pi,'LineWidth',1.0)
    title('yaw [°]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('yaw [°]','fontsize',12)
    legend('calc\_psi')


    subplot(1,2,2)
    plot(data.time,data.psi_dot*180/pi,'LineWidth',1.0)
    title('yaw\_rate [°/s]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('yaw/s [°/s]','fontsize',12)
    legend('calc\_psi_dot')

     %deltas
    figure('name', 'delta')
    plot(data.time,data.delta*180/pi,'LineWidth',1.0)
    title('Delta [°]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('delta [°]','fontsize',12)

     %throttle
    figure('name', 'throttle')
    plot(data.time,data.throttle,'LineWidth',1.0)
    title('throttle [-]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('throttle [-]','fontsize',12)
    
    close all
      
end