function data_out = get_data(data_temp)

    % get data: Nx1
    data_out = struct;
    time_d = data_temp.time_cl';
    T_pl = time_d(2,1) - time_d(1,1);
    x_d = data_temp.x_cl';
    y_d = data_temp.y_cl';
    vx_d =data_temp.vx_cl';
    vy_d = data_temp.vy_cl';
    ax_d = data_temp.ax_cl'; % total acc
    ay_d = data_temp.ay_cl'; % total acc
    jx_d = data_temp.jx_cl';
    jy_d = data_temp.jy_cl';
    psi_d = data_temp.psi_cl';
    psi_dot_d = data_temp.psi_dot_cl';
    psi_ddot_d = data_temp.psi_ddot_cl';
    throttle_d = data_temp.throttle_cl';
    delta_d = data_temp.delta_cl';
    throttle_dot_d = data_temp.throttle_dot_cl'; % one value less
    delta_dot_d = data_temp.delta_dot_cl'; % one value less
    aty_d = data_temp.aty_cl';
    any_d = data_temp.any_cl';
    atx_d = data_temp.atx_cl';
    anx_d = data_temp.anx_cl';
    
    % Adding a straight part to the reference path
    % at start
    T_e = 15;
    vx_start = vx_d(1,1);
    N_e = int64(T_e/T_pl); % Last point equals the first point of the original data_outset. 
    time_e = linspace(0,T_e,N_e+1)';
    x_e = linspace(0,vx_start*T_e,N_e+1)';
    y_e = zeros(N_e,1); 
    vx_e = vx_start*ones(N_e,1);
    vy_e = zeros(N_e,1); 
    psi_e = zeros(N_e,1); 
    psi_dot_e = zeros(N_e,1); 
    psi_ddot_e = zeros(N_e,1); 
    throttle_e = throttle_d(1,1)*ones(N_e,1);
    delta_e = zeros(N_e,1); 
    throttle_dot_e = zeros(N_e,1); 
    delta_dot_e = zeros(N_e,1); 
    ax_e = zeros(N_e,1); 
    atx_e = zeros(N_e,1);
    anx_e = zeros(N_e,1);
    ay_e = zeros(N_e,1); 
    aty_e = zeros(N_e,1);
    any_e = zeros(N_e,1);
    jx_e = zeros(N_e,1);
    jy_e = zeros(N_e,1);
    
    % at finish
    T_f = 2.5;
    vx_end = vx_d(end,1);
    y_end = y_d(end,1);
    N_f = int64(T_f/T_pl); % Last point equals the first point of the original data_outset. 
    time_f = linspace(0,T_f,N_f+1)';
    x_f = linspace(0,vx_end*T_f,N_f+1)';
    y_f = y_end*ones(N_f,1); 
    vx_f = vx_end*ones(N_f,1);
    vy_f = zeros(N_f,1); 
    psi_f = zeros(N_f,1); 
    psi_dot_f = zeros(N_f,1); 
    psi_ddot_f = zeros(N_f,1); 
    throttle_f = throttle_d(end,1)*ones(N_f,1);
    delta_f = zeros(N_f,1); 
    throttle_dot_f = zeros(N_f+1,1); % extra zero value given to obtain the same length
    delta_dot_f = zeros(N_f+1,1); % extra zero value given to obtain the same length
    ax_f = zeros(N_f,1); 
    atx_f = zeros(N_f,1);
    anx_f = zeros(N_f,1);
    ay_f = zeros(N_f,1); 
    aty_f = zeros(N_f,1);
    any_f = zeros(N_f,1);
    jx_f = zeros(N_f,1);
    jy_f = zeros(N_f,1);
        
    % Loading the signals in the data_outstruct
    data_out.time = [time_e(1:N_e);time_d+time_e(end);time_d(end)+time_e(end) + time_f(2:N_f+1)];
    data_out.x = [x_e(1:N_e);x_d+x_e(end);x_d(end)+x_e(end) + x_f(2:N_f+1)];
    data_out.y = [y_e;y_d;y_f];
    data_out.vx = [vx_e;vx_d;vx_f];
    data_out.vy = [vy_e;vy_d;vy_f];
    data_out.ax = [ax_e;ax_d;ax_f];
    data_out.ay = [ay_e;ay_d;ay_f];
    data_out.jx = [jx_e;jx_d;jx_f];
    data_out.jy = [jy_e;jy_d;jy_f];
    data_out.psi = [psi_e;psi_d;psi_f];
    data_out.psi_dot = [psi_dot_e;psi_dot_d;psi_dot_f];
    data_out.psi_ddot = [psi_ddot_e;psi_ddot_d;psi_ddot_f];
    data_out.throttle = [throttle_e;throttle_d;throttle_f];
    data_out.delta = [delta_e;delta_d;delta_f];
    data_out.throttle_dot = [throttle_dot_e;throttle_dot_d;throttle_dot_f];
    data_out.delta_dot = [delta_dot_e;delta_dot_d;delta_dot_f];
    data_out.aty = [aty_e;aty_d;aty_f];
    data_out.any = [any_e;any_d;any_f];
    data_out.atx = [atx_e;atx_d;atx_f];
    data_out.anx = [anx_e;anx_d;anx_f];
    
 
    % plotting
    %Postion vs time
    figure('name', 'pos')
    subplot(1,2,1)
    plot(data_out.time,data_out.x,'LineWidth',1.0)
    hold on
    plot(time_d,x_d,'LineWidth',1.0,'Color',[0.8500 0.3250 0.0980])
    title('X [m]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('Position X [m]','fontsize',12)
    legend('data_out\_pathx')


    subplot(1,2,2)
    plot(data_out.time,data_out.y,'LineWidth',1.0)
    title('Y [m]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('Position Y [m]','fontsize',12)
    legend('data_out\_pathy')

    % Velocity vs time
    figure('name', 'vel')
    subplot(1,2,1)
    plot(data_out.time,data_out.vx,'LineWidth',1.0)
    title('vx [m/s] local axis','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('Velocity x [m/s]','fontsize',12)
    legend('data_out\_vx')
    

    subplot(1,2,2)
    plot(data_out.time,data_out.vy,'LineWidth',1.0)
    title('vy [m/s] local axis','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('Velocity y [m/s]','fontsize',12)
    legend('calc\_vy')
    
     % Acceleartion vs time
    figure('name', 'acc')
    subplot(1,2,1)
    plot(data_out.time,data_out.ax,'LineWidth',1.0)
    title('ax [m/s²] local axis','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('acc x [m/s²]','fontsize',12)
    legend('data_out\_ax')
    

    subplot(1,2,2)
    plot(data_out.time,data_out.ay,'LineWidth',1.0)
    title('ay [m/s²] local axis','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('acc y [m/s]','fontsize',12)
    legend('calc\_ay')
    
    % Acceleartion y vs time
    figure('name', 'accy')
    subplot(1,2,1)
    plot(data_out.time,data_out.aty,'LineWidth',1.0)
    title('aty [m/s²] local axis','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('acc ty [m/s²]','fontsize',12)
    legend('data_out\_aty')
    

    subplot(1,2,2)
    plot(data_out.time,data_out.any,'LineWidth',1.0)
    title('any [m/s²] local axis','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('accny [m/s]','fontsize',12)
    legend('calc\_any')
    
    % Acceleartion x vs time
    figure('name', 'accx')
    subplot(1,2,1)
    plot(data_out.time,data_out.atx,'LineWidth',1.0)
    title('atx [m/s²] local axis','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('acc tx [m/s²]','fontsize',12)
    legend('data_out\_atx')
    

    subplot(1,2,2)
    plot(data_out.time,data_out.anx,'LineWidth',1.0)
    title('anx [m/s²] local axis','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('accnx [m/s]','fontsize',12)
    legend('calc\_anx')
    
    %Jerk vs time
    figure('name', 'jerk')
    subplot(1,2,1)
    plot(data_out.time,data_out.jx,'LineWidth',1.0)
    title('jx [m/s³]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('jerk jx [m]','fontsize',12)
    legend('data_out\_jerk x')


    subplot(1,2,2)
    plot(data_out.time,data_out.jy,'LineWidth',1.0)
    title('jy [m/s³]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('jerk jy [m/s³]','fontsize',12)
    legend('data_out\_jerk y')


     %Yaw and Yaw rate 
    figure('name', 'yaws')
    subplot(1,2,1)
    plot(data_out.time,data_out.psi*180/pi,'LineWidth',1.0)
    title('yaw [°]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('yaw [°]','fontsize',12)
    legend('calc\_psi')


    subplot(1,2,2)
    plot(data_out.time,data_out.psi_dot*180/pi,'LineWidth',1.0)
    title('yaw\_rate [°/s]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('yaw/s [°/s]','fontsize',12)
    legend('calc\_psi_dot')
    
    %Yaw acc and Yaw rate 
    figure('name', 'yaws acc')
    plot(data_out.time,data_out.psi_ddot*180/pi,'LineWidth',1.0)
    title('yaw_acc [°/s²]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('yaw acc [°/s²]','fontsize',12)
    legend('calc\_psi ddot')

     %deltas
    figure('name', 'delta')
    plot(data_out.time,data_out.delta*180/pi,'LineWidth',1.0)
    title('Delta [°]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('delta [°]','fontsize',12)

     %throttle
    figure('name', 'throttle')
    plot(data_out.time,data_out.throttle,'LineWidth',1.0)
    title('throttle [-]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('throttle [-]','fontsize',12)
    
    % deltas dot
    figure('name', 'delta_dot')
    plot(data_out.time,data_out.delta_dot*180/pi,'LineWidth',1.0)
    title('Delta\_dot [°/s]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('delta\_dot [°/s]','fontsize',12)

    % throttle dot
    figure('name', 'throttle\_dot')
    plot(data_out.time,data_out.throttle_dot,'LineWidth',1.0)
    title('throttle [1/s]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('throttle [1/s]','fontsize',12)
    
    close all
      
end