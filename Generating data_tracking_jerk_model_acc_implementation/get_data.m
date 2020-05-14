function data = get_data(file)
    % This function is for defining the data parameters in a structure
    % 'data'. 
    % Read data
    read = csvread(file,1);
    % Struct data
    data = struct;
    time_d = read(:,1);
    T_pl = time_d(2,1) - time_d(1,1);
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
    
    % Adding a straight part to the reference path
    % at start
    T_e = 15;
    vx_start = vx_d(1,1);
    N_e = int64(T_e/T_pl); % Last point equals the first point of the original dataset. 
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
    N_f = int64(T_f/T_pl); % Last point equals the first point of the original dataset. 
    time_f = linspace(0,T_f,N_f+1)';
    x_f = linspace(0,vx_end*T_f,N_f+1)';
    y_f = zeros(N_f,1); 
    vx_f = vx_end*ones(N_f,1);
    vy_f = zeros(N_f,1); 
    psi_f = zeros(N_f,1); 
    psi_dot_f = zeros(N_f,1); 
    psi_ddot_f = zeros(N_f,1); 
    throttle_f = throttle_d(end,1)*ones(N_f,1);
    delta_f = zeros(N_f,1); 
    throttle_dot_f = zeros(N_f,1); 
    delta_dot_f = zeros(N_f,1); 
    ax_f = zeros(N_f,1); 
    atx_f = zeros(N_f,1);
    anx_f = zeros(N_f,1);
    ay_f = zeros(N_f,1); 
    aty_f = zeros(N_f,1);
    any_f = zeros(N_f,1);
    jx_f = zeros(N_f,1);
    jy_f = zeros(N_f,1);
       
    
    % Loading the signals in the datastruct
    data.time = [time_e(1:N_e);time_d+time_e(end)];
    data.x = [x_e(1:N_e);x_d+x_e(end)];
    data.y = [y_e;y_d];
    data.vx = [vx_e;vx_d];
    data.vy = [vy_e;vy_d];
    data.ax = [ax_e;ax_d];
    data.ay = [ay_e;ay_d];
    data.jx = [jx_e;jx_d];
    data.jy = [jy_e;jy_d];
    data.psi = [psi_e;psi_d];
    data.psi_dot = [psi_dot_e;psi_dot_d];
    data.psi_ddot = [psi_ddot_e;psi_ddot_d];
    data.throttle = [throttle_e;throttle_d];
    data.delta = [delta_e;delta_d];
    data.throttle_dot = [throttle_dot_e;throttle_dot_d];
    data.delta_dot = [delta_dot_e;delta_dot_d];
    data.aty = [aty_e;aty_d];
    data.any = [any_e;any_d];
    data.atx = [atx_e;atx_d];
    data.anx = [anx_e;anx_d];
    
 
    % plotting
    %Postion vs time
    figure('name', 'pos')
    subplot(1,2,1)
    plot(data.time,data.x,'LineWidth',1.0)
    hold on
    plot(time_d,x_d,'LineWidth',1.0,'Color',[0.8500 0.3250 0.0980])
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
    
     % Acceleartion vs time
    figure('name', 'acc')
    subplot(1,2,1)
    plot(data.time,data.ax,'LineWidth',1.0)
    title('ax [m/s²] local axis','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('acc x [m/s²]','fontsize',12)
    legend('data\_ax')
    

    subplot(1,2,2)
    plot(data.time,data.ay,'LineWidth',1.0)
    title('ay [m/s²] local axis','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('acc y [m/s]','fontsize',12)
    legend('calc\_ay')
    
    % Acceleartion y vs time
    figure('name', 'accy')
    subplot(1,2,1)
    plot(data.time,data.aty,'LineWidth',1.0)
    title('aty [m/s²] local axis','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('acc ty [m/s²]','fontsize',12)
    legend('data\_aty')
    

    subplot(1,2,2)
    plot(data.time,data.any,'LineWidth',1.0)
    title('any [m/s²] local axis','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('accny [m/s]','fontsize',12)
    legend('calc\_any')
    
    % Acceleartion x vs time
    figure('name', 'accx')
    subplot(1,2,1)
    plot(data.time,data.atx,'LineWidth',1.0)
    title('atx [m/s²] local axis','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('acc tx [m/s²]','fontsize',12)
    legend('data\_atx')
    

    subplot(1,2,2)
    plot(data.time,data.anx,'LineWidth',1.0)
    title('anx [m/s²] local axis','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('accnx [m/s]','fontsize',12)
    legend('calc\_anx')
    
    %Jerk vs time
    figure('name', 'jerk')
    subplot(1,2,1)
    plot(data.time,data.jx,'LineWidth',1.0)
    title('jx [m/s³]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('jerk jx [m]','fontsize',12)
    legend('data\_jerk x')


    subplot(1,2,2)
    plot(data.time,data.jy,'LineWidth',1.0)
    title('jy [m/s³]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('jerk jy [m/s³]','fontsize',12)
    legend('data\_jerk y')


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
    
    %Yaw acc and Yaw rate 
    figure('name', 'yaws acc')
    plot(data.time,data.psi_ddot*180/pi,'LineWidth',1.0)
    title('yaw_acc [°/s²]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('yaw acc [°/s²]','fontsize',12)
    legend('calc\_psi ddot')

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
    
    % deltas dot
    figure('name', 'delta_dot')
    plot(data.time,data.delta_dot*180/pi,'LineWidth',1.0)
    title('Delta\_dot [°/s]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('delta\_dot [°/s]','fontsize',12)

    % throttle dot
    figure('name', 'throttle\_dot')
    plot(data.time,data.throttle_dot,'LineWidth',1.0)
    title('throttle [1/s]','fontsize',12,'fontweight','bold')
    xlabel('t [s]','fontsize',12)
    ylabel('throttle [1/s]','fontsize',12)
    
    close all
      
end