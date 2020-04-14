function data = get_data(file,sampling_rate,N)
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
    delta_d = read(:,14);
    aty_d = read(:,15);
    any_d = read(:,16);  
    % adjusting sampling rate --> amount of samples per second
    
    [x,time] = resample(x_d,time_d,sampling_rate,1,1);
    [y,~] = resample(y_d,time_d,sampling_rate,1,1);
    [vx,~] = resample(vx_d,time_d,sampling_rate,1,1);
    [vy,~] = resample(vy_d,time_d,sampling_rate,1,1);
    [ax,~] = resample(ax_d,time_d,sampling_rate,1,1);
    [ay,~] = resample(ay_d,time_d,sampling_rate,1,1);
    [jx,~] = resample(jx_d,time_d,sampling_rate,1,1);
    [jy,~] = resample(jy_d,time_d,sampling_rate,1,1);
    [psi,~] = resample(psi_d,time_d,sampling_rate,1,1);
    [psi_dot,~] = resample(psi_dot_d,time_d,sampling_rate,1,1);
    [psi_ddot,~] = resample(psi_ddot_d,time_d,sampling_rate,1,1);
    [throttle,~] = resample(throttle_d,time_d,sampling_rate,1,1);
    [delta,~] = resample(delta_d,time_d,sampling_rate,1,1);
    [aty,~] = resample(aty_d,time_d,sampling_rate,1,1);
    [any,~] = resample(any_d,time_d,sampling_rate,1,1);
    
    data.time = zeros(length(time)+N-1,1);
    data.time(1:length(time),1) = time;
    data.time(length(time)+1:end,1) = time(end)+1/sampling_rate:1/sampling_rate:time(end)+(N-1)*1/sampling_rate;
    
    data.x = zeros(length(time)+N-1,1);
    data.x(1:length(time),1) = x;
    data.x(length(time)+1:end,1) = diff([x(end-1),x(end)])/diff([time(end-1),time(end)])*(time(end)+1/sampling_rate:1/sampling_rate:time(end)+(N-1)*1/sampling_rate);
    
    data.y = zeros(length(time)+N-1,1);
    data.y(1:length(time),1) = y;
    data.y(length(time)+1:end,1) = y(end)*ones(N-1,1);
    
    data.vx = zeros(length(time)+N-1,1);
    data.vx(1:length(time),1) = vx;
    data.vx(length(time)+1:end,1) = vx(end)*ones(N-1,1);
    
    data.vy = zeros(length(time)+N-1,1);
    data.vy(1:length(time),1) = vy;
    data.vy(length(time)+1:end,1) = 0*(time(end)+1/sampling_rate:1/sampling_rate:time(end)+(N-1)*1/sampling_rate);
    
    data.ax = zeros(length(time)+N-1,1);
    data.ax(1:length(time),1) = ax;
    data.ax(length(time)+1:end,1) = 0*(time(end)+1/sampling_rate:1/sampling_rate:time(end)+(N-1)*1/sampling_rate);
    
    data.ay = zeros(length(time)+N-1,1);
    data.ay(1:length(time),1) = ay;
    data.ay(length(time)+1:end,1) = 0*(time(end)+1/sampling_rate:1/sampling_rate:time(end)+(N-1)*1/sampling_rate);
    
    data.jx = zeros(length(time)+N-1,1);
    data.jx(1:length(time),1) = jx;
    data.jx(length(time)+1:end,1) = 0*(time(end)+1/sampling_rate:1/sampling_rate:time(end)+(N-1)*1/sampling_rate);
    
    data.jy = zeros(length(time)+N-1,1);
    data.jy(1:length(time),1) = jy;
    data.jy(length(time)+1:end,1) = 0*(time(end)+1/sampling_rate:1/sampling_rate:time(end)+(N-1)*1/sampling_rate);
    
    data.psi = zeros(length(time)+N-1,1);
    data.psi(1:length(time),1) = psi;
    data.psi(length(time)+1:end,1) = 0*(time(end)+1/sampling_rate:1/sampling_rate:time(end)+(N-1)*1/sampling_rate);
    
    data.psi_dot = zeros(length(time)+N-1,1);
    data.psi_dot(1:length(time),1) = psi_dot;
    data.psi_dot(length(time)+1:end,1) = 0*(time(end)+1/sampling_rate:1/sampling_rate:time(end)+(N-1)*1/sampling_rate);
    
    data.psi_ddot = zeros(length(time)+N-1,1);
    data.psi_ddot(1:length(time),1) = psi_ddot;
    data.psi_ddot(length(time)+1:end,1) = 0*(time(end)+1/sampling_rate:1/sampling_rate:time(end)+(N-1)*1/sampling_rate);
    
    data.throttle = zeros(length(time)+N-1,1);
    data.throttle(1:length(time),1) = throttle;
    data.throttle(length(time)+1:end,1) = 0*(time(end)+1/sampling_rate:1/sampling_rate:time(end)+(N-1)*1/sampling_rate);
    
    data.delta = zeros(length(time)+N-1,1);
    data.delta(1:length(time),1) = delta;
    data.delta(length(time)+1:end,1) = 0*(time(end)+1/sampling_rate:1/sampling_rate:time(end)+(N-1)*1/sampling_rate);
    
    data.aty = zeros(length(time)+N-1,1);
    data.aty(1:length(time),1) = aty;
    data.aty(length(time)+1:end,1) = 0*(time(end)+1/sampling_rate:1/sampling_rate:time(end)+(N-1)*1/sampling_rate);
    
    data.any = zeros(length(time)+N-1,1);
    data.any(1:length(time),1) = any;
    data.any(length(time)+1:end,1) = 0*(time(end)+1/sampling_rate:1/sampling_rate:time(end)+(N-1)*1/sampling_rate);
    
%     % plotting
%     Postion vs time
%     figure('name', 'pos')
%     subplot(1,2,1)
%     plot(data.time,data.x,'LineWidth',1.0)
%     title('X [m]','fontsize',12,'fontweight','bold')
%     xlabel('t [s]','fontsize',12)
%     ylabel('Position X [m]','fontsize',12)
%     legend('data\_pathx')
% 
% 
%     subplot(1,2,2)
%     plot(data.time,data.y,'LineWidth',1.0)
%     title('Y [m]','fontsize',12,'fontweight','bold')
%     xlabel('t [s]','fontsize',12)
%     ylabel('Position Y [m]','fontsize',12)
%     legend('data\_pathy')
% 
%      Velocity vs time
%     figure('name', 'vel')
%     subplot(1,2,1)
%     plot(data.time,data.vx,'LineWidth',1.0)
%     title('vx [m/s] local axis','fontsize',12,'fontweight','bold')
%     xlabel('t [s]','fontsize',12)
%     ylabel('Velocity x [m/s]','fontsize',12)
%     legend('data\_vx')
%     
% 
%     subplot(1,2,2)
%     plot(data.time,data.vy,'LineWidth',1.0)
%     title('vy [m/s] local axis','fontsize',12,'fontweight','bold')
%     xlabel('t [s]','fontsize',12)
%     ylabel('Velocity y [m/s]','fontsize',12)
%     legend('calc\_vy')
% 
% 
%      Yaw and Yaw rate 
%     figure('name', 'yaws')
%     subplot(1,2,1)
%     plot(data.time,data.psi*180/pi,'LineWidth',1.0)
%     title('yaw [°]','fontsize',12,'fontweight','bold')
%     xlabel('t [s]','fontsize',12)
%     ylabel('yaw [°]','fontsize',12)
%     legend('calc\_psi')
% 
% 
%     subplot(1,2,2)
%     plot(data.time,data.psi_dot*180/pi,'LineWidth',1.0)
%     title('yaw\_rate [°/s]','fontsize',12,'fontweight','bold')
%     xlabel('t [s]','fontsize',12)
%     ylabel('yaw/s [°/s]','fontsize',12)
%     legend('calc\_psi_dot')
% 
%      deltas
%     figure('name', 'delta')
%     plot(data.time,data.delta*180/pi,'LineWidth',1.0)
%     title('Delta [°]','fontsize',12,'fontweight','bold')
%     xlabel('t [s]','fontsize',12)
%     ylabel('delta [°]','fontsize',12)
% 
%      throttle
%     figure('name', 'throttle')
%     plot(data.time,data.throttle,'LineWidth',1.0)
%     title('throttle [-]','fontsize',12,'fontweight','bold')
%     xlabel('t [s]','fontsize',12)
%     ylabel('throttle [-]','fontsize',12)
    
    
end