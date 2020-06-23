function comparing_features(data_cl)
    
    
    N = size(data_cl.x_cl,2) - 1;
    
    % X(t)/Y(t)
    figure(1)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.x_cl,'LineWidth',2)  
    
    hold on
    subplot(1, 2, 2)   
    plot(data_cl.time_cl,data_cl.y_cl,'LineWidth',2)  
    hold on
    
    % Path
    figure(2)
    plot(data_cl.x_cl,data_cl.y_cl,'LineWidth',2)
    hold on
    
    
    % VX(t)/VY(t)
    figure(3)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.vx_cl,'LineWidth',2)
    hold on
    
    
    subplot(1, 2, 2)    
    plot(data_cl.time_cl,data_cl.vy_cl,'LineWidth',2)  
    hold on
    

    
    % AX(t)/AY(t)
    figure(4)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.ax_cl,'LineWidth',2) 
    hold on
    
    
    subplot(1, 2, 2)    
    plot(data_cl.time_cl,data_cl.ay_cl,'LineWidth',2) 
    hold on
    
    
    
    % AtX(t)/AnX(t)
    figure(5)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.atx_cl,'LineWidth',2) 
    hold on
    
    
    subplot(1, 2, 2)   
    plot(data_cl.time_cl,data_cl.anx_cl,'LineWidth',2) 
    hold on
    
    
    % AtY(t)/AnY(t)
    figure(6)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.aty_cl,'LineWidth',2) 
    hold on
    
    
    subplot(1, 2, 2)    
    plot(data_cl.time_cl,data_cl.any_cl,'LineWidth',2)  
    hold on
    
    
      
    % JX(t)/JY(t)
    figure(7)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.jx_cl,'LineWidth',2) 
    hold on    
    
    subplot(1, 2, 2)  
    plot(data_cl.time_cl,data_cl.jy_cl,'LineWidth',2)
     hold on   
    
    % yaw(t)/yaw_dot(t)
    figure(8)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.psi_cl*180/pi,'LineWidth',2) 
    hold on
    
    subplot(1, 2, 2) 
    plot(data_cl.time_cl,data_cl.psi_dot_cl*180/pi,'LineWidth',2)
    hold on
    
    
    % % yaw_acc(t)
    figure(9)
    plot(data_cl.time_cl,data_cl.psi_ddot_cl*180/pi,'LineWidth',2)   
   hold on
    
    
    
    % tr(t)/delta(t) DELTA = angle front wheel
    figure(10)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.throttle_cl,'LineWidth',2)
    hold on
    
    
    subplot(1, 2, 2)    
    plot(data_cl.time_cl,data_cl.delta_cl*180/pi,'LineWidth',2) 
     hold on   
 
    % tr_dot(t)/delta_dot(t) DELTA = angle front wheel
    figure(11)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl(1,1:N),data_cl.throttle_dot_cl,'LineWidth',2) 
    hold on
    
    
    subplot(1, 2, 2)    
    plot(data_cl.time_cl(1,1:N),data_cl.delta_dot_cl*180/pi,'LineWidth',2) 
    hold on
    


 
end 