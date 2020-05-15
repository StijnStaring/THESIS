function comparing_features(data_cl)
    
    font = 14;
    font_ax = 12;    
    N = size(data_cl.x_cl,2) - 1;
    
    % X(t)/Y(t)
    figure(1)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.x_cl,'LineWidth',2)  
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Horizontal distance [m]", 'fontsize',font,'fontweight','bold')
    grid on
     
    hold on
    
    
    subplot(1, 2, 2)   
    plot(data_cl.time_cl,data_cl.y_cl,'LineWidth',2)  
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Vertical distance [m]", 'fontsize',font,'fontweight','bold')
    grid on
    
    hold on
    
    
    % Path
    figure(2)
    plot(data_cl.x_cl,data_cl.y_cl,'LineWidth',2)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Horizontal distance [m]", 'fontsize',font,'fontweight','bold')
    ylabel("Vertical distance [m]", 'fontsize',font,'fontweight','bold')
    grid on
        
    hold on
    
    
    % VX(t)/VY(t)
    figure(3)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.vx_cl,'LineWidth',2)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Horizontal velocity [m/s]", 'fontsize',font,'fontweight','bold')
    grid on
       
    hold on
    
    
    subplot(1, 2, 2)    
    plot(data_cl.time_cl,data_cl.vy_cl,'LineWidth',2)  
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Vertical velocity [m/s]", 'fontsize',font,'fontweight','bold')
    grid on
    
    hold on
    

    
    % AX(t)/AY(t)
    figure(4)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.ax_cl,'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Horizontal acceleration [m/s²]", 'fontsize',font,'fontweight','bold')
    grid on
      
    hold on
    
    
    subplot(1, 2, 2)    
    plot(data_cl.time_cl,data_cl.ay_cl,'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Vertical acceleration [m/s²]", 'fontsize',font,'fontweight','bold')
    grid on
     
    hold on
    
    
    
    % AtX(t)/AnX(t)
    figure(5)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.atx_cl,'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Tangential ax [m/s²]", 'fontsize',font,'fontweight','bold')
    grid on
      
    hold on
    
    
    subplot(1, 2, 2)   
    plot(data_cl.time_cl,data_cl.anx_cl,'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Normal ax [m/s²]", 'fontsize',font,'fontweight','bold')
    grid on
     
    hold on
    
    
    % AtY(t)/AnY(t)
    figure(6)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.aty_cl,'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Tangential ay [m/s²]", 'fontsize',font,'fontweight','bold')
    grid on
      
    hold on
    
    
    subplot(1, 2, 2)    
    plot(data_cl.time_cl,data_cl.any_cl,'LineWidth',2)  
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Normal ay [m/s²]", 'fontsize',font,'fontweight','bold')
    grid on
    
    hold on
    
    
      
    % JX(t)/JY(t)
    figure(7)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.jx_cl,'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Horizontal jerk [m/s³]", 'fontsize',font,'fontweight','bold')
    grid on
      
    hold on
    
    
    subplot(1, 2, 2)  
    plot(data_cl.time_cl,data_cl.jy_cl,'LineWidth',2)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Vertical jerk [m/s³]", 'fontsize',font,'fontweight','bold')
    grid on
      
    hold on
    
    
    % yaw(t)/yaw_dot(t)
    figure(8)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.psi_cl*180/pi,'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("yaw angle [°]", 'fontsize',font,'fontweight','bold')
    grid on
      
    hold on
    
    
    subplot(1, 2, 2) 
    plot(data_cl.time_cl,data_cl.psi_dot_cl*180/pi,'LineWidth',2)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("yaw rate [°/s]", 'fontsize',font,'fontweight','bold')
    grid on
      
    hold on
    
    
    % % yaw_acc(t)
    figure(9)
    plot(data_cl.time_cl,data_cl.psi_ddot_cl*180/pi,'LineWidth',2)   
   
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("yaw angle acc [°/s²]", 'fontsize',font,'fontweight','bold')
    grid on
     hold on
    
    
    % tr(t)/delta(t) DELTA = angle front wheel
    figure(10)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.throttle_cl,'LineWidth',2)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("throttle [-]", 'fontsize',font,'fontweight','bold')
    grid on
       
    hold on
    
    
    subplot(1, 2, 2)    
    plot(data_cl.time_cl,data_cl.delta_cl*180/pi,'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("delta [°]", 'fontsize',font,'fontweight','bold')
    grid on
     
    hold on
    
 
    % tr_dot(t)/delta_dot(t) DELTA = angle front wheel
    figure(11)
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl(1,1:N),data_cl.throttle_dot_cl,'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("throttle\_dot [1/s]", 'fontsize',font,'fontweight','bold')
    grid on
      
    hold on
    
    
    subplot(1, 2, 2)    
    plot(data_cl.time_cl(1,1:N),data_cl.delta_dot_cl*180/pi,'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("delta\_dot [°/s]", 'fontsize',font,'fontweight','bold')
    grid on
     
    hold on
    


 
end 