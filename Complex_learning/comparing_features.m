function comparing_features(data_cl,file)
    
    font = 14;
    font_ax = 12;    
    
    % X(t)/Y(t)
    figure("name","Comparison X(t) and Y(t)")
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.x_cl,'Color',[0.8500 0.3250 0.0980],'LineWidth',2)  
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Horizontal distance [m]", 'fontsize',font,'fontweight','bold')
    grid on
     
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
    
    subplot(1, 2, 2)   
    plot(data_cl.time_cl,data_cl.y_cl,'Color',[0.8500 0.3250 0.0980],'LineWidth',2)  
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Vertical distance [m]", 'fontsize',font,'fontweight','bold')
    grid on
    
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
    
    % Path
    figure("name","Comparison paths")
    plot(data_cl.x_cl,data_cl.y_cl,'Color',[0.8500 0.3250 0.0980],'LineWidth',2)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Horizontal distance [m]", 'fontsize',font,'fontweight','bold')
    ylabel("Vertical distance [m]", 'fontsize',font,'fontweight','bold')
    grid on
        
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
    
    % VX(t)/VY(t)
    figure("name","Comparison velocities")
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.vx_cl,'Color',[0.8500 0.3250 0.0980],'LineWidth',2)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Horizontal velocity [m/s]", 'fontsize',font,'fontweight','bold')
    grid on
       
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
    
    subplot(1, 2, 2)    
    plot(data_cl.time_cl,data_cl.vy_cl,'Color',[0.8500 0.3250 0.0980],'LineWidth',2)  
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Vertical velocity [m/s]", 'fontsize',font,'fontweight','bold')
    grid on
    
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))

    
    % AX(t)/AY(t)
    figure("name","Comparison accelerations")
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.ax_cl,'Color',[0.8500 0.3250 0.0980],'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Horizontal acceleration [m/s²]", 'fontsize',font,'fontweight','bold')
    grid on
      
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
    
    subplot(1, 2, 2)    
    plot(data_cl.time_cl,data_cl.ay_cl,'Color',[0.8500 0.3250 0.0980],'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Vertical acceleration [m/s²]", 'fontsize',font,'fontweight','bold')
    grid on
     
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
    
    
    % AtX(t)/AnX(t)
    figure("name","Comparison longitudinal accelerations")
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.atx_cl,'Color',[0.8500 0.3250 0.0980],'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Tangential ax [m/s²]", 'fontsize',font,'fontweight','bold')
    grid on
      
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
    
    subplot(1, 2, 2)   
    plot(data_cl.time_cl,data_cl.anx_cl,'Color',[0.8500 0.3250 0.0980],'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Normal ax [m/s²]", 'fontsize',font,'fontweight','bold')
    grid on
     
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
    
    % AtY(t)/AnY(t)
    figure("name","Comparison lateral accelerations")
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.aty_cl,'Color',[0.8500 0.3250 0.0980],'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Tangential ay [m/s²]", 'fontsize',font,'fontweight','bold')
    grid on
      
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
    
    subplot(1, 2, 2)    
    plot(data_cl.time_cl,data_cl.any_cl,'Color',[0.8500 0.3250 0.0980],'LineWidth',2)  
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Normal ay [m/s²]", 'fontsize',font,'fontweight','bold')
    grid on
    
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
    
      
    % JX(t)/JY(t)
    figure("name","Comparison jerks")
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.jx_cl,'Color',[0.8500 0.3250 0.0980],'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Horizontal jerk [m/s³]", 'fontsize',font,'fontweight','bold')
    grid on
      
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
    
    subplot(1, 2, 2)  
    plot(data_cl.time_cl,data_cl.jy_cl,'Color',[0.8500 0.3250 0.0980],'LineWidth',2)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("Vertical jerk [m/s³]", 'fontsize',font,'fontweight','bold')
    grid on
      
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
    
    % yaw(t)/yaw_dot(t)
    figure("name","yaw angle")
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.psi_cl*180/pi,'Color',[0.8500 0.3250 0.0980],'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("yaw angle [°]", 'fontsize',font,'fontweight','bold')
    grid on
      
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
    
    subplot(1, 2, 2) 
    plot(data_cl.time_cl,data_cl.psi_dot_cl*180/pi,'Color',[0.8500 0.3250 0.0980],'LineWidth',2)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("yaw rate [°/s]", 'fontsize',font,'fontweight','bold')
    grid on
      
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
    
    % % yaw_acc(t)
    figure("name","yaw angle acc")
    plot(data_cl.time_cl,data_cl.psi_ddot_cl*180/pi,'Color',[0.8500 0.3250 0.0980],'LineWidth',2)   
   
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("yaw angle acc [°/s²]", 'fontsize',font,'fontweight','bold')
    grid on
     hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
    
    % tr(t)/delta(t) DELTA = angle front wheel
    figure("name","tr&delta")
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.throttle_cl,'Color',[0.8500 0.3250 0.0980],'LineWidth',2)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("throttle [-]", 'fontsize',font,'fontweight','bold')
    grid on
       
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
    
    subplot(1, 2, 2)    
    plot(data_cl.time_cl,data_cl.delta_cl*180/pi,'Color',[0.8500 0.3250 0.0980],'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("delta [°]", 'fontsize',font,'fontweight','bold')
    grid on
     
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
 
    % tr_dot(t)/delta_dot(t) DELTA = angle front wheel
    figure("name","tr&delta")
    
    subplot(1, 2, 1)
    plot(data_cl.time_cl,data_cl.throttle_dot_cl,'Color',[0.8500 0.3250 0.0980],'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("throttle_dot [1/s]", 'fontsize',font,'fontweight','bold')
    grid on
      
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))
    
    subplot(1, 2, 2)    
    plot(data_cl.time_cl,data_cl.delta_dot_cl*180/pi,'Color',[0.8500 0.3250 0.0980],'LineWidth',2) 
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("Time [s]", 'fontsize',font,'fontweight','bold')
    ylabel("delta_dot [°/s]", 'fontsize',font,'fontweight','bold')
    grid on
     
    hold on
    legend("Data: " + file(6:11)+"_"+ file(13:17))


 
end 