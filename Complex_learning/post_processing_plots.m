function post_processing_plots(his_f_calc_rel,his_weights,his_multi_grads,his_grad_current,his_diff_theta)

    font = 14;
    font_ax = 12;

    % plot frel
    figure("name","Convergence of features")
    subplot(2, 3,1)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("f\_rel", 'fontsize',font,'fontweight','bold')
    title("Feature 1", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_f_calc_rel);
    for k=1:numel(fn)
        temp = his_f_calc_rel.(fn{k});
        scatter(k,temp(1,1),'o','MarkerFaceColor', 'b')
    end
    
    
    subplot(2, 3,2)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("f\_rel", 'fontsize',font,'fontweight','bold')
    title("Feature 2", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_f_calc_rel);
    for k=1:numel(fn)
        temp = his_f_calc_rel.(fn{k});
        scatter(k,temp(1,2),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,3)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("f\_rel", 'fontsize',font,'fontweight','bold')
    title("Feature 3", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_f_calc_rel);
    for k=1:numel(fn)
        temp = his_f_calc_rel.(fn{k});
        scatter(k,temp(1,3),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,4)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("f\_rel", 'fontsize',font,'fontweight','bold')
    title("Feature 4", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_f_calc_rel);
    for k=1:numel(fn)
        temp = his_f_calc_rel.(fn{k});
        scatter(k,temp(1,4),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,5)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("f\_rel", 'fontsize',font,'fontweight','bold')
    title("Feature 5", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_f_calc_rel);
    for k=1:numel(fn)
        temp = his_f_calc_rel.(fn{k});
        scatter(k,temp(1,5),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,6)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("f\_rel", 'fontsize',font,'fontweight','bold')
    title("Feature 6", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_f_calc_rel);
    for k=1:numel(fn)
        temp = his_f_calc_rel.(fn{k});
        scatter(k,temp(1,6),'o','MarkerFaceColor', 'b')
    end
      

 
%plot current gradient
    figure("name","Current gradient")
    subplot(2, 3,1)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("curr gradiënt", 'fontsize',font,'fontweight','bold')
    title("Feature 1", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_grad_current);
    for k=1:numel(fn)
        temp = his_grad_current.(fn{k});
        scatter(k,temp(1,1),'o','MarkerFaceColor', 'b')
    end
    
    
    subplot(2, 3,2)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("curr gradiënt", 'fontsize',font,'fontweight','bold')
    title("Feature 2", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_grad_current);
    for k=1:numel(fn)
        temp = his_grad_current.(fn{k});
        scatter(k,temp(1,2),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,3)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("curr gradiënt", 'fontsize',font,'fontweight','bold')
    title("Feature 3", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_grad_current);
    for k=1:numel(fn)
        temp = his_grad_current.(fn{k});
        scatter(k,temp(1,3),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,4)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("curr gradiënt", 'fontsize',font,'fontweight','bold')
    title("Feature 4", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_grad_current);
    for k=1:numel(fn)
        temp = his_grad_current.(fn{k});
        scatter(k,temp(1,4),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,5)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("curr gradiënt", 'fontsize',font,'fontweight','bold')
    title("Feature 5", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_grad_current);
    for k=1:numel(fn)
        temp = his_grad_current.(fn{k});
        scatter(k,temp(1,5),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,6)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("curr gradiënt", 'fontsize',font,'fontweight','bold')
    title("Feature 6", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_grad_current);
    for k=1:numel(fn)
        temp = his_grad_current.(fn{k});
        scatter(k,temp(1,6),'o','MarkerFaceColor', 'b')
    end


      % Weights 
    figure("name","Weights")
    subplot(2, 3,1)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Weight", 'fontsize',font,'fontweight','bold')
    title("Feature 1", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_weights);
    for k=1:numel(fn)
        temp = his_weights.(fn{k});
        scatter(k,temp(1,1),'o','MarkerFaceColor', 'b')
    end
    
    
    subplot(2, 3,2)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Weight", 'fontsize',font,'fontweight','bold')
    title("Feature 2", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_weights);
    for k=1:numel(fn)
        temp = his_weights.(fn{k});
        scatter(k,temp(1,2),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,3)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Weight", 'fontsize',font,'fontweight','bold')
    title("Feature 3", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_weights);
    for k=1:numel(fn)
        temp = his_weights.(fn{k});
        scatter(k,temp(1,3),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,4)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Weight", 'fontsize',font,'fontweight','bold')
    title("Feature 4", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_weights);
    for k=1:numel(fn)
        temp = his_weights.(fn{k});
        scatter(k,temp(1,4),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,5)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Weight", 'fontsize',font,'fontweight','bold')
    title("Feature 5", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_weights);
    for k=1:numel(fn)
        temp = his_weights.(fn{k});
        scatter(k,temp(1,5),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,6)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Weight", 'fontsize',font,'fontweight','bold')
    title("Feature 6", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_weights);
    for k=1:numel(fn)
        temp = his_weights.(fn{k});
        scatter(k,temp(1,6),'o','MarkerFaceColor', 'b')
    end

    % Plotting diff of theta over iterations
    figure("name","Diff theta")
    subplot(2, 3,1)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("diff weight", 'fontsize',font,'fontweight','bold')
    title("Feature 1", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_weights);
    for k=2:numel(fn)
        temp = his_weights.(fn{k}) - his_weights.(fn{k-1});
        scatter(k,temp(1,1),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,2)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("diff weight", 'fontsize',font,'fontweight','bold')
    title("Feature 2", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_weights);
    for k=2:numel(fn)
        temp = his_weights.(fn{k}) - his_weights.(fn{k-1});
        scatter(k,temp(1,2),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,3)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("diff weight", 'fontsize',font,'fontweight','bold')
    title("Feature 3", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_weights);
    for k=2:numel(fn)
        temp = his_weights.(fn{k}) - his_weights.(fn{k-1});
        scatter(k,temp(1,3),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,4)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("diff weight", 'fontsize',font,'fontweight','bold')
    title("Feature 4", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_weights);
    for k=2:numel(fn)
        temp = his_weights.(fn{k}) - his_weights.(fn{k-1});
        scatter(k,temp(1,4),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,5)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("diff weight", 'fontsize',font,'fontweight','bold')
    title("Feature 5", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_weights);
    for k=2:numel(fn)
        temp = his_weights.(fn{k}) - his_weights.(fn{k-1});
        scatter(k,temp(1,5),'o','MarkerFaceColor', 'b')
    end
    subplot(2, 3,6)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("diff weight", 'fontsize',font,'fontweight','bold')
    title("Feature 6", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_weights);
    for k=2:numel(fn)
        temp = his_weights.(fn{k}) - his_weights.(fn{k-1});
        scatter(k,temp(1,6),'o','MarkerFaceColor', 'b')
    end
     
    % multiplication of gradient
    figure("name","his_multi_grads")
    subplot(2, 3,1)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Case [-]", 'fontsize',font,'fontweight','bold')
    title("Feature 1", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_multi_grads);
    for k=1:numel(fn)
        temp = his_multi_grads.(fn{k});
        scatter(k,temp(1,1),'o','MarkerFaceColor', 'b')
    end
    
    
    subplot(2, 3,2)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Case [-]", 'fontsize',font,'fontweight','bold')
    title("Feature 2", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_multi_grads);
    for k=1:numel(fn)
        temp = his_multi_grads.(fn{k});
        scatter(k,temp(1,2),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,3)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Case [-]", 'fontsize',font,'fontweight','bold')
    title("Feature 3", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_multi_grads);
    for k=1:numel(fn)
        temp = his_multi_grads.(fn{k});
        scatter(k,temp(1,3),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,4)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Case [-]", 'fontsize',font,'fontweight','bold')
    title("Feature 4", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_multi_grads);
    for k=1:numel(fn)
        temp = his_multi_grads.(fn{k});
        scatter(k,temp(1,4),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,5)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Case [-]", 'fontsize',font,'fontweight','bold')
    title("Feature 5", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_multi_grads);
    for k=1:numel(fn)
        temp = his_multi_grads.(fn{k});
        scatter(k,temp(1,5),'o','MarkerFaceColor', 'b')
    end
    subplot(2, 3,6)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Case [-]", 'fontsize',font,'fontweight','bold')
    title("Feature 6", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_multi_grads);
    for k=1:numel(fn)
        temp = his_multi_grads.(fn{k});
        scatter(k,temp(1,6),'o','MarkerFaceColor', 'b')
    end

% history of difference with the chose weights
    figure("name","Diff chosen weights")
    subplot(2, 3,1)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Diff chosen", 'fontsize',font,'fontweight','bold')
    title("Feature 1", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_diff_theta);
    for k=1:numel(fn)
        temp = his_diff_theta.(fn{k});
        scatter(k,temp(1,1),'o','MarkerFaceColor', 'b')
    end
    
    
    subplot(2, 3,2)
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Diff chosen", 'fontsize',font,'fontweight','bold')
    title("Feature 2", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_diff_theta);
    for k=1:numel(fn)
        temp = his_diff_theta.(fn{k});
        scatter(k,temp(1,2),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,3)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Diff chosen", 'fontsize',font,'fontweight','bold')
    title("Feature 3", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_diff_theta);
    for k=1:numel(fn)
        temp = his_diff_theta.(fn{k});
        scatter(k,temp(1,3),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,4)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Diff chosen", 'fontsize',font,'fontweight','bold')
    title("Feature 4", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_diff_theta);
    for k=1:numel(fn)
        temp = his_diff_theta.(fn{k});
        scatter(k,temp(1,4),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,5)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Diff chosen", 'fontsize',font,'fontweight','bold')
    title("Feature 5", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_diff_theta);
    for k=1:numel(fn)
        temp = his_diff_theta.(fn{k});
        scatter(k,temp(1,5),'o','MarkerFaceColor', 'b')
    end
    
    subplot(2, 3,6)
    
    set(gca,'fontsize',font_ax,'fontweight','bold')
    xlabel("iteration [-]", 'fontsize',font,'fontweight','bold')
    ylabel("Diff chosen", 'fontsize',font,'fontweight','bold')
    title("Feature 6", 'fontsize',font,'fontweight','bold')
    grid on
    hold on
    fn = fieldnames(his_diff_theta);
    for k=1:numel(fn)
        temp = his_diff_theta.(fn{k});
        scatter(k,temp(1,6),'o','MarkerFaceColor', 'b')
    end
      

      
end
