%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% stijnstaring@hotmail.com
% 
% The program automatically uses all the csv files stored in the 'used_data' folder.
% Github stijn staring for more information
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
close all
clearvars

% Remarks
% No normalization necessary of gradient --> size doesn't matter in RPROP implementation.
% Optimization objective is normalized in order to have dimensionless weights --> better start (+-equal size of optimization terms at start)

% Defining weights
%%%%%%%%%%%%%%%%%%
his_diff_theta = struct();
his_multi_grads = struct();
his_grad_current = struct();
his_weights = struct();
his_exception = struct();
his_update = struct();
his_del_theta_prev = struct();
his_f_calc_rel = struct(); 
amount_features = 6;
rec = 1;
N = 1000;
tol = 1e-3;
max_iterations = 300;
% Comfort cost function: ax**2+t1*ay**2+t2*jx**2+t3*jy**2+t4*(vx-vdes)**2+t5*(y-ydes)**2
theta = [1.0,1.0,1.0,1.0,1.0,1.0];

% RPROP variables
del_0 = 0.1;
exception = zeros(1,amount_features);
del_theta_prev = zeros(1,amount_features);
update = del_0*ones(1,amount_features);

% Other
%%%%%%%%%%%%%%%%%
theta_tracker = struct();
theta_tracker.("iteration_"+num2str(rec)) = theta;
his_weights.("iteration_"+num2str(rec)) = theta;
theta_chosen = [4.0,5.0,1.0,6.0,1.0,2.0];% This is the theta used to generate the data

file_list = {'DCA2_V22.22_L3.47.csv'};
data_list  = cell(1,length(file_list));
for i = 1:1:length(file_list)
    file = file_list{1,i};
    fprintf("\n The name of the file: %s\n", file);
    data_cl = import_data2(file, 1);   
    data_list{1,i} = data_cl;
    comparing_features(data_cl)
end

%  Calculate the averaged
av_features_data = zeros(1,amount_features);
for k = 1:1:length(file_list)
    av_features_data = av_features_data + data_list{1,k}.features;
    av_features_data = av_features_data./length(file_list);
end

solutions = cell(1,max_iterations);
converged = 0;
grad_prev = zeros(1,amount_features);

% while converged ~= 1 && rec <= max_iterations
while rec <= 1
    
    dict_sol_list = cell(1,length(file_list));
    fprintf('Iteration: %i \n', rec)
    diff_theta = theta_chosen - theta;
    fprintf('This is the difference of theta: %i %i %i %i %i %i \n', diff_theta)
    his_diff_theta.("iteration_"+num2str(rec))=  diff_theta;
    fprintf('\n')
      
    for k = 1:1:length(file_list)
        file = file_list{1,k};
        curr_data = data_list{1,k};
        [data_s,lambda_sol] = optim_weights_ideal(theta,curr_data,rec,N,file);
        data_list{1,k}.lam_sol = lambda_sol;
        dict_sol_list{1,k} = data_s;
    end

    % Calculating averaged calculated solution
    av_features_calc = zeros(1,amount_features);
    for i = 1:1:length(file_list)
        av_features_calc = av_features_calc + dict_sol_list{1,i}.features;
        av_features_calc = av_features_calc ./ length(file_list);
        fprintf('av_features_data: %i %i %i %i %i %i',av_features_data)
        fprintf('\n')
        fprintf('av_features_calc: %i %i %i %i %i %i', av_features_calc)
        fprintf('\n')
    end

    % Normalization for plots
    f_calc_rel = av_features_calc./av_features_data;
    grad_curr = ones(1,amount_features) - f_calc_rel; % now is the normalized version

    fprintf('This is summed grad current: %i ', sum(abs(grad_curr)))
    fprintf('\n')
    fprintf("----------------------------------------------")
    fprintf('\n')
    fprintf('This is f_calc_rel: '); fprintf('\n');
    fprintf("iteration %i // %i %i %i %i %i %i",rec, f_calc_rel)
    fprintf('\n')
    his_f_calc_rel.("iteration_"+num2str(rec)) = f_calc_rel;
    his_grad_current.("iteration_"+num2str(rec)) = grad_curr;
    his_exception.("iteration_"+num2str(rec)) = exception;
    his_update.("iteration_"+num2str(rec)) = update;
    his_del_theta_prev.("iteration_"+num2str(rec)) = del_theta_prev;

    % Check if all features are converged or that the weights are not changing anymore
    f_calc_rel_check = [f_calc_rel(2),f_calc_rel(4),f_calc_rel(6)];
    update_check = [update(2), update(4), update(6)];
    if all(abs(f_calc_rel_check - 1) <= tol) || all(update_check <= 1e-4)
        converged = 1;
        if all(update <= 1e-4)
            fprintf('No change in theta detected anymore - learning terminated')
            fprintf('\n')
        end
    end
    fprintf('Converged vector: %i',converged)
    fprintf('\n')
    % Check what direction to go in optimization
    cas = ones(1,amount_features);
    for j = 1:1:amount_features
        fprintf('\n')
        fprintf('*********')
        fprintf('\n')
        if exception(j) == 1 || grad_curr(j) * grad_prev(j) == 0
            cas(j) = 3;
            fprintf("cas[%i] is: %i",j,cas(j))
        elseif grad_curr(j)*grad_prev(j)<0
            cas(j) = 2;
            fprintf("cas[%i] is: %i",j,cas(j))
        else
            fprintf("cas[%i] is: %i",j,cas(j))
            
        end
    end
    fprintf('\n')
    his_multi_grads.("iteration_"+num2str(rec)) = cas;
    
    solutions{1,rec} = dict_sol_list;
    if converged ~= 1
        [del_theta_prev, exception, theta, update] = RPROP(grad_curr,cas,amount_features,update,theta,del_theta_prev);
        grad_prev = grad_curr;
        rec = rec + 1;
        his_weights.("iteration_"+num2str(rec)) = theta;
            
    end

    % solutions.append([str(rec) + "//", dict_sol_list])
    
    
    fprintf('\n')
    fprintf("********************************************************************************************")
    fprintf('\n')
end

% Post - processing
theta_tracker.("iteration_"+num2str(rec)) = theta;
fprintf('\n')
fprintf("This is the history of his_multi_grads.")
fprintf('\n')
fprintf("------------------------------------------")
fprintf('\n')

fn = fieldnames(his_multi_grads);
for k=1:1:numel(fn)
fprintf('%s: %i %i %i %i %i %i ',fn{k},his_multi_grads.(fn{k}))
fprintf('\n')
end

fprintf("This is the history of current_grads.")
fprintf('\n')
fprintf("------------------------------------------")
fprintf('\n')
      
fn = fieldnames(his_grad_current);
for k=1:numel(fn)
fprintf('%s: %i %i %i %i %i %i ',fn{k},his_grad_current.(fn{k}))
fprintf('\n')
end
   
fprintf("This is the history of f_calc_rel.")
fprintf('\n')
fprintf("------------------------------------------")
fprintf('\n')
    
fn = fieldnames(his_f_calc_rel);
for k=1:numel(fn)
fprintf('%s: %i %i %i %i %i %i ',fn{k},his_f_calc_rel.(fn{k}))
fprintf('\n')
end

fprintf("This is the history of the used weights.")
fprintf('\n')
fprintf("------------------------------------------")
fprintf('\n')
    
fn = fieldnames(his_weights);
for k=1:numel(fn)
fprintf('%s: %i %i %i %i %i %i ',fn{k},his_weights.(fn{k}))
fprintf('\n')
end

fprintf("This is the history of the update of the weights.")
fprintf('\n')
fprintf("----------------------------------------------------")
fprintf('\n')
if rec ~= 1 
    fn = fieldnames(his_weights);
    for k=2:numel(fn)
        diff = his_weights.(fn{k}) - his_weights.(fn{k-1});
    fprintf('%s: %i %i %i %i %i %i ',fn{k},diff)
    fprintf('\n')
    end
else
    fprintf('\n')
    fprintf('Only one iteration - not able to calculate a difference of used weights.')
    fprintf('\n')
end
   
% Plotting end solution in comparinson
for k = 1:1: length(file_list)
    file = file_list{1,k};
    data_s = solutions{1, rec}{1, k};

        % X(t)/Y(t)
    figure(1)
    
    subplot(1, 2, 1)
    plot(data_s.time_s,data_s.x_s,'LineWidth',2)  
         
    hold on
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
    
    
    subplot(1, 2, 2)   
    plot(data_s.time_s,data_s.y_s,'LineWidth',2)  
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
    
    hold on
    
    saveas(gcf,".\written_data\1X_Y_N"+string(N)+"IT"+string(rec) + ".png")
    saveas(gcf,".\written_data\1X_Y_N"+string(N)+"IT"+string(rec) + ".fig")
    
    
    % Path
    figure(2)
    plot(data_s.x_s,data_s.y_s,'LineWidth',2)
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
            
    hold on
    saveas(gcf,".\written_data\2path_N"+string(N)+"IT"+string(rec) + ".png")
    saveas(gcf,".\written_data\2path_N"+string(N)+"IT"+string(rec) + ".fig")
    
    % VX(t)/VY(t)
    figure(3)
    
    subplot(1, 2, 1)
    plot(data_s.time_s,data_s.vx_s,'LineWidth',2)
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
           
    hold on
    
    
    subplot(1, 2, 2)    
    plot(data_s.time_s,data_s.vy_s,'LineWidth',2)  
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
        
    hold on
    
    saveas(gcf,".\written_data\3VX_VY_N"+string(N)+"IT"+string(rec) + ".png")
    saveas(gcf,".\written_data\3VX_VY_N"+string(N)+"IT"+string(rec) + ".fig")

    
    % AX(t)/AY(t)
    figure(4)
    
    subplot(1, 2, 1)
    plot(data_s.time_s,data_s.ax_tot_s,'LineWidth',2) 
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
          
    hold on

    
    
    subplot(1, 2, 2)    
    plot(data_s.time_s,data_s.ay_tot_s,'LineWidth',2) 
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
         
    hold on
    
    saveas(gcf,".\written_data\4AX_AY_N"+string(N)+"IT"+string(rec) + ".png")
    saveas(gcf,".\written_data\4AX_AY_N"+string(N)+"IT"+string(rec) + ".fig")
    
    % AtX(t)/AnX(t)
    figure(5)
    
    subplot(1, 2, 1)
    plot(data_s.time_s,data_s.atx_s,'LineWidth',2) 
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
          
    hold on
    
    
    subplot(1, 2, 2)   
    plot(data_s.time_s,data_s.anx_s,'LineWidth',2) 
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
         
    hold on
    
    saveas(gcf,".\written_data\5AtX_AnX_N"+string(N)+"IT"+string(rec) + ".png")
    saveas(gcf,".\written_data\5AtX_AnX_N"+string(N)+"IT"+string(rec) + ".fig")
    
    % AtY(t)/AnY(t)
    figure(6)
    
    subplot(1, 2, 1)
    plot(data_s.time_s,data_s.aty_s,'LineWidth',2) 
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
          
    hold on
    
    
    subplot(1, 2, 2)    
    plot(data_s.time_s,data_s.any_s,'LineWidth',2)  
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
        
    hold on
    
    saveas(gcf,".\written_data\6AtY_AnY_N"+string(N)+"IT"+string(rec) + ".png")
    saveas(gcf,".\written_data\6AtY_AnY_N"+string(N)+"IT"+string(rec) + ".fig")
    
      
    % JX(t)/JY(t)
    figure(7)
    
    subplot(1, 2, 1)
    plot(data_s.time_s,data_s.jx_s,'LineWidth',2) 
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
          
    hold on
    
    
    subplot(1, 2, 2)  
    plot(data_s.time_s,data_s.jy_s,'LineWidth',2)
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
          
    hold on
    
    saveas(gcf,".\written_data\7JX_JY_N"+string(N)+"IT"+string(rec) + ".png")
    saveas(gcf,".\written_data\7JX_JY_N"+string(N)+"IT"+string(rec) + ".fig")
    
    % yaw(t)/yaw_dot(t)
    figure(8)
    
    subplot(1, 2, 1)
    plot(data_s.time_s,data_s.psi_s*180/pi,'LineWidth',2) 
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
          
    hold on
    
    
    subplot(1, 2, 2) 
    plot(data_s.time_s,data_s.psi_dot_s*180/pi,'LineWidth',2)
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
          
    hold on
    
    saveas(gcf,".\written_data\8yaws_N"+string(N)+"IT"+string(rec) + ".png")
    saveas(gcf,".\written_data\8yaws_N"+string(N)+"IT"+string(rec) + ".fig")
    
    % % yaw_acc(t)
    figure(9)
    plot(data_s.time_s,data_s.psi_ddot_s*180/pi,'LineWidth',2)   
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
       
     hold on
    
    saveas(gcf,".\written_data\9yawacc_N"+string(N)+"IT"+string(rec) + ".png")
    saveas(gcf,".\written_data\9yawacc_N"+string(N)+"IT"+string(rec) + ".fig")
    % tr(t)/delta(t) DELTA = angle front wheel
    figure(10)
    
    subplot(1, 2, 1)
    plot(data_s.time_s,data_s.throttle_s,'LineWidth',2)
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
    hold on
    
    
    subplot(1, 2, 2)    
    plot(data_s.time_s,data_s.delta_s*180/pi,'LineWidth',2) 
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
    
    hold on
    
    saveas(gcf,".\written_data\10trdelta_N"+string(N)+"IT"+string(rec) + ".png")
    saveas(gcf,".\written_data\10trdelta_N"+string(N)+"IT"+string(rec) + ".fig")
 
    % tr_dot(t)/delta_dot(t) DELTA = angle front wheel
    figure(11)
    
    subplot(1, 2, 1)
    plot(data_s.time_s(1,1:end-1),data_s.throttle_dot_s,'LineWidth',2) 
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
      
    hold on
    
    
    subplot(1, 2, 2)    
    plot(data_s.time_s(1,1:end-1),data_s.delta_dot_s*180/pi,'LineWidth',2)
    legend("Data: " + file(6:11)+"-"+ file(13:17),"Init: " + file(6:11)+"-"+ file(13:17),"Learned: " + file(6:11)+"-"+ file(13:17));
    hold on
    
    saveas(gcf,".\written_data\11trdelta_dot_N"+string(N)+"IT"+string(rec) + ".png")
    saveas(gcf,".\written_data\11trdelta_dot_N"+string(N)+"IT"+string(rec) + ".fig")
    
   
   % Save values
   throttle_dot_save = [data_s.throttle_dot_s,data_s.throttle_dot_s(end)]';
   delta_dot_save = [data_s.delta_dot_s,data_s.delta_dot_s(end)]';
    M = [data_s.time_s',data_s.x_s',data_s.y_s',data_s.vx_s',data_s.vy_s',data_s.ax_tot_s',data_s.ay_tot_s',data_s.jx_s',data_s.jy_s',data_s.psi_s',data_s.psi_dot_s',data_s.psi_ddot_s',data_s.throttle_s',data_s.delta_s',throttle_dot_save,delta_dot_save,data_s.aty_s',data_s.any_s',data_s.atx_s',data_s.anx_s'];
    % Convert cell to a table and use first row as variable names
    T = array2table(M,'VariableNames',{'time','x','y','vx','vy','ax','ay','jx','jy','psi','psi_dot','psi_ddot','throttle','delta','throttle_dot','delta_dot','aty','a_ny','atx','anx'});
    % Write the table to a CSV file
    writetable(T,convertStringsToChars(".\written_data\DCA2_V"+convertCharsToStrings(file(7:11))+"_L"+ convertCharsToStrings(file(14:17))+".csv"))
    disp('CSV-file written')
    
end
    
fprintf('\n')
for k = 1:1: length(file_list)
    file = file_list{1,k};
    data_s = solutions{1, rec}{1, k};
    final_calculated = data_s.features;
    final_data = data_list{1,k}.features;
    fprintf("This is f_cal_rel of file %s: %i %i %i %i %i %i",file,final_calculated./final_data)
    fprintf("\n")
end
fprintf('\n')
fprintf('This is the theta_tracker: '); 
fprintf('\n')
fn = fieldnames(theta_tracker);
for k=1:numel(fn)
fprintf('%s: %i %i %i %i %i %i ',fn{k},theta_tracker.(fn{k}))
fprintf('\n')
end

post_processing_plots(his_f_calc_rel,his_weights,his_multi_grads,his_grad_current,his_diff_theta)

disp('Learning process finished')
disp('----------------------------')



