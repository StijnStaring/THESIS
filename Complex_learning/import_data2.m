function [data_cl] = import_data2(file,ak)

% "time","x","y","vx","vy","ax","ay","jx","jy","psi","psi_dot","psi_ddot","throttle","delta","throttle_dot","delta_dot","aty","any","atx","anx"
% Read data
read = csvread(file,1);
% Struct data: 1xN
data_cl = struct();
data_cl.time_cl = read(:,1)';
N = size(data_cl.time_cl,2) -1;
data_cl.dt_cl = data_cl.time_cl(1,2) - data_cl.time_cl(1,1);
data_cl.x_cl = read(:,2)';
data_cl.y_cl = read(:,3)';
data_cl.vx_cl =read(:,4)';
data_cl.vy_cl = read(:,5)';
data_cl.ax_cl = read(:,6)'; % total acc
data_cl.ay_cl = read(:,7)'; % total acc
data_cl.jx_cl = read(:,8)';
data_cl.jy_cl = read(:,9)';
data_cl.psi_cl = read(:,10)';
data_cl.psi_dot_cl = read(:,11)';
data_cl.psi_ddot_cl = read(:,12)';
data_cl.throttle_cl = read(:,13)';
data_cl.delta_cl = read(:,14)'; % delta is the angle of the front wheel
throttle_dot_cl_temp = read(:,15)';
data_cl.throttle_dot_cl = throttle_dot_cl_temp(1,1:end-1);
delta_dot_cl_temp = read(:,16)';
data_cl.delta_dot_cl = delta_dot_cl_temp(1,1:end-1); % delta is the angle of the front wheel
data_cl.aty_cl = read(:,17)';
data_cl.any_cl= read(:,18)';
data_cl.atx_cl = read(:,19)';
data_cl.anx_cl = read(:,20)';
data_cl.width = data_cl.y_cl(1,end);
data_cl.vx_start = data_cl.vx_cl(1,1);


% % Calculation of features
% % f0: longitudinal acceleration
% integrand = data_cl.ax_cl .^  2;
% f0_cal = 0;
% for i = 1:1:N
%     f0_cal = f0_cal + 0.5 * (integrand(i) + integrand(i+1) )* data_cl.dt_cl;
% end
% 
% % f1: lateral acceleration
% integrand = data_cl.ay_cl .^  2;
% f1_cal = 0;
% for i = 1:1:N
%     f1_cal = f1_cal + 0.5 * (integrand(i) + integrand(i+1) )* data_cl.dt_cl;
% end
% % f2: longitudinal jerk
% integrand = data_cl.jx_cl .^  2;
% f2_cal = 0;
% for i = 1:1:N
%     f2_cal = f2_cal + 0.5 * (integrand(i) + integrand(i+1) )* data_cl.dt_cl;
% end
% % f3: lateral jerk
% integrand = data_cl.jy_cl .^  2;
% f3_cal = 0;
% for i = 1:1:N
%     f3_cal = f3_cal + 0.5 * (integrand(i) + integrand(i+1) )* data_cl.dt_cl;
% end
% % f4: desired velocity
% integrand = (data_cl.vx_start - data_cl.vx_cl) .^  2;
% f4_cal = 0;
% for i = 1:1:N
%     f4_cal = f4_cal + 0.5 * (integrand(i) + integrand(i+1) )* data_cl.dt_cl;
% end
% % f5: desired lane change
% integrand = (data_cl.width - data_cl.y_cl) .^  2;
% f5_cal = 0;
% for i = 1:1:N
%     f5_cal = f5_cal + 0.5 * (integrand(i) + integrand(i+1))* data_cl.dt_cl;
% end
% if ak == 1
%     fprintf("\n")
%     fprintf('Integrated feature values of the DATA ')
%     fprintf("\n")
%     fprintf('------------------------------')
%     fprintf("\n")
%     fprintf('integrand = plt.squeeze(data_cl[ax_cl].^ 2)')
%     fprintf("\n")
%     fprintf('%i ',f0_cal)
%     fprintf("\n")
%     fprintf('integrand = plt.squeeze(data_cl[ay_cl] .^  2)')
%     fprintf("\n")
%     fprintf('%i ',f1_cal)
%     fprintf('integrand = plt.squeeze(data_cl[jx_cl] .^  2)')
%     fprintf("\n")
%     fprintf('%i ',f2_cal)
%     fprintf("\n")
%     fprintf('integrand = plt.squeeze(data_cl[jy_cl] .^  2)')
%     fprintf("\n")
%     fprintf('%i ',f3_cal)
%     fprintf("\n")
%     fprintf('integrand = plt.squeeze((desired_speed - data_cl[vx_cl]) .^  2)')
%     fprintf("\n")
%     fprintf('%i ',f4_cal)
%     fprintf("\n")
%     fprintf('integrand = plt.squeeze((delta_lane - data_cl[y_cl]) .^  2)')
%     fprintf("\n")
%     fprintf('%i ',f5_cal)
%     fprintf("\n")
% end
% 
% data_cl.features = [f0_cal,f1_cal,f2_cal,f3_cal,f4_cal,f5_cal];


end 