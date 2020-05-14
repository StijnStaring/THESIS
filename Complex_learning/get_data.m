function [data] = get_data(file)

% Read data
read = csvread(file,1);
% Struct data
data = struct;
data.time_d = read(:,1);
data.T_pl = time_d(2,1) - time_d(1,1);
data.x_d = read(:,2);
data.y_d = read(:,3);
data.vx_d =read(:,4);
data.vy_d = read(:,5);
data.ax_d = read(:,6); % total acc
data.ay_d = read(:,7); % total acc
data.jx_d = read(:,8);
data.jy_d = read(:,9);
data.psi_d = read(:,10);
data.psi_dot_d = read(:,11);
data.psi_ddot_d = read(:,12);
data.throttle_d = read(:,13);
data.delta_d = 1/16.96*read(:,14);
data.throttle_dot_d = read(:,15);
data.delta_dot_d = 1/16.96*read(:,16);
data.aty_d = read(:,17);
data.any_d = read(:,18);
data.atx_d = read(:,19);
data.anx_d = read(:,20);





end