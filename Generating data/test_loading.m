clc
clearvars
close all

import casadi.*
load('previous_solution');
x_sol_prev = previous_solution{1, 1}.x_sol_prev;
lam_prev = previous_solution{1, 1}.lam_prev;


planner_lane_change = Function.load('planner_lane_change.casadi');
DM.set_precision(15);

% inputs = [X0,theta,width_road,opti.x,opti.lam_g]
% outputs = [U[:,0],opti.x,opti.lam_g]
[controls,x_sol_prev,lam_prev] = planner_lane_change([5,1,80/3.6,0.2,2*pi/180,1*pi/180],[4,5,6,1,2],3.46990715,x_sol_prev,lam_prev);
disp('simulation completed!')

