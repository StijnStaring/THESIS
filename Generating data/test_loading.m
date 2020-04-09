clc
clearvars
close all

import casadi.*
load('toto');
x_sol_prev = toto{1, 1}.x_sol_prev;
lam_prev = toto{1, 1}.lam_prev;


planner = Function.load('planner.casadi');
DM.set_precision(15);
[controls,x_sol_prev,lam_prev] = planner([0,0,80/3.6,0,0,0],[4,5,6,1,2],3.46990715,x_sol_prev,lam_prev);


% inputs = [X0,theta,width_road,opti.x,opti.lam_g]
% outputs = [U[:,0],opti.x,opti.lam_g]
% planner = opti.to_function('planner',inputs,outputs)
% print('');print('Simulation completed!')
% planner.save('planner.casadi')


disp('simulation completed!')

