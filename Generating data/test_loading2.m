clc
close all
clearvars

load('previous_solution');
global x_sol_prev lam_prev
x_sol_prev = previous_solution{1, 1}.x_sol_prev;
lam_prev = previous_solution{1, 1}.lam_prev;

[controls,x_sol_prev,lam_prev] = test_loading();

