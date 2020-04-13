function [result] = PlanningFunction_lane_change(current_states)

    global  width_road x_sol_prev lam_prev theta planner_lane_change
    disp('This is the current state')
    current_states
    
%   current_states(3) = 80/3.6;
    % Solving the optimization problem by calling the CasADi function,
    % previously implemented and loaded as a global variable   
    [controls,x_sol_prev,lam_prev] = planner_lane_change(current_states,theta,width_road,x_sol_prev,lam_prev);
%     [controls,~,~] = planner_lane_change(current_states,theta,width_road,x_sol_prev,lam_prev);    
% controls = [throttle (-), delta (rad)]
%     result = [0.01,0.7]';
    result = full(controls);
    disp('simulation completed!')

    


    
    
    















end