function [controls] = PlanningFunction_lane_change(current_states)

    global  width_road x_sol_prev lam_prev theta planner_lane_change
    
    % Solving the optimization problem by calling the CasADi function,
    % previously implemented and loaded as a global variable   
    [controls,x_sol_prev,lam_prev] = planner_lane_change(current_states,theta,width_road,x_sol_prev,lam_prev);
    % controls = [throttle (-), delta (rad)]
    disp('simulation completed!')

    


    
    
    















end