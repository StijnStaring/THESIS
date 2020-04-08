function [throttle, delta, x_sol_prev, lam_prev] = PlanningFunction_lane_change(current_states,x_sol_prev,lam_prev)

    global Planner

    % Solving the optimization problem by calling the CasADi function,
    % previously implemented and loaded as a global variable
    sol = Planner(current_states,x_sol_prev,lam_prev);
    [throttle, delta, x_sol_prev, lam_prev] = full(sol);


    
    
    















end