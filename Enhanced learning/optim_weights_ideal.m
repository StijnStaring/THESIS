function [data_s, sol_lambda,sol_x] = optim_weights_ideal(theta,data_cl,iteration,N,file)
    import casadi.*

    width_road = data_cl.width;
    vx_start = data_cl.vx_start;

    norm0 = 0.007276047781441449;
    norm1 = 2.6381715506137424;
    norm2 = 0.007276047781441449;
    norm3 = 11.283498669013454;
    norm4 = 0.046662223759442054;
    norm5 = 17.13698903738383;

%      Parameters of the non-linear bicycle model used to generate the data. (from Siemens)
%      Remark: x and y are coordinates in global axis
    M = 1430;
    Izz = 1300;
    a = 1.056;
    b = 1.344;
    Kyf = 41850.8527587;
    Kyr = 51175.775017;
    Cr0 = 0.6;
    Cr2 = 0.1;
    rw = 0.292;
    Tmax = 584;
    
    % Parameters of the optimization
    nx = 10;  % amount of states
    nc = 2;   % amount of controls

    x_start = 0;
    y_start = 0;
    vy_start = 0;
    psi_start = 0;
    psi_dot_start = 0;

    T_limit = 25;
    fprintf("\n")
    fprintf("start opti weights simulatie! \n")
    fprintf('-------------------------------------------------- \n')
    fprintf('N:  %i, and T_limit: %i  ', N,T_limit)
    fprintf('\n')
    fprintf("The name of the file: %s", file)
    fprintf('\n')
    
        % Resampling mostly not needed with this data file
    N_old = size(data_cl.x_cl,2) - 1;
    if N_old ~= N
        error("Error: N_old is not equal to N, make use of resampling of the observation.")
    end
    time_guess = data_cl.time_cl(1, end);
    % x_guess = signal.resample_poly(data_cl('x_cl'), N, N_old, axis=1, padtype='line')
    % x_guess = x_guess(None, 0, 0:N + 1)
    x_guess = data_cl.x_cl;
    % y_guess = signal.resample_poly(data_cl('y_cl'), N, N_old, axis=1, padtype='maximum')
    % y_guess = y_guess(None, 0, 0:N + 1)
    y_guess = data_cl.y_cl;
    % vx_guess = signal.resample_poly(data_cl('vx_cl'), N, N_old,axis= 1 ,padtype='line')
    % vx_guess = vx_guess(None, 0, 0:N + 1)
    % vx_guess = signal.resample(data_cl('vx_cl'), N + 1, axis=1)
    vx_guess = data_cl.vx_cl;
    % vy_guess = signal.resample(data_cl('vy_cl'), N + 1, axis=1)
    vy_guess = data_cl.vy_cl;
    % psi_guess = signal.resample(data_cl('psi_cl'), N + 1, axis=1)
    psi_guess = data_cl.psi_cl;
    % psi_dot_guess = signal.resample(data_cl('psi_dot_cl'), N + 1, axis=1)
    psi_dot_guess = data_cl.psi_dot_cl;
    % throttle_guess = signal.resample(data_cl('throttle_cl'), N + 1,axis=1)  % throttle and delta use to be inputs
    throttle_guess = data_cl.throttle_cl;
    % delta_guess = signal.resample(data_cl('delta_cl'), N + 1, axis=1)
    delta_guess = data_cl.delta_cl;
    % throttle_dot_guess = signal.resample(data_cl('throttle_dot_cl'), N, axis=1)
    throttle_dot_guess = data_cl.throttle_dot_cl;
    % delta_dot_guess = signal.resample(data_cl('delta_dot_cl'), N, axis=1)
    delta_dot_guess = data_cl.delta_dot_cl;
    % ax_total_guess = signal.resample(data_cl('ax_cl'),N+1,axis= 1)
    ax_total_guess = data_cl.ax_cl;
    % ay_total_guess = signal.resample(data_cl('ay_cl'), N + 1, axis=1)
    ay_total_guess = data_cl.ay_cl;
    
    fprintf('This is the shape of your guesses: %i %i ', size(data_cl.x_cl))
    fprintf('\n')

    % %%%%%%%%%%%%%%%
    % Plot guesses
    %%%%%%%%%%%%%%%
%     fprintf('This is the size of x_guess: %i %i',size(x_guess))
%     figure('name','x')
%     plot(linspace(0,time_guess,N+1),squeeze(x_guess))
%     figure('name','y')
%     plot(linspace(0,time_guess,N+1),squeeze(y_guess))
%     figure('name','vx')
%     plot(linspace(0,time_guess,N+1),squeeze(vx_guess))
%     figure('name','vy')
%     plot(linspace(0,time_guess,N+1),squeeze(vy_guess))
%     figure('name','psi')
%     plot(linspace(0,time_guess,N+1),squeeze(psi_guess))
%     figure('name','psi_dot')
%     plot(linspace(0,time_guess,N+1),squeeze(psi_dot_guess))
%     figure('name','throttle')
%     plot(linspace(0, time_guess, N + 1), squeeze(throttle_guess))
%     figure('name','delta')
%     plot(linspace(0, time_guess, N + 1), squeeze(delta_guess))
%     figure('name','throttle_dot')
%     plot(linspace(0, time_guess, N ), squeeze(throttle_dot_guess))
%     figure('name','delta_dot')
%     plot(linspace(0, time_guess, N), squeeze(delta_dot_guess))
%     figure('name','ax')
%     plot(linspace(0,time_guess,N+1),squeeze(ax_total_guess))
%     figure('name','ay')
%     plot(linspace(0,time_guess,N+1),squeeze(ay_total_guess))

    fprintf("\n")
    fprintf("vx_start: %i and width_road: %i", vx_start,width_road)
    fprintf("\n")
    
    % Equations of the vehicle model
    x = SX.sym('x');  % in global axis
    y = SX.sym('y');  % in global axis
    vx = SX.sym('vx');  % in local axis
    vy = SX.sym('vy') ; % in local axis
    psi = SX.sym('psi') ; % yaw angle
    psi_dot = SX.sym('psi_dot');  % rate of yaw angle
    throttle = SX.sym('throttle');
    delta = SX.sym('delta');  % this is the angle of the front tire --> Gsteer factor needed if want steerwheelangle (Amesim model)
    ax_total = SX.sym('ax_tot');
    ay_total = SX.sym('ay_tot');

    % Controls
    throttle_dot = SX.sym('throttle_dot');
    delta_dot = SX.sym('delta_dot');

    % Other
    vx_des = vx_start;
    y_change = width_road;

    x_dot_glob = vx * cos(psi) - vy * sin(psi);
    y_dot_glob = vx * sin(psi) + vy * cos(psi);
    slipangle_f = atan2(vy + psi_dot * a, vx) - delta;
    slipangle_r = atan2(vy - psi_dot * b, vx);
    Fxf = throttle * Tmax / (2 * rw);
    c = Tmax / (2 * rw);
    Fxr = Fxf;
    Fyf = -2 * Kyf * slipangle_f;
    Fyr = -2 * Kyr * slipangle_r;
    F_d = Cr0 + Cr2 * vx * vx;
    atx = (cos(delta) * Fxf - sin(delta) * Fyf + Fxr - F_d) / M + vy * psi_dot;  % tangential acceleration
    aty = (sin(delta) * Fxf + cos(delta) * Fyf + Fyr) / M - vx * psi_dot;
    an_y = vx * psi_dot;  % normal acceleration
    anx = -vy * psi_dot;
    psi_ddot = (sin(delta) * Fxf * a + cos(delta) * Fyf * a - b * Fyr) / Izz;

    % ax_total = (cos(delta)*Fxf-sin(delta)*Fyf+Fxr-F_d)/M
    % ay_total = (sin(delta) * Fxf + cos(delta) * Fyf + Fyr) / M

    % j_total = derivative(ax_total(t),t) --> see photos of my notes (c)
    jx_total = (c * throttle_dot - 2 * Cr2 * vx * atx + 2 * Kyf * sin(delta) * (((a * psi_ddot + aty) / vx - ((vy + a * psi_dot) * atx) / vx ^ 2) / ((vy + a * psi_dot) ^ 2 / vx ^ 2 + 1) - delta_dot) + c * cos(delta) * throttle_dot - 2 * Kyf * cos(delta) * (delta - atan2((vy + a * psi_dot), vx)) * delta_dot - c * sin(delta) * throttle * delta_dot) / M;
    jy_total = ((2 * Kyr * ((b * psi_ddot - aty) / vx + ((vy - b * psi_dot) * atx) / vx ^ 2)) / ((vy - b * psi_dot) ^ 2 / vx ^ 2 + 1) - 2 * Kyf * cos(delta) * (((a * psi_ddot + aty) / vx - ((vy + a * psi_dot) * atx) / vx ^ 2) / ((vy + a * psi_dot) ^ 2 / vx ^ 2 + 1) - delta_dot) + c * sin(delta) * throttle_dot - 2 * Kyf * sin(delta) * (delta - atan2((vy + a * psi_dot), vx)) * delta_dot + c * cos(delta) * throttle * delta_dot) / M;

    ax_total_int = ax_total ^ 2;
    ay_total_int = ay_total ^ 2;
    jx_total_int = jx_total ^ 2;
    jy_total_int = jy_total ^ 2;
    vx_diff_int = vx ^ 2 - 2 * vx * vx_des + vx_des ^ 2 ; % (vx- vx_des)^2 --> vx_des is the start vx at beginning lane change
    y_diff_int = y ^ 2 - 2 * y * y_change + y_change ^ 2 ; % (y-y_des)^2 --> y_des is 3.47, distance to be travelled to change lane

    % ----------------------------------
    %    continuous system dot(x)=f(x,u)
    % ----------------------------------
    % states: x, y, vx, vy, psi, psi_dot, throttle, delta
    % controls: throttle_dot, delta_dot
    rhs = [x_dot_glob; y_dot_glob; atx; aty; psi_dot; psi_ddot; throttle_dot; delta_dot; jx_total; jy_total];
    states = [x; y; vx; vy; psi; psi_dot; throttle; delta; ax_total; ay_total];
    controls = [throttle_dot; delta_dot];
    f = Function('f', {states, controls}, {rhs}, {'states', 'controls'}, {'rhs'});

    % Other functions:
    stock = Function('stock', {states}, {ax_total, ay_total, psi_ddot, atx, anx, aty, an_y}, {'states'},{'ax_total', 'ay_total', 'psi_ddot', 'atx', 'anx', 'aty', 'an_y'});
    stock2 = Function('stock2', {states, controls}, {jx_total, jy_total}, {'states', 'controls'},{'jx_total', 'jy_total'});

    AXT_int = Function('AXT_int', {states}, {ax_total_int}, {'states'}, {'ax_total_int'});
    AYT_int = Function('AYT_int', {states}, {ay_total_int}, {'states'}, {'ay_total_int'});
    JXT_int = Function('JXT_int', {states, controls}, {jx_total_int}, {'states', 'controls'},{'jx_total_int'});
    JYT_int = Function('JYT_int', {states, controls}, {jy_total_int}, {'states', 'controls'},{'jy_total_int'});
    JXT = Function('JXT', {states, controls}, {jx_total}, {'states', 'controls'}, {'jx_total'});
    JYT = Function('JYT', {states, controls}, {jy_total}, {'states', 'controls'}, {'jy_total'});
    VXD_int = Function('VXD_int', {states}, {vx_diff_int}, {'states'}, {'vx_diff_int'});
    YD_int = Function('YD_int', {states}, {y_diff_int}, {'states'}, {'y_diff_int'});

    %%
    % -----------------------------------
    %    Discrete system x_next = F(x,u)
    % -----------------------------------
    dt = SX.sym('dt');
    k1 = f(states, controls);
    k2 = f(states + dt / 2 * k1, controls);
    k3 = f(states + dt / 2 * k2, controls);
    k4 = f(states + dt * k3, controls);
    states_next = states + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    F = Function('F', {states, controls, dt}, {states_next}, {'states', 'controls', 'dt'}, {'states_next'});

    % -----------------------------------
    % Objective of path planning = th0/n0*int(ax^2,dt)+th1/n1*int(ay^2,dt)+th2/n2*int(jx^2,dt)+th3/n3*int(jy^2,dt)+th4/n4*int(vx_diff^2,dt)+th5/n5*int(y_diff^2,dt)
    % integration needed over the entire time horizon
    % th_ = theta/weigth, n_ = norm factor
    % -----------------------------------

    %%
    % -----------------------------------------------
    %    Optimal control problem, multiple shooting
    % -----------------------------------------------

    opti = casadi.Opti();
    X = opti.variable(nx, N + 1);
    U = opti.variable(nc, N);
    T = opti.variable() ; % Time [s]

    % Aliases for states
    x = X(1, :);
    y = X(2, :);
    vx = X(3, :);
    vy = X(4, :);
    psi = X(5, :) ; % yaw
    psi_dot = X(6, :);  % yaw rate
    throttle = X(7, :) ; % gas padel
    delta = X(8, :) ; % angle of front wheel
    ax_total = X(9, :);
    ay_total = X(10, :);

    % Decision variables for control vector
    throttle_dot = U(1, :);
    delta_dot = U(2, :);

    % Gap-closing shooting constraints
    for k  = 1:1:N
        opti.subject_to(X(:, k + 1) == F(X(:, k), U(:, k), T / N))
    end
    % Path constraints
    opti.subject_to(-1<= throttle <= 1)
    % opti.subject_to(opti.bounded(-2.618,delta,2.618)) % Limit on steeringwheelangle (150°)
    opti.subject_to(-width_road / 2<= y <=width_road * 3 / 2)  % Stay on road--> start middle of right lane of a two lane road
    opti.subject_to(x(0, 2:end) >= 0)  % vehicle has to drive forward

    % Initial constraints
    % states: x, y, vx, vy, psi, psi_dot, throttle, delta, ax, ay
    opti.subject_to(X(1:6, 1) == [x_start; y_start; vx_start; vy_start; psi_start; psi_dot_start]);
    % jx_start = JX_TOT(X(:,0),U(:,0))
    % jy_start = JY_TOT(X(:, 0), U(:, 0))
    % opti.subject_to(jx_start == 0)
    % opti.subject_to(jy_start == 0)
    opti.subject_to(X(7, 1) == (Cr0 + Cr2 * vx_start ^ 2) / (2 * c))  % no longitudinal acc when drag force taken into account
    opti.subject_to(X(8, 1) == 0)  % driving straigth
    % T_limit = 10
    opti.subject_to(T <= T_limit)

    % Terminal constraints
    opti.subject_to(y(end) == width_road);  % should move 3,47 m
    opti.subject_to(vy(end) == 0);  % lane change is completed
    opti.subject_to(psi(end) == 0);  % assuming straight road
    opti.subject_to(psi_dot(end) == 0);  % Fy is zero
    opti.subject_to(delta(end) == 0);  % Fy is zero
    Gsteer = 16.96;
    limit = 150 / Gsteer ; % max steerwheelangle / constant factor to front wheel (paper Son)
    opti.subject_to(-limit * pi / 180 <= delta <= limit * pi / 180);

    %  Set guesses
    opti.set_initial(x, x_guess);
    opti.set_initial(y, y_guess);
    opti.set_initial(vx, vx_guess);
    opti.set_initial(vy, vy_guess);
    opti.set_initial(psi, psi_guess);
    opti.set_initial(psi_dot, psi_dot_guess);
    opti.set_initial(throttle, throttle_guess);
    opti.set_initial(delta, delta_guess);
    opti.set_initial(throttle_dot, throttle_dot_guess);
    opti.set_initial(delta_dot, delta_dot_guess);
    opti.set_initial(T, time_guess);
    opti.set_initial(ax_total, ax_total_guess);
    opti.set_initial(ay_total, ay_total_guess);
    
    if iteration ~= 1
        opti.set_initial(opti.lam_g, data_cl.lam_sol) % gives the solution of the previous iteration as starting point  
    end
    %%
    % -----------------------------------------------
    %    objective
    % -----------------------------------------------

    % Comfort cost function: t0/n0*axtot^2+t1/n1*aytot^2+t2/n2*jxtot^2+t3/n3*jytot^2+t4/n4*(vx-vdes)^2+t5/n5*(y-ydes)^2
    % Jerk is evaluated at point of states and when just the new control is applied --> beginning of next interval.
    f0_cal = 0;
    f1_cal = 0;
    f2_cal = 0;
    f3_cal = 0;
    f4_cal = 0;
    f5_cal = 0;
    
    for i = 1:1:N
        if i == N
            f0_cal = f0_cal + 0.5 * (AXT_int(X(:, i)) + AXT_int(X(:, i + 1))) * (T / N);
            f1_cal = f1_cal + 0.5 * (AYT_int(X(:, i)) + AYT_int(X(:, i + 1))) * (T / N);
            f2_cal = f2_cal + 0.5 * (JXT_int(X(:, i), U(:, i)) + JXT_int(X(:, i + 1), U(:, i))) * (T / N);
            f3_cal = f3_cal + 0.5 * (JYT_int(X(:, i), U(:, i)) + JYT_int(X(:, i + 1), U(:, i))) * (T / N);
            f4_cal = f4_cal + 0.5 * (VXD_int(X(:, i)) + VXD_int(X(:, i + 1))) * (T / N);
            f5_cal = f5_cal + 0.5 * (YD_int(X(:, i)) + YD_int(X(:, i + 1))) * (T / N);
        else
            f0_cal = f0_cal + 0.5 * (AXT_int(X(:, i)) + AXT_int(X(:, i + 1))) * (T / N);
            f1_cal = f1_cal + 0.5 * (AYT_int(X(:, i)) + AYT_int(X(:, i + 1))) * (T / N);
            f2_cal = f2_cal + 0.5 * (JXT_int(X(:, i), U(:, i)) + JXT_int(X(:, i + 1), U(:, i + 1))) * (T / N);
            f3_cal = f3_cal + 0.5 * (JYT_int(X(:, i), U(:, i)) + JYT_int(X(:, i + 1), U(:, i + 1))) * (T / N);
            f4_cal = f4_cal + 0.5 * (VXD_int(X(:, i)) + VXD_int(X(:, i + 1))) * (T / N);
            f5_cal = f5_cal + 0.5 * (YD_int(X(:, i)) + YD_int(X(:, i + 1))) * (T / N);
        end
    end
    % Comfort cost function: t0*ax^2+t1*ay^2+t2*jy^2+t3*(vx-vdes)^2+t4*(y-ydes)^2
    opti.minimize(theta(1) / norm0 * f0_cal + theta(2) / norm1 * f1_cal + theta(3) / norm2 * f2_cal + theta(4) / norm3 * f3_cal + theta(5) / norm4 * f4_cal + theta(6) / norm5 * f5_cal);
    % opti.minimize(theta(0) / norm0 * f0_cal + theta(1) / norm1 * f1_cal + theta(4) / norm4 * f4_cal +theta(5) / norm5 * f5_cal)

    fprintf('Absolute weights: %i %i %i %i %i %i', (theta ./ [norm0, norm1, norm2, norm3, norm4, norm5]))
    fprintf("\n")
    fprintf('Relative weights: %i %i %i %i %i %i', theta)
    fprintf("\n")

    % Implementation of the solver

    % options = dict()
    % options.expand = true
    opti.solver('ipopt');
    sol = opti.solve();

    % ----------------------------------
    %    Post processing
    % ----------------------------------
%     x_sol = sol.value(x);
%     y_sol = sol.value(y);
%     vx_sol = sol.value(vx);
%     vy_sol = sol.value(vy);
%     psi_sol = sol.value(psi);
%     psi_dot_sol = sol.value(psi_dot);
%     throttle_sol = sol.value(throttle);
%     delta_sol = sol.value(delta);
%     throttle_dot_sol = sol.value(throttle_dot);
%     delta_dot_sol = sol.value(delta_dot);
    T_sol = sol.value(T);
    dt_sol = T_sol / N;
    ax_tot_sol = zeros(1,N + 1);
    ay_tot_sol = zeros(1,N + 1);
    jx_tot_sol = zeros(1,N + 1);
    jy_tot_sol = zeros(1,N + 1);
    psi_ddot_sol = zeros(1,N + 1);
    aty_sol = zeros(1,N + 1);
    any_sol = zeros(1,N + 1);
    atx_sol = zeros(1,N + 1);
    anx_sol = zeros(1,N + 1);

    for i = 1:1:N+1
        [res1,res2,res3,res4,res5,res6,res7] = stock(sol.value(X(:, i)));
        ax_tot_sol(i) = full(res1);
        ay_tot_sol(i) = full(res2);
        psi_ddot_sol(i) = full(res3);
        atx_sol(i) = full(res4);
        anx_sol(i) = full(res5);
        aty_sol(i) = full(res6);
        any_sol(i) = full(res7);
    end

    for i = 1:1:N
        [res21,res22] = stock2(sol.value(X(:, i)), sol.value(U(:, i)));
        jx_tot_sol(i) = full(res21);
        jy_tot_sol(i) = full(res22);
    end
    jx_tot_sol(N+1) = full(JXT(sol.value(X(:, N+1)),sol.value(U(:, N ))));  % uses previous control to approximate the jerk
    jy_tot_sol(N+1) = full(JYT(sol.value(X(:, N+1)), sol.value(U(:, N))));

    if (T_limit - dt_sol) < T_sol
        fprintf("-------------------------------------")
        fprintf("\n")
        fprintf("Warning: Time constraint is binding!")
        fprintf("\n")
        fprintf("-------------------------------------")
        fprintf("\n")
    end
    

    data_s = struct();
    data_s.x_cl = sol.value(x);
    data_s.time_cl = linspace(0, T_sol, size(data_s.x_cl,2));
    data_s.y_cl = sol.value(y);
    data_s.vx_cl = sol.value(vx);
    data_s.vy_cl = sol.value(vy);
    data_s.psi_cl = sol.value(psi);
    data_s.psi_dot_cl = sol.value(psi_dot);
    data_s.psi_ddot_cl = psi_ddot_sol;
    data_s.throttle_cl = sol.value(throttle);
    data_s.delta_cl = sol.value(delta);
    data_s.throttle_dot_cl = sol.value(throttle_dot);
    data_s.delta_dot_cl = sol.value(delta_dot);
    data_s.T_cl = sol.value(T);
    data_s.dt_cl = data_s.T_cl / N;
    data_s.ax_cl = ax_tot_sol;
    data_s.ay_cl = ay_tot_sol;
    data_s.aty_cl = aty_sol;
    data_s.any_cl = any_sol;
    data_s.atx_cl = atx_sol;
    data_s.anx_cl = anx_sol;
    data_s.jx_cl = jx_tot_sol;
    data_s.jy_cl = jy_tot_sol;
    
    sol_lambda = sol.value(opti.lam_g);
    sol_x = sol.value(opti.x);
    
    fprintf('Simulation opti weights completed!')
    fprintf("\n")
end
