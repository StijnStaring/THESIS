import pylab as plt
from scipy import signal
from define_plots import define_plots
from derivative import derivative
from casadi import *

def optim_weights_ideal(theta,width_road,vx_start,data_cl,iteration,N,plotting,axcom1a,axcom1b,axcom2,axcom3a,axcom3b,axcom4a,axcom4b,axcom5a,axcom5b,axcom6a,axcom6b,axcom7a,axcom7b,axcom8a,axcom8b,axcom9,file):
    # theta = plt.array([4,5,6,1,2]) en met data guess berekende norm waarden en data guess zelf. (example lane change)
    theta = plt.squeeze(theta)
    norm0 = 0.007276047781441449
    norm1 = 2.6381715506137424
    norm2 = 11.283498669013454
    norm3 = 0.046662223759442054
    norm4 = 17.13698903738383
    # norm0 = 1.0
    # norm1 = 1.0
    # norm2 = 1.0
    # norm3 = 1.0
    # norm4 = 1.0
    # [data_cl,_,width_road, vx_start] = import_data2(file,0)

    # data_cl = dict_list[0]
    # Parameters of the non-linear bicycle model used to generate the data.
    # Remark !x and y are coordinates in global axis!
    M = 1430
    Izz = 1300
    a = 1.056
    b = 1.344
    Kyf = 41850.8527587
    Kyr = 51175.775017
    Cr0 = 0.6
    Cr2 = 0.1
    rw = 0.292
    Tmax = 584

    # Parameters of the optimization
    nx = 6  # amount of states
    nc = 2  # amount of controls

    x_start = 0
    y_start = 0
    vy_start = 0
    psi_start = 0
    psi_dot_start = 0

    # Resampling and guesses
    time_guess = data_cl['time_cl'][-1]
    x_guess = signal.resample(data_cl['x_cl'], N + 1).T
    y_guess = signal.resample(data_cl['y_cl'], N + 1).T
    vx_guess = signal.resample(data_cl['vx_cl'], N + 1).T
    vy_guess = signal.resample(data_cl['vy_cl'], N + 1).T
    psi_guess = signal.resample(data_cl['psi_cl'], N + 1).T
    psi_dot_guess = signal.resample(data_cl['psi_dot_cl'], N + 1).T
    throttle_guess = signal.resample(data_cl['throttle_cl'], N).T
    delta_guess = signal.resample(data_cl['delta_cl'], N).T  # Error made in data Siemens --> SWA not 40 degrees

    # Comfort cost function: ax**2+t1*ay**2+t2*jy**2+t3*(vx-vdes)**2+t4*(y-ydes)**2
    # theta = plt.array([4, 5, 6, 1, 2])

    # Equations of the vehicle model
    x = MX.sym('x')  # in global axis
    y = MX.sym('y')  # in global axis
    vx = MX.sym('vx')  # in local axis
    vy = MX.sym('vy')  # in local axis
    psi = MX.sym('psi')  # yaw angle
    psi_dot = MX.sym('psi_dot')  # rate of yaw angle

    # Controls
    throttle = MX.sym('throttle')
    delta = MX.sym('delta')

    x_dot_glob = vx * cos(psi) - vy * sin(psi)
    y_dot_glob = vx * sin(psi) + vy * cos(psi)
    slipangle_f = plt.arctan2(vy + psi_dot * a, vx) - delta
    slipangle_r = plt.arctan2(vy - psi_dot * b, vx)
    Fxf = throttle * Tmax / (2 * rw)
    Fxr = Fxf
    Fyf = -2 * Kyf * slipangle_f
    Fyr = -2 * Kyr * slipangle_r
    F_d = Cr0 + Cr2 * vx * vx
    ddx = (cos(delta) * Fxf - sin(delta) * Fyf + Fxr - F_d) / M + vy * psi_dot  # not total acceleration
    ddy = (sin(delta) * Fxf + cos(delta) * Fyf + Fyr) / M - vx * psi_dot  # not total acceleration
    ddpsi = (sin(delta) * Fxf * a + cos(delta) * Fyf * a - b * Fyr) / Izz

    # ----------------------------------
    #    continuous system dot(x)=f(x,u)
    # ----------------------------------
    # states: x, y, vx, vy, psi, psi_dot
    # controls: T, delta_dot
    rhs = vertcat(x_dot_glob, y_dot_glob, ddx, ddy, psi_dot, ddpsi)
    states = vertcat(x, y, vx, vy, psi, psi_dot)
    controls = vertcat(throttle, delta)
    f = Function('f', [states, controls], [rhs], ['states', 'controls'], ['rhs'])

    ##
    # -----------------------------------
    #    Discrete system x_next = F(x,u)
    # -----------------------------------
    dt = MX.sym('dt')
    k1 = f(states, controls)
    k2 = f(states + dt / 2 * k1, controls)
    k3 = f(states + dt / 2 * k2, controls)
    k4 = f(states + dt * k3, controls)
    states_next = states + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    F = Function('F', [states, controls, dt], [states_next], ['states', 'controls', 'dt'], ['states_next'])

    ##
    # -----------------------------------------------
    #    Optimal control problem, multiple shooting
    # -----------------------------------------------

    opti = casadi.Opti()
    X = opti.variable(nx, N + 1)
    T = opti.variable()  # Time [s] 

    # Aliases for states
    x = X[0, :]
    y = X[1, :]
    vx = X[2, :]
    vy = X[3, :]
    psi = X[4, :]
    psi_dot = X[5, :]

    # Decision variables for control vector
    U = opti.variable(nc, N)
    throttle = U[0, :]
    delta = U[1, :]

    # Gap-closing shooting constraints
    for k in range(N):
        opti.subject_to(X[:, k + 1] == F(X[:, k], U[k], T / N))

    # Path constraints
    opti.subject_to(opti.bounded(-1, throttle, 1))  # local axis [m/s^2]
    opti.subject_to(opti.bounded(-20*pi/180,delta,20*pi/180)) # Limit on steeringwheelangle (150°)
    opti.subject_to(opti.bounded(-width_road / 2, y, width_road * 3 / 2))  # Stay on road
    opti.subject_to(x[0, 1:] >= 0)  # vehicle has to drive forward
    opti.subject_to(T > 3)  # every lane change is taking at least 3 seconds
    opti.subject_to(opti.bounded(-1, vy, 1))

    # Initial constraints
    # states: x, y, vx, vy, psi, psi_dot
    opti.subject_to(X[:, 0] == vertcat(x_start, y_start, vx_start, vy_start, psi_start, psi_dot_start))
    opti.subject_to(y[-1] == width_road)  # should move 3,47 m lateral
    opti.subject_to(vy[-1] == 0)  # lane change is completed
    opti.subject_to(psi[-1] == 0)  # assuming straight road
    opti.subject_to(psi_dot[-1] == 0)

    #  Set guesses
    opti.set_initial(x, x_guess)
    opti.set_initial(y, y_guess)
    opti.set_initial(vx, vx_guess)
    opti.set_initial(vy, vy_guess)
    opti.set_initial(psi, psi_guess)
    opti.set_initial(psi_dot, psi_dot_guess)
    opti.set_initial(throttle, throttle_guess)
    opti.set_initial(delta, delta_guess)
    opti.set_initial(T, time_guess)

    ##
    # -----------------------------------------------
    #    objective
    # -----------------------------------------------
    # Objective: The total accelerations in the local axis are considered!
    time_list = []
    for i in range(N + 1):
        time_list.append(i * T / N)
    # time_vector = plt.array(time_list)

    # normal lateral accelleration
    anx_list = []
    for k in range(N + 1):
        anx_list.append(-vy[k] * psi_dot[k])

    any_list = []
    for k in range(N + 1):
        any_list.append(vx[k] * psi_dot[k])

    # tangential lateral accelleration
    aty_list = []
    for i in plt.arange(0, len(time_list), 1):
        if i == 0:
            aty_list.append((vy[i + 1] - vy[i]) / (T / N))
        elif i == len(time_list) - 1:
            aty_list.append((vy[i] - vy[i - 1]) / (T / N))
        else:
            aty_list.append((vy[i + 1] - vy[i - 1]) / (2 * (T / N)))

    # tangential longitudinal accelleration
    atx_list = []
    for i in plt.arange(0, len(time_list), 1):
        if i == 0:
            atx_list.append((vx[i + 1] - vx[i]) / (T / N))
        elif i == len(time_list) - 1:
            atx_list.append((vx[i] - vx[i - 1]) / (T / N))
        else:
            atx_list.append((vx[i + 1] - vx[i - 1]) / (2 * (T / N)))

    ax_tot = plt.array(atx_list) + plt.array(anx_list)
    ay_tot = plt.array(aty_list) + plt.array(any_list)

    # yaw_acceleration
    psi_ddot_list = []
    for i in plt.arange(0, len(time_list), 1):
        if i == 0:
            psi_ddot_list.append((psi_dot[i + 1] - psi_dot[i]) / (T / N))
        elif i == len(time_list) - 1:
            psi_ddot_list.append((psi_dot[i] - psi_dot[i - 1]) / (T / N))
        else:
            psi_ddot_list.append((psi_dot[i + 1] - psi_dot[i - 1]) / (2 * (T / N)))

    # calculation lateral jerk -> jerk is calculated from the total acceleration!
    # Implementation of second order scheme
    jy_list_t = []
    for i in plt.arange(0, len(time_list), 1):
        if i == 0:
            jy_list_t.append((aty_list[i + 1] - aty_list[i]) / (T / N))
        elif i == len(time_list) - 1:
            jy_list_t.append((aty_list[i] - aty_list[i - 1]) / (T / N))
        else:
            # jy_list.append((ay_tot[i + 1] - ay_tot[i - 1]) / (2 * (T/N)))
            jy_list_t.append((vy[i + 1] - 2 * vy[i] + vy[i - 1]) / ((T / N) ** 2))

    jy_list_n = []
    for k in range(N + 1):
        jy_list_n.append(psi_dot[k] * atx_list[k] + vx[k] * psi_ddot_list[k])
    jy_tot = plt.array(jy_list_t) + plt.array(jy_list_n)

    vx_des_list = []
    for k in range(N + 1):
        vx_des_list.append(vx[k] - vx_start)

    y_des_list = []
    for k in range(N + 1):
        y_des_list.append(y[k] - width_road)

    # Extra constraints on acceleration and jerk:
    opti.subject_to(aty_list[-1] == 0)  # to avoid shooting through
    opti.subject_to(jy_tot[-1] == 0)  # fully end of lane change --> no lateral acceleration in the next sample
    opti.subject_to(aty_list[0] == 0)  # start from the beginning of the lane change
    opti.subject_to(jy_tot[0] == 0)  # start from the beginning of the lane change
    for i in plt.arange(0,len(ay_tot),1):
        opti.subject_to(opti.bounded(-2, ay_tot[i], 2))


    # Comfort cost function: t0*axtot**2+t1*aytot**2+t2*jytot**2+t3*(vx-vdes)**2+t4*(y-ydes)**2

    # Crack-nicolson integration gave same results as simpson integration + more robuust solving
    # f0: longitudinal acceleration
    integrand = ax_tot ** 2
    f0_cal = 0
    for i in plt.arange(0, len(integrand) - 1, 1):
        f0_cal = f0_cal + 0.5 * (integrand[i] + integrand[i + 1]) * (T / N)

    # f1: lateral acceleration
    integrand = ay_tot ** 2
    f1_cal = 0
    for i in plt.arange(0, len(integrand) - 1, 1):
        f1_cal = f1_cal + 0.5 * (integrand[i] + integrand[i + 1]) * (T / N)

    # f2: lateral jerk
    integrand = jy_tot ** 2
    f2_cal = 0
    for i in plt.arange(0, len(integrand) - 1, 1):
        f2_cal = f2_cal + 0.5 * (integrand[i] + integrand[i + 1]) * (T / N)

    # f3: desired velocity
    integrand = plt.array(vx_des_list) ** 2
    f3_cal = 0
    for i in plt.arange(0, len(integrand) - 1, 1):
        f3_cal = f3_cal + 0.5 * (integrand[i] + integrand[i + 1]) * (T / N)

    # f4: desired lane change
    integrand = plt.array(y_des_list) ** 2
    f4_cal = 0
    for i in plt.arange(0, len(integrand) - 1, 1):
        f4_cal = f4_cal + 0.5 * (integrand[i] + integrand[i + 1]) * (T / N)


    # Comfort cost function: t0*ax**2+t1*ay**2+t2*jy**2+t3*(vx-vdes)**2+t4*(y-ydes)**2
    opti.minimize(theta[0] / norm0 * f0_cal + theta[1] / norm1 * f1_cal + theta[2] / norm2 * f2_cal + theta[3] / norm3 * f3_cal +theta[4] / norm4 * f4_cal)
    # opti.minimize(theta[0]/norm0*f0_cal+theta[1]/norm1*f1_cal+theta[2]/norm2*f2_cal+theta[3]/norm3*f3_cal+theta[4]/norm4*f4_cal)

    print('Absolute weights: ', theta / plt.array([norm0, norm1, norm2, norm3, norm4]))
    print('Relative weights: ', theta)

    # Implementation of the solver
    opti.solver('ipopt')
    sol = opti.solve()

    # ----------------------------------
    #    Post processing
    # ----------------------------------
    x_sol = sol.value(x)
    y_sol = sol.value(y)
    vx_sol = sol.value(vx)
    vy_sol = sol.value(vy)
    psi_sol = sol.value(psi)
    psi_dot_sol = sol.value(psi_dot)
    throttle_sol = sol.value(throttle)
    delta_sol = sol.value(delta)
    T_sol = sol.value(T)
    dt_sol = T_sol / (len(x_sol) - 1)

    anx_list = []
    for k in range(N + 1):
        anx_list.append(sol.value(-vy[k] * psi_dot[k]))
    anx_sol = plt.array(anx_list)

    vx_list = []
    for k in range(N + 1):
        vx_list.append(vx_sol[k])
    atx_sol = derivative(vx_list, dt_sol)
    ax_tot_sol = atx_sol + anx_sol

    any_list = []
    for k in range(N + 1):
        any_list.append(sol.value(vx[k] * psi_dot[k]))
    any_sol = plt.array(any_list)

    vy_list = []
    for k in range(N + 1):
        vy_list.append(vy_sol[k])
    aty_sol = derivative(vy_list, dt_sol)

    ay_tot_sol = aty_sol + any_sol

    psi_ddot_sol = derivative(sol.value(psi_dot), dt_sol)

    # Lateral jerk
    jyt_sol = []
    for i in plt.arange(0, len(vy_sol), 1):
        if i == 0:
            jyt_sol.append((aty_sol[i + 1] - aty_sol[i]) / dt_sol)
        elif i == len(vy_sol) - 1:
            jyt_sol.append((aty_sol[i] - aty_sol[i - 1]) / dt_sol)
        else:
            # jy_list.append((ay_tot[i + 1] - ay_tot[i - 1]) / (2 * dt_sol))
            jyt_sol.append((vy_sol[i + 1] - 2 * vy_sol[i] + vy_sol[i - 1]) / (dt_sol ** 2))

    jyn_sol = []
    for k in range(N + 1):
        jyn_sol.append(sol.value(psi_dot)[k] * atx_sol[k] + vx_sol[k] * psi_ddot_sol[k])

    jy_tot_sol = plt.array(jyt_sol) + plt.array(jyn_sol)

    # Longitudinal jerk
    jxt_sol = []
    for i in plt.arange(0, len(vx_sol), 1):
        if i == 0:
            jxt_sol.append((atx_sol[i + 1] - atx_sol[i]) / dt_sol)
        elif i == len(vx_sol) - 1:
            jxt_sol.append((atx_sol[i] - atx_sol[i - 1]) / dt_sol)
        else:
            # jx_list.append((ax_tot[i + 1] - ax_tot[i - 1]) / (2 * dt_sol))
            jxt_sol.append((vx_sol[i + 1] - 2 * vx_sol[i] + vx_sol[i - 1]) / (dt_sol ** 2))

    jxn_sol = []
    for k in range(N + 1):
        jxn_sol.append(-sol.value(psi_dot)[k] * aty_sol[k] - vy_sol[k] * psi_ddot_sol[k])

    jx_tot_sol = plt.array(jxt_sol) + plt.array(jxn_sol)

    if plotting == 1:
        define_plots("1",x_sol,y_sol,vx_sol,vy_sol,ax_tot_sol,ay_tot_sol,jx_tot_sol,jy_tot_sol,psi_sol,psi_dot_sol,psi_ddot_sol,throttle_sol,delta_sol,T_sol,aty_sol,any_sol,vx_start,width_road)


    f0 = sol.value(f0_cal)
    f1 = sol.value(f1_cal)
    f2 = sol.value(f2_cal)
    f3 = sol.value(f3_cal)
    f4 = sol.value(f4_cal)

    print("\n")
    print('Integrated feature values: iterations ',str(iteration))
    print('------------------------------')
    print('integrand = plt.squeeze(data_cl[ax_cl]**2)')
    print(f0)
    print('integrand = plt.squeeze(data_cl[ay_cl] ** 2)')
    print(f1)
    print('integrand = plt.squeeze(data_cl[jy_cl] ** 2)')
    print(f2)
    print('integrand = plt.squeeze((desired_speed - data_cl[vx_cl]) ** 2)')
    print(f3)
    print('integrand = plt.squeeze((delta_lane - data_cl[y_cl]) ** 2)')
    print(f4)
    print('dt of the optimization is: ', dt_sol)

    data_s = dict()
    data_s['x_s'] = sol.value(x)
    data_s['y_s'] = sol.value(y)
    data_s['vx_s'] = sol.value(vx)
    data_s['vy_s'] = sol.value(vy)
    data_s['psi_s'] = sol.value(psi)
    data_s['psi_dot_s'] = sol.value(psi_dot)
    data_s['throttle_s'] = sol.value(throttle)
    data_s['delta_s'] = sol.value(delta)
    data_s['T_s'] = sol.value(T)
    data_s['dt_s'] = data_s['T_s'] / (len(data_s['x_s']) - 1)
    data_s['ax_tot_s'] = ax_tot_sol
    data_s['ay_tot_s'] = ay_tot_sol
    data_s['aty_s'] = aty_sol
    data_s['any_s'] = any_sol
    data_s['jx_s'] = jx_tot_sol
    data_s['jy_s'] = jy_tot_sol
    data_s['psi_ddot_s'] = psi_ddot_sol

    features = plt.array([f0,f1,f2,f3,f4])
    features = features[:,plt.newaxis]
    data_s['features'] = features
    time_vector = plt.linspace(0, T_sol, len(x_sol))

    # Plotting
    # axcom1a.plot(time_vector, x_sol, '.-', linewidth=3.0,label="it: "+str(iteration))
    # axcom1b.plot(time_vector, y_sol, '.-',  linewidth=3.0,label="it: "+str(iteration))
    # axcom2.plot(x_sol, y_sol, '.-',  linewidth=3.0,label="it: "+str(iteration))
    # axcom3a.plot(time_vector, vx_sol, '.-',  linewidth=3.0,label="it: "+str(iteration))
    # axcom3b.plot(time_vector, vy_sol, '.-',  linewidth=3.0,label="it: "+str(iteration))
    # axcom4a.plot(time_vector, ax_tot_sol, '.-',  linewidth=3.0,label="it: "+str(iteration))
    # axcom4b.plot(time_vector, ay_tot_sol, '.-', linewidth=3.0,label="it: "+str(iteration))
    # axcom5a.plot(time_vector, jx_sol, '.-', linewidth=3.0,label="it: "+str(iteration))
    # axcom5b.plot(time_vector, jy_sol, '.-', linewidth=3.0,label="it: "+str(iteration))
    # axcom6a.plot(time_vector, psi_sol*180/plt.pi, '.-', linewidth=3.0,label="it: "+str(iteration))
    # axcom6b.plot(time_vector, psi_dot_sol*180/plt.pi, '.-', linewidth=3.0,label="it: "+str(iteration))
    # axcom7a.plot(time_vector[0:-1], throttle_sol, '.-', linewidth=3.0,label="it: "+str(iteration))
    # axcom7b.plot(time_vector[0:-1], delta_sol*180/plt.pi, '.-', linewidth=3.0,label="it: "+str(iteration))
    # axcom8a.plot(time_vector, aty_sol, '.-', linewidth=3.0,label="it: "+str(iteration))
    # axcom8b.plot(time_vector, any_sol, '.-', linewidth=3.0,label="it: "+str(iteration))
    if iteration == 1:
        axcom1a.plot(time_vector, x_sol, '.-', linewidth=3.0,label="init "+file[16:-4])
        axcom1b.plot(time_vector, y_sol, '.-',  linewidth=3.0,label="init "+file[16:-4])
        axcom2.plot(x_sol, y_sol, '.-',  linewidth=3.0,label="initial solution"+file[16:-4])
        axcom3a.plot(time_vector, vx_sol, '.-',  linewidth=3.0,label="init "+file[16:-4])
        axcom3b.plot(time_vector, vy_sol, '.-',  linewidth=3.0,label="init "+file[16:-4])
        axcom4a.plot(time_vector, ax_tot_sol, '.-',  linewidth=3.0,label="init "+file[16:-4])
        axcom4b.plot(time_vector, ay_tot_sol, '.-', linewidth=3.0,label="init "+file[16:-4])
        axcom5a.plot(time_vector, jx_tot_sol, '.-', linewidth=3.0,label="init "+file[16:-4])
        axcom5b.plot(time_vector, jy_tot_sol, '.-', linewidth=3.0,label="init "+file[16:-4])
        axcom6a.plot(time_vector, psi_sol*180/plt.pi, '.-', linewidth=3.0,label="init "+file[16:-4])
        axcom6b.plot(time_vector, psi_dot_sol*180/plt.pi, '.-', linewidth=3.0,label="init "+file[16:-4])
        axcom7a.plot(time_vector[0:-1], throttle_sol, '.-', linewidth=3.0,label="init "+file[16:-4])
        axcom7b.plot(time_vector[0:-1], delta_sol*180/plt.pi, '.-', linewidth=3.0,label="init "+file[16:-4])
        axcom8a.plot(time_vector, aty_sol, '.-', linewidth=3.0,label="init "+file[16:-4])
        axcom8b.plot(time_vector, any_sol, '.-', linewidth=3.0,label="init "+file[16:-4])
        axcom9.plot(time_vector, psi_ddot_sol*180/plt.pi, '.-', linewidth=3.0, label="init "+file[16:-4])

        axcom1a.legend()
        axcom1b.legend()
        axcom2.legend()
        axcom3a.legend()
        axcom3b.legend()
        axcom4a.legend()
        axcom4b.legend()
        axcom5a.legend()
        axcom5b.legend()
        axcom6a.legend()
        axcom6b.legend()
        axcom7a.legend()
        axcom7b.legend()
        axcom8a.legend()
        axcom8b.legend()
        axcom9.legend()

    return data_s, features


