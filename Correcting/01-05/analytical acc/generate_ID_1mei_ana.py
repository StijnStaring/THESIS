"""
This function is generating data using the same vehicle model as in the learning algorithm and uses the comfortcostfunction
as objective when pathplanning.
Feedforward control of the vehicle. One optimization is solved over the whole control horizon.
Linear tyre model -> valid if slipangle < 5 degrees and lat_acc < 4 m/s^2
Producing different lane changes: different longitudinal start velocities and different time to finish the lane change
A real driver is taking a limit of the time of the lane change into account
time_range: free time range
speed_range: [80-90] km/hour
:return: csv-datafiles
"""
import glob
# import csv
import pylab as plt
from scipy import signal
from import_ID_1mei_ana import import_ID_1mei
from define_plots import define_plots

from casadi import *

norm0 = 0.007276047781441449
norm1 = 2.6381715506137424
norm2 = 0.007276047781441449
norm3 = 11.283498669013454
norm4 = 0.046662223759442054
norm5 = 17.13698903738383

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
pi = 3.14159265359

# Parameters of the optimization
theta = plt.array([4,5,1,6,1,2])
nx = 8 # amount of states
nc = 2 # amount of controls
N = 500

x_start = 0
y_start = 0
vy_start = 0
psi_start = 0
psi_dot_start = 0

# ----------------------------------
#    for loop over optimization
# ----------------------------------
file_list = glob.glob("reading_dataset/*.csv")
for file in file_list:
    print('\n')
    print("The name of the file: ", file)
    data_cl = import_ID_1mei(file)
    width_road = data_cl['width']
    vx_start = data_cl['vx_start']

    # Resampling and guesses !! Resampling not good for not periodic signal!!
    # N_old = len(data_cl['x_cl']) - 1
    time_guess = data_cl['time_cl'][0,-1]
    # x_guess = signal.resample_poly(data_cl['x_cl'], N, N_old, ,axis= 1,padtype='line')
    x_guess = data_cl['x_cl']
    # y_guess = signal.resample_poly(data_cl['y_cl'], N, N_old, ,axis= 1, padtype='maximum')
    y_guess = data_cl['y_cl']
    # vx_guess = signal.resample_poly(data_cl['vx_cl'], N, N_old,axis= 1 padtype='line')
    vx_guess = data_cl['vx_cl']
    # vy_guess = signal.resample(data_cl['vy_cl'], N + 1,axis= 1)
    vy_guess = data_cl['vy_cl']
    # psi_guess = signal.resample(data_cl['psi_cl'], N + 1,axis= 1)
    psi_guess = data_cl['psi_cl']
    # psi_dot_guess = signal.resample(data_cl['psi_dot_cl'], N + 1,axis= 1)
    psi_dot_guess = data_cl['psi_dot_cl']
    throttle_guess = signal.resample(data_cl['throttle_cl'], N+1,axis= 1)
    delta_guess = signal.resample(data_cl['delta_cl'], N+1,axis= 1)
    # throttle_dot_guess = signal.resample(data_cl['throttle_dot_cl'], N+1,axis= 1)
    throttle_dot_guess = data_cl['throttle_dot_cl']
    # delta_dot_guess = signal.resample(data_cl['delta_dot_cl'], N+1,axis= 1)
    delta_dot_guess = data_cl['delta_dot_cl']
    # ###############
    # Plot guesses
    ###############
    print('This is the size of x_guess: ',x_guess.shape)
    plt.figure('x')
    plt.plot(plt.linspace(0,time_guess,N+1),plt.squeeze(x_guess))
    plt.figure('y')
    plt.plot(plt.linspace(0,time_guess,N+1),plt.squeeze(y_guess))
    plt.figure('vx')
    plt.plot(plt.linspace(0,time_guess,N+1),plt.squeeze(vx_guess))
    plt.figure('vy')
    plt.plot(plt.linspace(0,time_guess,N+1),plt.squeeze(vy_guess))
    plt.figure('psi')
    plt.plot(plt.linspace(0,time_guess,N+1),plt.squeeze(psi_guess))
    plt.figure('psi_dot')
    plt.plot(plt.linspace(0,time_guess,N+1),plt.squeeze(psi_dot_guess))
    plt.figure('throttle')
    plt.plot(plt.linspace(0, time_guess, N + 1), plt.squeeze(throttle_guess))
    plt.figure('delta')
    plt.plot(plt.linspace(0, time_guess, N + 1), plt.squeeze(delta_guess))
    plt.figure('throttle_dot')
    plt.plot(plt.linspace(0, time_guess, N ), plt.squeeze(throttle_dot_guess))
    plt.figure('delta_dot')
    plt.plot(plt.linspace(0, time_guess, N), plt.squeeze(delta_dot_guess))

    print("")
    print("vx_start: ",vx_start," and width_road: ",width_road)

    # Equations of the vehicle model
    x = SX.sym('x') # in global axis
    y = SX.sym('y') # in global axis
    vx = SX.sym('vx') # in local axis
    vy = SX.sym('vy') # in local axis
    psi = SX.sym('psi') #yaw angle
    psi_dot = SX.sym('psi_dot') #rate of yaw angle
    throttle = SX.sym('throttle')
    delta = SX.sym('delta')  # this is the angle of the front tire --> Gsteer factor needed if want steerwheelangle (Amesim model)

    # Controls
    throttle_dot = SX.sym('throttle_dot')
    delta_dot = SX.sym('delta_dot')

    # Other
    vx_des = SX.sym('vx_des')
    y_change = SX.sym('y_change')

    x_dot_glob = vx*cos(psi)-vy*sin(psi)
    y_dot_glob = vx*sin(psi)+vy*cos(psi)
    slipangle_f = plt.arctan2(vy+psi_dot*a,vx) - delta
    slipangle_r = plt.arctan2(vy-psi_dot*b,vx)
    Fxf = throttle*Tmax/(2*rw)
    c = Tmax/(2*rw)
    Fxr = Fxf
    Fyf = -2*Kyf* slipangle_f
    Fyr = -2*Kyr* slipangle_r
    F_d = Cr0 + Cr2*vx*vx
    atx = (cos(delta)*Fxf-sin(delta)*Fyf+Fxr-F_d)/M +vy*psi_dot # not total acceleration
    aty = (sin(delta)*Fxf+cos(delta)*Fyf+Fyr)/M -vx*psi_dot # not total acceleration
    an_y = vx*psi_dot
    psi_ddot = (sin(delta)*Fxf*a+cos(delta)*Fyf*a-b*Fyr)/Izz

    ax_total = (cos(delta)*Fxf-sin(delta)*Fyf+Fxr-F_d)/M

    ay_total = (sin(delta) * Fxf + cos(delta) * Fyf + Fyr) / M

    jx_total = (-sin(delta)*throttle*c+cos(delta)*throttle_dot*c+cos(delta)*2*Kyf*plt.arctan2(vy+psi_dot*a,vx)+sin(delta)*2*Kyf*(vx*aty+vx*psi_ddot*a-atx*vy-atx*psi_dot*a)/(vx**2+vy**2+2*vy*psi_dot*a+psi_dot**2*a**2)-cos(delta)*2*Kyf*delta-sin(delta)*2*Kyf*delta_dot+throttle_dot*c-Cr2*2*vx)/M

    jy_total = (cos(delta)*throttle*c+sin(delta)*throttle_dot*c+sin(delta)*2*Kyf*plt.arctan2(vy+psi_dot*a,vx)-cos(delta)*2*Kyf*(vx*aty+vx*psi_ddot*a-atx*vy-atx*psi_dot*a)/(vx**2+vy**2+2*vy*psi_dot*a+psi_dot**2*a**2)-sin(delta)*2*Kyf*delta+cos(delta)*2*Kyf*delta_dot-2*Kyr*(vx*aty-vx*psi_ddot*b-atx*vy+atx*psi_dot*b)/(vx**2+vy**2-2*vy*psi_dot*b+psi_dot**2*b**2))/M

    ax_total_int = ax_total ** 2
    ay_total_int = ay_total**2
    jx_total_int = jx_total ** 2
    jy_total_int = jy_total ** 2
    vx_diff_int = vx ** 2 - 2 * vx * vx_des + vx_des ** 2
    y_diff_int = y ** 2 - 2 * y * y_change + y_change ** 2

    # ----------------------------------
    #    continuous system dot(x)=f(x,u)
    # ----------------------------------
     # states: x, y, vx, vy, psi, psi_dot, throttle, delta
     # controls: throttle_dot, delta_dot
    rhs = vertcat(x_dot_glob, y_dot_glob, atx, aty, psi_dot, psi_ddot,throttle_dot,delta_dot)
    states = vertcat(x, y, vx, vy, psi, psi_dot, throttle, delta)
    controls = vertcat(throttle_dot, delta_dot)
    f = Function('f', [states,controls], [rhs],['states','controls'],['rhs'])

    # Other functions:
    AXT_int = Function('AXT_int', [states, controls], [ax_total_int], ['states', 'controls'], ['ax_total_int'])
    AYT_int = Function('AYT_int', [states, controls], [ay_total_int], ['states', 'controls'], ['ay_total_int'])
    JXT_int = Function('JXT_int', [states, controls], [jx_total_int], ['states', 'controls'], ['jx_total_int'])
    JYT_int = Function('JYT_int', [states, controls], [jy_total_int], ['states', 'controls'], ['jy_total_int'])
    VXD_int = Function('VXD_int', [states, controls], [vx_diff_int], ['states', 'controls'], ['vx_diff_int'])
    YD_int = Function('YD_int', [states, controls], [y_diff_int], ['states', 'controls'], ['y_diff_int'])

    ##
    # -----------------------------------
    #    Discrete system x_next = F(x,u)
    # -----------------------------------
    dt = SX.sym('dt')
    k1 = f(states, controls)
    k2 = f(states + dt/2 * k1, controls)
    k3 = f(states + dt/2 * k2, controls)
    k4 = f(states + dt * k3, controls)
    states_next = states+dt/6*(k1 +2*k2 +2*k3 +k4)
    F = Function('F', [states, controls, dt], [states_next],['states','controls','dt'],['states_next'])
    # -----------------------------------
    # -----------------------------------
    feature0_current = SX.sym('feature0_current')
    a1 = AXT_int(states, controls)
    a2 = AXT_int(states + dt / 2 * a1, controls)
    a3 = AXT_int(states + dt / 2 * a2, controls)
    a4 = AXT_int(states + dt * a3, controls)
    feature0_next = feature0_current + dt / 6 * (a1 + 2 * a2 + 2 * a3 + a4)
    AXT_F = Function('AXT_F', [states, controls, feature0_current, dt], [feature0_next], ['states', 'controls', 'feature0_current', 'dt'], ['feature0_next'])
    # -----------------------------------
    feature1_current = SX.sym('feature1_current')
    b1 = AYT_int(states, controls)
    b2 = AYT_int(states + dt / 2 * b1, controls)
    b3 = AYT_int(states + dt / 2 * b2, controls)
    b4 = AYT_int(states + dt * b3, controls)
    feature1_next = feature1_current + dt / 6 * (b1 + 2 * b2 + 2 * b3 + b4)
    AYT_F = Function('AYT_F', [states, controls, feature1_current, dt], [feature1_next], ['states', 'controls', 'feature1_current', 'dt'], ['feature1_next'])
    # -----------------------------------
    feature2_current = SX.sym('feature2_current')
    c1 = JXT_int(states, controls)
    c2 = JXT_int(states + dt / 2 * c1, controls)
    c3 = JXT_int(states + dt / 2 * c2, controls)
    c4 = JXT_int(states + dt * c3, controls)
    feature2_next = feature2_current + dt / 6 * (c1 + 2 * c2 + 2 * c3 + c4)
    JXT_F = Function('JXT_F', [states, controls, feature2_current, dt], [feature2_next],['states', 'controls', 'feature2_current', 'dt'], ['feature2_next'])
    # -----------------------------------
    feature3_current = SX.sym('feature3_current')
    d1 = JYT_int(states, controls)
    d2 = JYT_int(states + dt / 2 * d1, controls)
    d3 = JYT_int(states + dt / 2 * d2, controls)
    d4 = JYT_int(states + dt * d3, controls)
    feature3_next = feature3_current + dt / 6 * (d1 + 2 * d2 + 2 * d3 + d4)
    JYT_F = Function('JYT_F', [states, controls, feature3_current, dt], [feature3_next],['states', 'controls', 'feature3_current', 'dt'], ['feature3_next'])
    # -----------------------------------
    feature4_current = SX.sym('feature4_current')
    e1 = VXD_int(states, controls)
    e2 = VXD_int(states + dt / 2 * e1, controls)
    e3 = VXD_int(states + dt / 2 * e2, controls)
    e4 = VXD_int(states + dt * e3, controls)
    feature4_next = feature4_current + dt / 6 * (e1 + 2 * e2 + 2 * e3 + e4)
    VXD_F = Function('VXD_F', [states, controls, feature4_current, dt], [feature4_next],['states', 'controls', 'feature4_current', 'dt'], ['feature4_next'])
    # -----------------------------------
    feature5_current = SX.sym('feature5_current')
    f1 = YD_int(states, controls)
    f2 = YD_int(states + dt / 2 * f1, controls)
    f3 = YD_int(states + dt / 2 * f2, controls)
    f4 = YD_int(states + dt * f3, controls)
    feature5_next = feature5_current + dt / 6 * (f1 + 2 * f2 + 2 * f3 + f4)
    YD_F = Function('YD_F', [states, controls, feature5_current, dt], [feature5_next],['states', 'controls', 'feature5_current', 'dt'], ['feature5_next'])

    ##
    # -----------------------------------------------
    #    Optimal control problem, multiple shooting
    # -----------------------------------------------

    opti = casadi.Opti()
    X = opti.variable(nx,N+1)
    T =  opti.variable() # Time [s]
    feature0_current = opti.parameter()
    feature1_current = opti.parameter()
    feature2_current = opti.parameter()
    feature3_current = opti.parameter()
    feature4_current = opti.parameter()
    feature5_current = opti.parameter()
    opti.set_value(feature0_current,0)
    opti.set_value(feature1_current, 0)
    opti.set_value(feature2_current, 0)
    opti.set_value(feature3_current, 0)
    opti.set_value(feature4_current, 0)
    opti.set_value(feature5_current,0)

    # Aliases for states
    x  = X[0,:]
    y  = X[1,:]
    vx = X[2,:]
    vy = X[3,:]
    psi = X[4,:]
    psi_dot = X[5,:]
    throttle = X[6,:]
    delta = X[7,:]

    # Decision variables for control vector
    U =  opti.variable(nc,N)
    throttle_dot = U[0,:]
    delta_dot = U[1,:]

    # Gap-closing shooting constraints
    for k in range(N):
        opti.subject_to(X[:, k + 1] == F(X[:, k], U[:,k],T/N))

    # Path constraints
    opti.subject_to(opti.bounded(-1,throttle,1)) # local axis [m/s^2]
    # opti.subject_to(opti.bounded(-2.618,delta,2.618)) # Limit on steeringwheelangle (150°)
    opti.subject_to(opti.bounded(-width_road/2,y,width_road*3/2)) # Stay on road
    opti.subject_to(x[0,1:]>=0) # vehicle has to drive forward

    # Initial constraints
    # states: x, y, vx, vy, psi, psi_dot
    opti.subject_to(X[0:6,0]== vertcat(x_start,y_start,vx_start,vy_start,psi_start,psi_dot_start))
    opti.subject_to(X[6, 0] == (Cr0+Cr2*vx**2)/(2*c)) # no longitudinal acc when drag force taken into account
    opti.subject_to(X[7,0] == 0)
    # Controls set on zero in first interval --> want to start from steady state.
    # opti.subject_to(delta[0] == 0)
    # opti.subject_to(throttle[0] == 0)
    # (This will cause a very small deceleration but makes sure ay is starting from zero)
    # Terminal constraints
    opti.subject_to(y[-1] == width_road) # should move 3,47 m lateral
    opti.subject_to(vy[-1] == 0) # lane change is completed
    opti.subject_to(psi[-1] == 0) # assuming straight road
    opti.subject_to(psi_dot[-1] == 0)
    # opti.subject_to(delta[-1] == 0)
    limit = 150/16.96 # max steerwheelanlge / constant factor to front wheel (paper Son)
    opti.subject_to(opti.bounded(-limit*pi/180,delta,limit*pi/180))

    #  Set guesses
    opti.set_initial(x,x_guess)
    opti.set_initial(y,y_guess)
    opti.set_initial(vx,vx_guess)
    opti.set_initial(vy,vy_guess)
    opti.set_initial(psi,psi_guess)
    opti.set_initial(psi_dot,psi_dot_guess)
    opti.set_initial(throttle,throttle_guess)
    opti.set_initial(delta,delta_guess)
    opti.set_initial(throttle_dot, throttle_dot_guess)
    opti.set_initial(delta_dot, delta_dot_guess)
    opti.set_initial(T, time_guess)

    ##
    # -----------------------------------------------
    #    objective
    # -----------------------------------------------
    # Objective: The total accelerations in the local axis are considered!
    # time_list = []
    # for i in range(N+1):
    #     time_list.append(i*T/N)
    # time_vector = plt.array(time_list)
    #
    # # normal lateral accelleration
    # anx_list = []
    # for k in range(N+1):
    #     anx_list.append(-vy[k]*psi_dot[k])
    #
    # any_list = []
    # for k in range(N+1):
    #     any_list.append(vx[k]*psi_dot[k])
    #
    # # tangential lateral accelleration
    # aty_list = []
    # atx_list = []
    # psi_ddot_list = []
    # for i in plt.arange(0, N, 1):
    #     atx_list.append(f(X[:,i],U[:,i])[2,0])
    #     aty_list.append(f(X[:,i],U[:,i])[3,0])
    #     psi_ddot_list.append(f(X[:, i], U[:, i])[5, 0])
    # for i in plt.arange(N,N+1,1):
    #     aty_list.append((vy[i] - vy[i - 1]) / (T / N))
    #     atx_list.append((vx[i] - vx[i - 1]) / (T / N))
    #     psi_ddot_list.append((psi_dot[i] - psi_dot[i - 1]) / (T / N))
    # for i in plt.arange(0, len(time_list), 1):
    #     if i == 0:
    #         aty_list.append((vy[i + 1]-vy[i])/(T/N))
    #         # aty_list.append(0) # manually removed as opti variable in order to avoid large psi_ddot an jy peaks at the start
    #     elif i == len(time_list)-1:
    #         aty_list.append((vy[i]-vy[i-1])/(T/N))
    #     else:
    #         aty_list.append((vy[i + 1] - vy[i - 1]) / (2 * (T/N)))

    # tangential longitudinal accelleration
    # atx_list = []
    # for i in plt.arange(0, len(time_list), 1):
    #     if i == 0:
    #         atx_list.append((vx[i + 1]-vx[i])/(T/N))
    #     elif i == len(time_list)-1:
    #         atx_list.append((vx[i]-vx[i-1])/(T/N))
    #     else:
    #         atx_list.append((vx[i + 1] - vx[i - 1]) / (2 * (T/N)))

    # ax_tot = plt.array(atx_list) + plt.array(anx_list)
    # ay_tot = plt.array(aty_list) + plt.array(any_list)

    # yaw_acceleration

    # for i in plt.arange(0, len(time_list), 1):
    #     if i == 0:
    #         psi_ddot_list.append((psi_dot[i + 1] - psi_dot[i]) / (T / N))
    #     elif i == len(time_list) - 1:
    #         psi_ddot_list.append((psi_dot[i] - psi_dot[i - 1]) / (T / N))
    #     else:
    #         psi_ddot_list.append((psi_dot[i + 1] - psi_dot[i - 1]) / (2 * (T / N)))

    # calculation longitudinal jerk --> jerk is calculated from the total acceleration!
    # Implementation of second order scheme
    # Longitudinal jerk
    # jxt_list = []
    # for i in plt.arange(0, len(time_list), 1):
    #     if i == 0:
    #         jxt_list.append((atx_list[i + 1] - atx_list[i]) / (T / N))
    #     elif i == len(time_list) - 1:
    #         jxt_list.append((atx_list[i] - atx_list[i - 1]) / (T / N))
    #     else:
    #         # jx_list.append((ax_tot[i + 1] - ax_tot[i - 1]) / (2 * dt_sol))
    #         jxt_list.append((vx[i + 1] - 2 * vx[i] + vx[i - 1]) / ((T / N) ** 2))
    #
    # jxn_list = []
    # for k in range(N + 1):
    #     jxn_list.append(-psi_dot[k] * aty_list[k] - vy[k] * psi_ddot_list[k])
    #
    # jx_tot = plt.array(jxt_list) + plt.array(jxn_list)
    #
    # # calculation lateral jerk -> jerk is calculated from the total acceleration!
    # # Implementation of second order scheme
    # jy_list_t = []
    # for i in plt.arange(0, len(time_list), 1):
    #     if i == 0:
    #         jy_list_t.append((aty_list[i + 1]-aty_list[i])/(T/N))
    #     elif i == len(time_list)-1:
    #         jy_list_t.append((aty_list[i]-aty_list[i-1])/(T/N))
    #     else:
    #         # jy_list.append((ay_tot[i + 1] - ay_tot[i - 1]) / (2 * (T/N)))
    #         jy_list_t.append((vy[i + 1] -2*vy[i] +vy[i - 1]) / ((T / N)**2))
    #
    # jy_list_n = []
    # for k in range(N+1):
    #     jy_list_n.append(psi_dot[k]*atx_list[k] + vx[k]*psi_ddot_list[k])
    # jy_tot = plt.array(jy_list_t) + plt.array(jy_list_n)
    #
    # vx_des_list = []
    # for k in range(N+1):
    #     vx_des_list.append(vx[k]-vx_start)
    #
    # y_des_list = []
    # for k in range(N+1):
    #     y_des_list.append(y[k]-width_road)

    # Extra constraints on acceleration and jerk:
    # opti.subject_to(aty_list[-1] == 0) # to avoid shooting through
    opti.subject_to(T<50)
    # opti.subject_to(jy_tot[-1] == 0) # fully end of lane change --> no lateral acceleration in the next sample
    # opti.subject_to(aty_list[0] == 0) # start from the beginning of the lane change
    # opti.subject_to(jy_tot[0] == 0) # start from the beginning of the lane change

    # for i in plt.arange(0,N+1,1):
    #     opti.subject_to(jy_list_t[i] <= 5)
    #     opti.subject_to(jy_list_t[i] >= -5)


    # Comfort cost function: t0*axtot**2+t1*aytot**2+t2*jxtot**2+t3*jytot**2+t4*(vx-vdes)**2+t5*(y-ydes)**2
    for i in plt.arange(0,N,1):
        feature0_current =  AXT_F(X[:,i],U[:,i],feature0_current,T/N)
        feature1_current =  AYT_F(X[:, i], U[:, i], feature1_current, T / N)
        feature2_current =  JXT_F(X[:, i], U[:, i], feature2_current, T / N)
        feature3_current =  JYT_F(X[:, i], U[:, i], feature3_current, T / N)
        feature4_current =  VXD_F(X[:, i], U[:, i], feature4_current, T / N)
        feature5_current =  YD_F(X[:, i], U[:, i], feature5_current, T / N)


    # # f0: longitudinal acceleration
    # integrand = ax_tot** 2
    # f0_cal = 0
    # for i in plt.arange(0, len(integrand) - 1, 1):
    #     f0_cal = f0_cal + 0.5 * (integrand[i] + integrand[i + 1]) * (T/N)
    #
    # # f0_cal = scipy.integrate.simps(integrand,plt.array(time_list))
    #
    # # f1: lateral acceleration
    # integrand = ay_tot** 2
    # f1_cal = 0
    # for i in plt.arange(0, len(integrand) - 1, 1):
    #     f1_cal = f1_cal + 0.5 * (integrand[i] + integrand[i + 1]) * (T/N)
    # # f1_cal = scipy.integrate.simps(integrand,plt.array(time_list))
    # # print('f1: ',f1_cal)
    #
    # # f2: lateral jerk
    # integrand = jy_tot** 2
    # f2_cal = 0
    # for i in plt.arange(0, len(integrand) - 1, 1):
    #     f2_cal = f2_cal + 0.5 * (integrand[i] + integrand[i + 1]) * (T/N)
    # # f2_cal = scipy.integrate.simps(integrand,plt.array(time_list))
    #
    # # f3: desired velocity
    # integrand = plt.array(vx_des_list)** 2
    # f3_cal = 0
    # for i in plt.arange(0, len(integrand) - 1, 1):
    #     f3_cal = f3_cal + 0.5 * (integrand[i] + integrand[i + 1]) * (T/N)
    # # f4_cal = scipy.integrate.simps(integrand,plt.array(time_list))
    #
    # # f4: desired lane change
    # integrand = plt.array(y_des_list)**2
    # f4_cal = 0
    # for i in plt.arange(0, len(integrand) - 1, 1):
    #     f4_cal = f4_cal + 0.5 * (integrand[i] + integrand[i + 1]) * (T/N)
    # # f5_cal = scipy.integrate.simps(integrand,plt.array(time_list))
    #
    # # f5: lateral jerk
    # integrand = jx_tot ** 2
    # f5_cal = 0
    # for i in plt.arange(0, len(integrand) - 1, 1):
    #     f5_cal = f5_cal + 0.5 * (integrand[i] + integrand[i + 1]) * (T / N)
    # # f2_cal = scipy.integrate.simps(integrand,plt.array(time_list))

    # Comfort cost function: t0*ax**2+t1*ay**2+t2*jy**2+t3*(vx-vdes)**2+t4*(y-ydes)**2
    opti.minimize(theta[0]/norm0*feature0_current+theta[1]/norm1*feature1_current+theta[2]/norm2*feature2_current+theta[3]/norm3*feature3_current+theta[4]/norm4*feature4_current+theta[5]/norm5*feature5_current)
    # opti.minimize(theta[0]/norm0*f0_cal+theta[1]/norm1*f1_cal+theta[2]/norm2*f2_cal+theta[3]/norm3*f3_cal+theta[4]/norm4*f4_cal)

    print('Absolute weights: ',theta/plt.array([norm0,norm1,norm2,norm3,norm4,norm5]))
    print('Relative weights: ', theta)

    # Implementation of the solver
    options = dict()
    # options["expand"] = True
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
    throttle_dot_sol = sol.value(throttle_dot)
    delta_dot_sol = sol.value(delta_dot)
    T_sol = sol.value(T)
    dt_sol = T_sol/N
    ax_tot_sol = sol.value(ax_total)
    ay_tot_sol = sol.value(ay_total)
    jx_tot_sol = sol.value(jx_total)
    jy_tot_sol = sol.value(jy_total)
    psi_ddot_sol = sol.value(psi_ddot)
    aty_sol = sol.value(aty)
    any_sol = sol.value(an_y)

    width = plt.around(width_road, 2)
    speed = plt.around(vx_start, 2)
    define_plots("1",x_sol,y_sol,vx_sol,vy_sol,ax_tot_sol,ay_tot_sol,jx_tot_sol,jy_tot_sol,psi_sol,psi_dot_sol,psi_ddot_sol,throttle_sol,delta_sol,T_sol,aty_sol,any_sol,speed,width)

    print("\n")
    print('Integrated feature values: ')
    print('------------------------------')
    print('integrand = plt.squeeze(data_cl[ax_cl]**2)')
    print(sol.value(feature0_current))
    print('integrand = plt.squeeze(data_cl[ay_cl] ** 2)')
    print(sol.value(feature1_current))
    print('integrand = plt.squeeze(data_cl[jx_cl] ** 2)')
    print(sol.value(feature2_current))
    print('integrand = plt.squeeze(data_cl[jy_cl] ** 2)')
    print(sol.value(feature3_current))
    print('integrand = plt.squeeze((desired_speed - data_cl[vx_cl]) ** 2)')
    print(sol.value(feature4_current))
    print('integrand = plt.squeeze((delta_lane - data_cl[y_cl]) ** 2)')
    print(sol.value(feature5_current))

    # ----------------------------------
    #    Storing of data in csv-file
    # ----------------------------------

    # path = "writting_C\ DATAC2_V" + str(speed) + "_L"+str(width)+".csv"
    # file = open(path,'w',newline= "")
    # writer = csv.writer(file)
    # writer.writerow(["time","x","y","vx","vy","ax","ay","jx","jy","psi","psi_dot","psi_ddot","throttle","delta","aty","any","atx","anx"])
    #
    # for i in range(N+1):
    #     if i == N: # last control point has no physical meaning
    #         writer.writerow([i * dt_sol, x_sol[i], y_sol[i], vx_sol[i], vy_sol[i], ax_tot_sol[i], ay_tot_sol[i], jx_tot_sol[i], jy_tot_sol[i],psi_sol[i], psi_dot_sol[i], psi_ddot_sol[i], throttle_sol[i-1], delta_sol[i-1], aty_sol[i], a_ny_sol[i],atx_sol[i], anx_sol[i]])
    #     else:
    #         writer.writerow([i * dt_sol, x_sol[i], y_sol[i], vx_sol[i], vy_sol[i], ax_tot_sol[i], ay_tot_sol[i], jx_tot_sol[i], jy_tot_sol[i],psi_sol[i], psi_dot_sol[i], psi_ddot_sol[i], throttle_sol[i], delta_sol[i], aty_sol[i],a_ny_sol[i],atx_sol[i], anx_sol[i]])
    #
    # file.close()
    # print('dt of the optimization is: ', dt_sol)
    # print('')
    # print('Simulation completed!')
    # print('\n')


# ----------------------------------
#    Show
# ----------------------------------

plt.show()