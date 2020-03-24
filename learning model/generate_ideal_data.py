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
import csv
import pylab as plt
from scipy import signal
# from import_data import import_data
from import_ideal_data import import_ideal_data
from define_plots import define_plots
from derivative import derivative
# from generate_delta_guess import generate_delta_guess
from casadi import *

# [_,_,_,_,_,init_matrix,des_matrix,dict_list,files] = import_data(0)
# width_road = 3.46990715
# vx_start = 23.10159175
time_guess = 4.01
data_cl = import_ideal_data()

# theta = plt.array([4,5,6,1,2]) en met data guess 1 berekende norm waarden en data guess 1 zelf. (example lane change)
norm0 = 0.007276047781441449
norm1 = 2.6381715506137424
norm2 = 11.283498669013454
norm3 = 0.046662223759442054
norm4 = 17.13698903738383

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
pi = 3.14159265359

# Parameters of the optimization
nx = 6 # amount of states
nc = 2 # amount of controls
N = 500

x_start = 0
y_start = 0
# vx_start = des_matrix[0,1] # this is the desired velocity

vy_start = 0
psi_start = 0
psi_dot_start = 0
# width_road = des_matrix[0,0]

# Resampling and guesses
x_guess = signal.resample(data_cl['x_cl'],N+1).T
y_guess = signal.resample(data_cl['y_cl'],N+1).T
vx_guess = signal.resample(data_cl['vx_cl'],N+1).T
vy_guess = signal.resample(data_cl['vy_cl'],N+1).T
psi_guess = signal.resample(data_cl['psi_cl'],N+1).T
psi_dot_guess = signal.resample(data_cl['psi_dot_cl'],N+1).T
throttle_guess = signal.resample(data_cl['throttle_cl'],N).T
delta_guess = signal.resample(data_cl['delta_cl'],N).T # Error made in data Siemens --> SWA not 40 degrees
# time_guess = des_matrix[0,2]
# delta_guess = generate_delta_guess(time_guess,N)[plt.newaxis,:]
# plt.figure()
# plt.plot(plt.linspace(0,time_guess,delta_guess.shape[1]),plt.squeeze(delta_guess)*180/plt.pi)
# plt.xlabel("Time [s]", fontsize=14)
# plt.ylabel("delta [degrees]", fontsize=14)
# plt.title('delta local', fontsize=14)
# plt.grid(True)

# ----------------------------------
#    for loop over optimization
# ----------------------------------
# vx_start_a = plt.array([80/3.6,90/3.6,100/3.6,110/3.6])
# width_road_a = plt.array([3.46990715, 3.46990715*2])
vx_start_a = plt.array([80/3.6])
width_road_a = plt.array([3.46990715])
for vx_start in vx_start_a:
    for width_road in width_road_a:


        # Comfort cost function: t0*ax**2+t1*ay**2+t2*jy**2+t3*(vx-vdes)**2+t4*(y-ydes)**2
        # Normalization numbers are taken from the non-linear tracking algorithm --> take the inherentely difference in order of size into account.
        # theta = plt.array([4,5,6,1,2]) # deze wegingsfactoren dienen achterhaald te worden. (in ax en ay zit ook de normal acceleration)
        # theta = plt.array([4,5,6,1,2])
        theta = plt.array([4,5,6,1,2])

        # Equations of the vehicle model
        x = MX.sym('x') # in global axis
        y = MX.sym('y') # in global axis
        vx = MX.sym('vx') # in local axis
        vy = MX.sym('vy') # in local axis
        psi = MX.sym('psi') #yaw angle
        psi_dot = MX.sym('psi_dot') #rate of yaw angle

        #Controls
        throttle = MX.sym('throttle')
        delta = MX.sym('delta')

        x_dot_glob = vx*cos(psi)-vy*sin(psi)
        y_dot_glob = vx*sin(psi)+vy*cos(psi)
        slipangle_f = plt.arctan2(vy+psi_dot*a,vx) - delta
        slipangle_r = plt.arctan2(vy-psi_dot*b,vx)
        Fxf = throttle*Tmax/(2*rw)
        Fxr = Fxf
        Fyf = -2*Kyf* slipangle_f
        Fyr = -2*Kyr* slipangle_r
        F_d = Cr0 + Cr2*vx*vx
        ddx = (cos(delta)*Fxf-sin(delta)*Fyf+Fxr-F_d)/M +vy*psi_dot # not total acceleration
        ddy = (sin(delta)*Fxf+cos(delta)*Fyf+Fyr)/M -vx*psi_dot # not total acceleration
        ddpsi = (sin(delta)*Fxf*a+cos(delta)*Fyf*a-b*Fyr)/Izz

        # ----------------------------------
        #    continuous system dot(x)=f(x,u)
        # ----------------------------------
         # states: x, y, vx, vy, psi, psi_dot
         # controls: T, delta_dot
        rhs = vertcat(x_dot_glob, y_dot_glob, ddx, ddy, psi_dot, ddpsi)
        states = vertcat(x, y, vx, vy, psi, psi_dot)
        controls = vertcat(throttle, delta)
        f = Function('f', [states,controls], [rhs],['states','controls'],['rhs'])

        ##
        # -----------------------------------
        #    Discrete system x_next = F(x,u)
        # -----------------------------------
        dt = MX.sym('dt')
        k1 = f(states, controls)
        k2 = f(states + dt/2 * k1, controls)
        k3 = f(states + dt/2 * k2, controls)
        k4 = f(states + dt * k3, controls)
        states_next = states+dt/6*(k1 +2*k2 +2*k3 +k4)
        F = Function('F', [states, controls, dt], [states_next],['states','controls','dt'],['states_next'])

        ##
        # -----------------------------------------------
        #    Optimal control problem, multiple shooting
        # -----------------------------------------------

        opti = casadi.Opti()
        X = opti.variable(nx,N+1)
        T =  opti.variable() # Time [s]

        # Aliases for states
        x    = X[0,:]
        y  = X[1,:]
        vx = X[2,:]
        vy = X[3,:]
        psi = X[4,:]
        psi_dot = X[5,:]

        # Decision variables for control vector
        U =  opti.variable(nc,N)
        throttle = U[0,:]
        delta = U[1,:]

        # Gap-closing shooting constraints
        for k in range(N):
            opti.subject_to(X[:, k + 1] == F(X[:, k], U[k],T/N))

        # Path constraints
        opti.subject_to(opti.bounded(-1,throttle,1)) # local axis [m/s^2]
        # opti.subject_to(opti.bounded(-2.618,delta,2.618)) # Limit on steeringwheelangle (150°)
        opti.subject_to(opti.bounded(-width_road/2,y,width_road*3/2)) # Stay on road
        opti.subject_to(x[0,1:]>=0) # vehicle has to drive forward

        # Initial constraints
        # states: x, y, vx, vy, psi, psi_dot
        opti.subject_to(X[:,0]== vertcat(x_start,y_start,vx_start,vy_start,psi_start,psi_dot_start))
        # Controls set on zero in first interval --> want to start from steady state.
        # opti.subject_to(delta[0] == 0)
        # opti.subject_to(throttle[0] == 0)
        # (This will cause a very small deceleration but makes sure ay is starting from zero)
        # Terminal constraints
        opti.subject_to(y[-1] == width_road) # should move 3 m lateral
        opti.subject_to(vy[-1] == 0) # lane change is completed
        opti.subject_to(psi[-1] == 0) # assuming straight road
        opti.subject_to(psi_dot[-1] == 0)


        #  Set guesses
        opti.set_initial(x,x_guess)
        opti.set_initial(y,y_guess)
        opti.set_initial(vx,vx_guess)
        opti.set_initial(vy,vy_guess)
        opti.set_initial(psi,psi_guess)
        opti.set_initial(psi_dot,psi_dot_guess)
        opti.set_initial(throttle,throttle_guess)
        opti.set_initial(delta,delta_guess)
        opti.set_initial(T, time_guess)

        ##
        # -----------------------------------------------
        #    objective
        # -----------------------------------------------
        # Objective: The total accelerations in the local axis are considered!
        time_list = []
        for i in range(N+1):
            time_list.append(i*T/N)
        time_vector = plt.array(time_list)

        # normal lateral accelleration
        anx_list = []
        for k in range(N+1):
            anx_list.append(-vy[k]*psi_dot[k])

        any_list = []
        for k in range(N+1):
            any_list.append(vx[k]*psi_dot[k])

        # tangential lateral accelleration
        aty_list = []
        for i in plt.arange(0, len(time_list), 1):
            if i == 0:
                aty_list.append((vy[i + 1]-vy[i])/(T/N))
            elif i == len(time_list)-1:
                aty_list.append((vy[i]-vy[i-1])/(T/N))
            else:
                aty_list.append((vy[i + 1] - vy[i - 1]) / (2 * (T/N)))

        # tangential longitudinal accelleration
        atx_list = []
        for i in plt.arange(0, len(time_list), 1):
            if i == 0:
                atx_list.append((vx[i + 1]-vx[i])/(T/N))
            elif i == len(time_list)-1:
                atx_list.append((vx[i]-vx[i-1])/(T/N))
            else:
                atx_list.append((vx[i + 1] - vx[i - 1]) / (2 * (T/N)))

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
                jy_list_t.append((aty_list[i + 1]-aty_list[i])/(T/N))
            elif i == len(time_list)-1:
                jy_list_t.append((aty_list[i]-aty_list[i-1])/(T/N))
            else:
                # jy_list.append((ay_tot[i + 1] - ay_tot[i - 1]) / (2 * (T/N)))
                jy_list_t.append((vy[i + 1] -2*vy[i] +vy[i - 1]) / ((T / N)**2))

        jy_list_n = []
        for k in range(N+1):
            jy_list_n.append(psi_dot[k]*atx_list[k] + vx[k]*psi_ddot_list[k])
        jy_tot = plt.array(jy_list_t) + plt.array(jy_list_n)

        vx_des_list = []
        for k in range(N+1):
            vx_des_list.append(vx[k]-vx_start)

        y_des_list = []
        for k in range(N+1):
            y_des_list.append(y[k]-width_road)

        # Extra constraints on acceleration and jerk:
        opti.subject_to(aty_list[-1] == 0) # to avoid shooting through
        opti.subject_to(jy_tot[-1] == 0) # fully end of lane change --> no lateral acceleration in the next sample
        opti.subject_to(aty_list[0] == 0) # start from the beginning of the lane change
        opti.subject_to(jy_tot[0] == 0) # start from the beginning of the lane change


        # Comfort cost function: t0*axtot**2+t1*aytot**2+t2*jytot**2+t3*(vx-vdes)**2+t4*(y-ydes)**2

        # f0: longitudinal acceleration
        integrand = ax_tot** 2
        f0_cal = 0
        for i in plt.arange(0, len(integrand) - 1, 1):
            f0_cal = f0_cal + 0.5 * (integrand[i] + integrand[i + 1]) * (T/N)

        # f0_cal = scipy.integrate.simps(integrand,plt.array(time_list))

        # f1: lateral acceleration
        integrand = ay_tot** 2
        f1_cal = 0
        for i in plt.arange(0, len(integrand) - 1, 1):
            f1_cal = f1_cal + 0.5 * (integrand[i] + integrand[i + 1]) * (T/N)
        # f1_cal = scipy.integrate.simps(integrand,plt.array(time_list))
        # print('f1: ',f1_cal)

        # f2: lateral jerk
        integrand = jy_tot** 2
        f2_cal = 0
        for i in plt.arange(0, len(integrand) - 1, 1):
            f2_cal = f2_cal + 0.5 * (integrand[i] + integrand[i + 1]) * (T/N)
        # f2_cal = scipy.integrate.simps(integrand,plt.array(time_list))

        # f3: desired velocity
        integrand = plt.array(vx_des_list)** 2
        f3_cal = 0
        for i in plt.arange(0, len(integrand) - 1, 1):
            f3_cal = f3_cal + 0.5 * (integrand[i] + integrand[i + 1]) * (T/N)
        # f4_cal = scipy.integrate.simps(integrand,plt.array(time_list))

        # f4: desired lane change
        integrand = plt.array(y_des_list)** 2
        f4_cal = 0
        for i in plt.arange(0, len(integrand) - 1, 1):
            f4_cal = f4_cal + 0.5 * (integrand[i] + integrand[i + 1]) * (T/N)
        # f5_cal = scipy.integrate.simps(integrand,plt.array(time_list))


        # Comfort cost function: t0*ax**2+t1*ay**2+t2*jy**2+t3*(vx-vdes)**2+t4*(y-ydes)**2
        opti.minimize(theta[0]/norm0*f0_cal+theta[1]/norm1*f1_cal+theta[2]/norm2*f2_cal+theta[3]/norm3*f3_cal+theta[4]/norm4*f4_cal)
        # opti.minimize(theta[0]/norm0*f0_cal+theta[1]/norm1*f1_cal+theta[2]/norm2*f2_cal+theta[3]/norm3*f3_cal+theta[4]/norm4*f4_cal)

        print('Absolute weights: ',theta/plt.array([norm0,norm1,norm2,norm3,norm4]))
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
        dt_sol = T_sol/(len(x_sol)-1)
        ax_list = []

        # for k in range(N+1):
        #     ax_list.append(sol.value(f(X[:, k], U[:,k])[2]))
        # ax_sol = plt.array(ax_list)

        anx_list = []
        for k in range(N+1):
            anx_list.append(sol.value(-vy[k]*psi_dot[k]))
        anx_sol = plt.array(anx_list)

        vx_list = []
        for k in range(N+1):
            vx_list.append(vx_sol[k])
        atx_sol = derivative(vx_list,dt_sol)
        ax_tot_sol = atx_sol + anx_sol

        any_list = []
        for k in range(N+1):
            any_list.append(sol.value(vx[k]*psi_dot[k]))
        any_sol = plt.array(any_list)

        vy_list = []
        for k in range(N+1):
            vy_list.append(vy_sol[k])
        aty_sol = derivative(vy_list,dt_sol)

        ay_tot_sol = aty_sol + any_sol

        psi_ddot_sol = derivative(sol.value(psi_dot),dt_sol)

        # Lateral jerk
        jyt_sol = []
        for i in plt.arange(0, len(vy_sol), 1):
            if i == 0:
                jyt_sol.append((aty_sol[i + 1]-aty_sol[i])/dt_sol)
            elif i == len(vy_sol)-1:
                jyt_sol.append((aty_sol[i]-aty_sol[i-1])/dt_sol)
            else:
                # jy_list.append((ay_tot[i + 1] - ay_tot[i - 1]) / (2 * dt_sol))
                jyt_sol.append((vy_sol[i + 1] -2*vy_sol[i] +vy_sol[i - 1]) / (dt_sol**2))

        jyn_sol = []
        for k in range(N+1):
            jyn_sol.append(sol.value(psi_dot)[k]*atx_sol[k] + vx_sol[k]*psi_ddot_sol[k])

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

        width = plt.around(width_road, 2)
        speed = plt.around(vx_start, 2)
        define_plots("1",x_sol,y_sol,vx_sol,vy_sol,ax_tot_sol,ay_tot_sol,jx_tot_sol,jy_tot_sol,psi_sol,psi_dot_sol,psi_ddot_sol,throttle_sol,delta_sol,T_sol,aty_sol,any_sol,speed,width)

        print("\n")
        print('Integrated feature values: ')
        print('------------------------------')
        print('integrand = plt.squeeze(data_cl[ax_cl]**2)')
        print(sol.value(f0_cal))
        print('integrand = plt.squeeze(data_cl[ay_cl] ** 2)')
        print(sol.value(f1_cal))
        print('integrand = plt.squeeze(data_cl[jy_cl] ** 2)')
        print(sol.value(f2_cal))
        print('integrand = plt.squeeze((delta_lane - data_cl[y_cl]) ** 2)')
        print(sol.value(f3_cal))
        print('integrand = plt.squeeze((desired_speed - data_cl[vx_cl]) ** 2)')
        print(sol.value(f4_cal))

        # ----------------------------------
        #    Storing of data in csv-file
        # ----------------------------------

        path = "check_2deriv\ DATA_2deriv_V" + str(speed) + "_L"+str(width)+".csv"
        file = open(path,'w',newline= "")
        writer = csv.writer(file)
        # writer.writerow(["time","x","y","vx","vy","ax","ay","jx","jy","psi","psi_dot","throttle","delta","aty","any"])

        for i in range(N+1):
            if i == N: # last control point has no physical meaning
                writer.writerow([i * dt_sol, x_sol[i], y_sol[i], vx_sol[i], vy_sol[i], ax_tot_sol[i], ay_tot_sol[i], jx_tot_sol[i], jy_tot_sol[i],psi_sol[i], psi_dot_sol[i], psi_ddot_sol[i], throttle_sol[i-1], delta_sol[i-1], aty_sol[i], any_sol[i]])
            else:
                writer.writerow([i * dt_sol, x_sol[i], y_sol[i], vx_sol[i], vy_sol[i], ax_tot_sol[i], ay_tot_sol[i], jx_tot_sol[i], jy_tot_sol[i],psi_sol[i], psi_dot_sol[i], psi_ddot_sol[i], throttle_sol[i], delta_sol[i], aty_sol[i],any_sol[i]])

        file.close()
        print('dt of the optimization is: ', dt_sol)

# ----------------------------------
#    Show
# ----------------------------------

plt.show()