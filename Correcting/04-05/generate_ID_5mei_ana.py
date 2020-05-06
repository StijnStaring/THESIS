"""
This script is generating data using the same vehicle model as in the learning algorithm and uses the comfortcostfunction
as objective when pathplanning.
Feedforward control of the vehicle --> One optimization is solved over the whole control horizon.
Linear tyre model -> valid if slipangle < 5 degrees and lat_acc < 4 m/s^2
Producing different lane changes: different longitudinal start velocities and lateral distances to be convered
:return: csv-datafiles
"""
import glob
import csv
import pylab as plt
from scipy import signal

# from scipy import integrate
from import_ID_5mei_ana import import_ID_5mei
from define_plots_5mei_ana import define_plots
from casadi import *

norm0 = 0.007276047781441449
norm1 = 2.6381715506137424
norm2 = 0.007276047781441449
norm3 = 11.283498669013454
norm4 = 0.046662223759442054
norm5 = 17.13698903738383

# Parameters of the non-linear bicycle model used to generate the data. (from Siemens)
# Remark: x and y are coordinates in global axis
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
nx = 10 # amount of states
nc = 2 # amount of controls
N = 1000 # amount of control intervals

x_start = 0
y_start = 0
vy_start = 0
psi_start = 0
psi_dot_start = 0

# ----------------------------------
#    for loop over optimization
# ----------------------------------
# N_list = [500,1000]
N_list = [1000]
# T_limit_list = [20,50,100]
T_limit_list = [30]
for N in N_list:
    for T_limit in T_limit_list:
        print("")
        print("start simulatie!")
        print('--------------------------------------------------')
        print('N: ',N, 'and T_limit: ',T_limit)
        file_list = glob.glob("reading_dataset/*.csv")
        for file in file_list:
            print('\n')
            print("The name of the file: ", file)
            data_cl = import_ID_5mei(file) # imported the solution of nummeric (other approach)
            width_road = data_cl['width']
            vx_start = data_cl['vx_start']

            # Resampling mostly not needed with this data file
            N_old = data_cl['x_cl'].shape[1] - 1
            time_guess = data_cl['time_cl'][0,-1]
            x_guess = signal.resample_poly(data_cl['x_cl'], N, N_old,axis= 1,padtype='line')
            x_guess = x_guess[None,0,0:N+1]
            # x_guess = data_cl['x_cl']
            y_guess = signal.resample_poly(data_cl['y_cl'], N, N_old ,axis= 1, padtype='maximum')
            y_guess = y_guess[None, 0, 0:N + 1]
            # y_guess = data_cl['y_cl']
            # vx_guess = signal.resample_poly(data_cl['vx_cl'], N, N_old,axis= 1 ,padtype='line')
            # vx_guess = vx_guess[None, 0, 0:N + 1]
            vx_guess = signal.resample(data_cl['vx_cl'],N+1,axis= 1)
            # vx_guess = data_cl['vx_cl']
            vy_guess = signal.resample(data_cl['vy_cl'], N + 1,axis= 1)
            # vy_guess = data_cl['vy_cl']
            psi_guess = signal.resample(data_cl['psi_cl'], N + 1,axis= 1)
            # psi_guess = data_cl['psi_cl']
            psi_dot_guess = signal.resample(data_cl['psi_dot_cl'], N + 1,axis= 1)
            # psi_dot_guess = data_cl['psi_dot_cl']
            throttle_guess = signal.resample(data_cl['throttle_cl'], N+1,axis= 1) # throttle and delta use to be inputs
            delta_guess = signal.resample(data_cl['delta_cl'], N+1,axis= 1)
            throttle_dot_guess = signal.resample(data_cl['throttle_dot_cl'], N,axis= 1)
            # throttle_dot_guess = data_cl['throttle_dot_cl']
            delta_dot_guess = signal.resample(data_cl['delta_dot_cl'], N,axis= 1)
            # delta_dot_guess = data_cl['delta_dot_cl']

            # ###############
            # Plot guesses
            ###############
            # print('This is the size of x_guess: ',x_guess.shape)
            # plt.figure('x')
            # plt.plot(plt.linspace(0,time_guess,N+1),plt.squeeze(x_guess))
            # plt.figure('y')
            # plt.plot(plt.linspace(0,time_guess,N+1),plt.squeeze(y_guess))
            # plt.figure('vx')
            # plt.plot(plt.linspace(0,time_guess,N+1),plt.squeeze(vx_guess))
            # plt.figure('vy')
            # plt.plot(plt.linspace(0,time_guess,N+1),plt.squeeze(vy_guess))
            # plt.figure('psi')
            # plt.plot(plt.linspace(0,time_guess,N+1),plt.squeeze(psi_guess))
            # plt.figure('psi_dot')
            # plt.plot(plt.linspace(0,time_guess,N+1),plt.squeeze(psi_dot_guess))
            # plt.figure('throttle')
            # plt.plot(plt.linspace(0, time_guess, N + 1), plt.squeeze(throttle_guess))
            # plt.figure('delta')
            # plt.plot(plt.linspace(0, time_guess, N + 1), plt.squeeze(delta_guess))
            # plt.figure('throttle_dot')
            # plt.plot(plt.linspace(0, time_guess, N ), plt.squeeze(throttle_dot_guess))
            # plt.figure('delta_dot')
            # plt.plot(plt.linspace(0, time_guess, N), plt.squeeze(delta_dot_guess))

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
            ax_total = SX.sym('ax_tot')
            ay_total = SX.sym('ay_tot')

            # Controls
            throttle_dot = SX.sym('throttle_dot')
            delta_dot = SX.sym('delta_dot')

            # Other
            vx_des = vx_start
            y_change = width_road

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
            atx = (cos(delta)*Fxf-sin(delta)*Fyf+Fxr-F_d)/M +vy*psi_dot # tangential acceleration
            aty = (sin(delta)*Fxf+cos(delta)*Fyf+Fyr)/M -vx*psi_dot
            an_y = vx*psi_dot # normal acceleration
            anx = -vy*psi_dot
            psi_ddot = (sin(delta)*Fxf*a+cos(delta)*Fyf*a-b*Fyr)/Izz

            # ax_total = (cos(delta)*Fxf-sin(delta)*Fyf+Fxr-F_d)/M
            # ay_total = (sin(delta) * Fxf + cos(delta) * Fyf + Fyr) / M

            # j_total = derivative(ax_total(t),t) --> see photos of my notes (c)
            jx_total = (c*throttle_dot - 2*Cr2*vx*atx + 2*Kyf*sin(delta)*(((a*psi_ddot + aty)/vx - ((vy + a*psi_dot)*atx)/vx**2)/((vy + a*psi_dot)**2/vx**2 + 1) - delta_dot) + c*cos(delta)*throttle_dot - 2*Kyf*cos(delta)*(delta- plt.arctan2((vy + a*psi_dot),vx))*delta_dot - c*sin(delta)*throttle*delta_dot)/M
            jy_total = ((2*Kyr*((b*psi_ddot - aty)/vx + ((vy - b*psi_dot)*atx)/vx**2))/((vy - b*psi_dot)**2/vx**2 + 1) - 2*Kyf*cos(delta)*(((a*psi_ddot + aty)/vx - ((vy + a*psi_dot)*atx)/vx**2)/((vy + a*psi_dot)**2/vx**2 + 1) - delta_dot) + c*sin(delta)*throttle_dot - 2*Kyf*sin(delta)*(delta - plt.arctan2((vy + a*psi_dot),vx))*delta_dot + c*cos(delta)*throttle*delta_dot)/M

            ax_total_int = ax_total ** 2
            ay_total_int = ay_total**2
            jx_total_int = jx_total ** 2
            jy_total_int = jy_total ** 2
            vx_diff_int = vx ** 2 - 2 * vx * vx_des + vx_des ** 2 # (vx- vx_des)**2 --> vx_des is the start vx at beginning lane change
            y_diff_int = y ** 2 - 2 * y * y_change + y_change ** 2 # (y-y_des)**2 --> y_des is 3.47, distance to be travelled to change lane

            # ----------------------------------
            #    continuous system dot(x)=f(x,u)
            # ----------------------------------
             # states: x, y, vx, vy, psi, psi_dot, throttle, delta
             # controls: throttle_dot, delta_dot
            rhs = vertcat(x_dot_glob, y_dot_glob, atx, aty, psi_dot, psi_ddot,throttle_dot,delta_dot,jx_total,jy_total)
            states = vertcat(x, y, vx, vy, psi, psi_dot, throttle, delta,ax_total,ay_total)
            controls = vertcat(throttle_dot, delta_dot)
            f = Function('f', [states,controls], [rhs],['states','controls'],['rhs'])

            # Other functions:
            stock = Function('stock', [states], [ax_total,ay_total,psi_ddot,atx,anx,aty,an_y], ['states'], ['ax_total','ay_total','psi_ddot','atx','anx','aty','an_y'])
            stock2 = Function('stock2', [states,controls], [jx_total,jy_total], ['states','controls'],['jx_total','jy_total'])

            AXT_int = Function('AXT_int', [states], [ax_total_int], ['states'], ['ax_total_int'])
            AYT_int = Function('AYT_int', [states], [ay_total_int], ['states'], ['ay_total_int'])
            JXT_int = Function('JXT_int', [states, controls], [jx_total_int], ['states', 'controls'], ['jx_total_int'])
            JYT_int = Function('JYT_int', [states, controls], [jy_total_int], ['states', 'controls'], ['jy_total_int'])
            VXD_int = Function('VXD_int', [states], [vx_diff_int], ['states'], ['vx_diff_int'])
            YD_int = Function('YD_int', [states], [y_diff_int], ['states'], ['y_diff_int'])

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
            # Objective of path planning = th0/n0*int(ax**2,dt)+th1/n1*int(ay**2,dt)+th2/n2*int(jx**2,dt)+th3/n3*int(jy**2,dt)+th4/n4*int(vx_diff**2,dt)+th5/n5*int(y_diff**2,dt)
            # integration needed over the entire time horizon
            # th_ = theta/weigth, n_ = norm factor
            # -----------------------------------

            ##
            # -----------------------------------------------
            #    Optimal control problem, multiple shooting
            # -----------------------------------------------

            opti = casadi.Opti()
            X = opti.variable(nx,N+1)
            U = opti.variable(nc, N)
            T =  opti.variable() # Time [s]

            # Aliases for states
            x  = X[0,:]
            y  = X[1,:]
            vx = X[2,:]
            vy = X[3,:]
            psi = X[4,:] #yaw
            psi_dot = X[5,:] #yaw rate
            throttle = X[6,:] # gas padel
            delta = X[7,:] # angle of front wheel
            ax_total = X[8,:]
            ay_total = X[9,:]

            # Decision variables for control vector
            throttle_dot = U[0,:]
            delta_dot = U[1,:]

            # Gap-closing shooting constraints
            for k in range(N):
                opti.subject_to(X[:, k + 1] == F(X[:, k], U[:,k],T/N))

            # Path constraints
            opti.subject_to(opti.bounded(-1,throttle,1))
            # opti.subject_to(opti.bounded(-2.618,delta,2.618)) # Limit on steeringwheelangle (150°)
            opti.subject_to(opti.bounded(-width_road/2,y,width_road*3/2)) # Stay on road--> start middle of right lane of a two lane road
            opti.subject_to(x[0,1:] >= 0 ) # vehicle has to drive forward

            # Initial constraints
            # states: x, y, vx, vy, psi, psi_dot, throttle, delta
            opti.subject_to(X[0:6,0]== vertcat(x_start,y_start,vx_start,vy_start,psi_start,psi_dot_start))
            # jx_start = JX_TOT(X[:,0],U[:,0])
            # jy_start = JY_TOT(X[:, 0], U[:, 0])
            # opti.subject_to(jx_start == 0)
            # opti.subject_to(jy_start == 0)
            opti.subject_to(X[6, 0] == (Cr0+Cr2*vx_start**2)/(2*c)) # no longitudinal acc when drag force taken into account
            opti.subject_to(X[7,0] == 0) # driving straigth
            # T_limit = 10
            opti.subject_to(T <= T_limit)

            # Terminal constraints
            opti.subject_to(y[-1] == width_road) # should move 3,47 m
            opti.subject_to(vy[-1] == 0) # lane change is completed
            opti.subject_to(psi[-1] == 0) # assuming straight road
            opti.subject_to(psi_dot[-1] == 0) # Fy is zero
            opti.subject_to(delta[-1] == 0)  # Fy is zero
            Gsteer = 16.96
            limit = 150/Gsteer # max steerwheelangle / constant factor to front wheel (paper Son)
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

            # Comfort cost function: t0/n0*axtot**2+t1/n1*aytot**2+t2/n2*jxtot**2+t3/n3*jytot**2+t4/n4*(vx-vdes)**2+t5/n5*(y-ydes)**2
            # Jerk is evaluated at point of states and when just the new control is applied --> beginning of next interval.
            f0_cal = 0
            f1_cal = 0
            f2_cal = 0
            f3_cal = 0
            f4_cal = 0
            f5_cal = 0
            for i in plt.arange(0, N, 1):
                if i == N-1:
                    f0_cal = f0_cal + 0.5 * (AXT_int(X[:, i]) + AXT_int(X[:,i + 1])) * (T / N)
                    f1_cal = f1_cal + 0.5 * (AYT_int(X[:, i]) + AYT_int(X[:,i + 1])) * (T / N)
                    f2_cal = f2_cal + 0.5 * (JXT_int(X[:, i], U[:, i]) +  JXT_int(X[:, i+1], U[:, i]))* (T / N)
                    f3_cal = f3_cal + 0.5 * (JYT_int(X[:, i], U[:, i]) + JYT_int(X[:, i+1], U[:, i]))* (T / N)
                    f4_cal = f4_cal + 0.5 * (VXD_int(X[:, i]) + VXD_int(X[:,i + 1])) * (T / N)
                    f5_cal = f5_cal + 0.5 * (YD_int(X[:, i]) + YD_int(X[:,i + 1])) * (T / N)
                else:
                    f0_cal = f0_cal + 0.5 * (AXT_int(X[:,i]) + AXT_int(X[:,i + 1]))* (T / N)
                    f1_cal = f1_cal + 0.5 * (AYT_int(X[:,i]) + AYT_int(X[:,i + 1]))*(T / N)
                    f2_cal = f2_cal + 0.5 * (JXT_int(X[:, i],U[:,i]) + JXT_int(X[:,i + 1],U[:,i+1])) * (T / N)
                    f3_cal = f3_cal + 0.5 * (JYT_int(X[:, i],U[:,i]) + JYT_int(X[:,i + 1],U[:,i+1])) * (T / N)
                    f4_cal = f4_cal + 0.5 * (VXD_int(X[:, i]) + VXD_int(X[:,i + 1])) * (T / N)
                    f5_cal = f5_cal + 0.5 * (YD_int(X[:, i]) + YD_int(X[:,i + 1]))* (T / N)

            # Comfort cost function: t0*ax**2+t1*ay**2+t2*jy**2+t3*(vx-vdes)**2+t4*(y-ydes)**2
            opti.minimize(theta[0]/norm0*f0_cal+theta[1]/norm1*f1_cal+theta[2]/norm2*f2_cal+theta[3]/norm3*f3_cal+theta[4]/norm4*f4_cal+theta[5]/norm5*f5_cal)
            # opti.minimize(theta[0] / norm0 * f0_cal + theta[1] / norm1 * f1_cal + theta[4] / norm4 * f4_cal +theta[5] / norm5 * f5_cal)

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
            ax_tot_sol = plt.zeros(N+1)
            ay_tot_sol = plt.zeros(N+1)
            jx_tot_sol = plt.zeros(N+1)
            jy_tot_sol = plt.zeros(N+1)
            psi_ddot_sol = plt.zeros(N+1)
            aty_sol = plt.zeros(N+1)
            any_sol = plt.zeros(N+1)
            atx_sol = plt.zeros(N+1)
            anx_sol = plt.zeros(N+1)

            for i in plt.arange(0,N+1,1):
                res = stock(sol.value(X[:,i]))
                ax_tot_sol[i] = res[0]
                ay_tot_sol[i] = res[1]
                psi_ddot_sol[i] = res[2]
                atx_sol[i] = res[3]
                anx_sol[i] = res[4]
                aty_sol[i] = res[5]
                any_sol[i] = res[6]

            for i in plt.arange(0,N,1):
                res = stock2(sol.value(X[:,i]),sol.value(U[:,i]))
                jx_tot_sol[i] = res[0]
                jy_tot_sol[i] = res[1]
            jx_tot_sol[N] = (ax_tot_sol[N] - ax_tot_sol[N-1])/(T_sol/N)
            jy_tot_sol[N] = (ay_tot_sol[N] - ay_tot_sol[N-1])/(T_sol/N)

            width = plt.around(width_road, 2)
            speed = plt.around(vx_start, 2)
            define_plots("1",x_sol,y_sol,vx_sol,vy_sol,ax_tot_sol,ay_tot_sol,jx_tot_sol,jy_tot_sol,psi_sol,psi_dot_sol,psi_ddot_sol,throttle_sol,delta_sol,throttle_dot_sol,delta_dot_sol,T_sol,aty_sol,any_sol,atx_sol,anx_sol,speed,width)

            print("")
            print("dt is equal to: ", dt_sol)
            print("")
            print('Integrated feature values: ')
            print('------------------------------')
            print('integrand = plt.squeeze(data_cl[ax_cl]**2)')
            print(sol.value(f0_cal))
            print('integrand = plt.squeeze(data_cl[ay_cl] ** 2)')
            print(sol.value(f1_cal))
            print('integrand = plt.squeeze(data_cl[jx_cl] ** 2)')
            print(sol.value(f2_cal))
            print('integrand = plt.squeeze(data_cl[jy_cl] ** 2)')
            print(sol.value(f3_cal))
            print('integrand = plt.squeeze((desired_speed - data_cl[vx_cl]) ** 2)')
            print(sol.value(f4_cal))
            print('integrand = plt.squeeze((delta_lane - data_cl[y_cl]) ** 2)')
            print(sol.value(f5_cal))

            print("")
            if (T_limit - dt_sol) < T_sol:
                print("-------------------------------------")
                print("Warning: Time constraint is binding!")
                print("-------------------------------------")
            print('Simulation completed!')


            # ----------------------------------
            #    Storing of data in a csv-file
            # ----------------------------------

            path = "writting_C\ DATAC2_V" + str(speed) + "_L"+str(width)+".csv"
            file = open(path,'w',newline= "")
            writer = csv.writer(file)
            writer.writerow(["time","x","y","vx","vy","ax","ay","jx","jy","psi","psi_dot","psi_ddot","throttle","delta","throttle_dot","delta_dot","aty","any","atx","anx"])
            a_ny_sol = any_sol
            for i in range(N+1):
                if i == N: # last control point has no physical meaning
                    writer.writerow([i * dt_sol, x_sol[i], y_sol[i], vx_sol[i], vy_sol[i], ax_tot_sol[i], ay_tot_sol[i], jx_tot_sol[i], jy_tot_sol[i],psi_sol[i], psi_dot_sol[i], psi_ddot_sol[i], throttle_sol[i], delta_sol[i], throttle_dot_sol[i-1],delta_dot_sol[i-1], aty_sol[i], a_ny_sol[i],atx_sol[i], anx_sol[i]])
                else:
                    writer.writerow([i * dt_sol, x_sol[i], y_sol[i], vx_sol[i], vy_sol[i], ax_tot_sol[i], ay_tot_sol[i], jx_tot_sol[i], jy_tot_sol[i],psi_sol[i], psi_dot_sol[i], psi_ddot_sol[i], throttle_sol[i], delta_sol[i],throttle_dot_sol[i],delta_dot_sol[i], aty_sol[i],a_ny_sol[i],atx_sol[i], anx_sol[i]])

            file.close()
            print('dt of the optimization is: ', dt_sol)
            print('')
            print('Simulation completed!')
            print('\n')


# ----------------------------------
#    Show
# ----------------------------------
plt.show()

