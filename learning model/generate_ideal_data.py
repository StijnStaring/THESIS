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
from scipy import integrate, signal
from import_data import import_data
from define_plots import define_plots
from jerk import jerk
import pylab as plt
from casadi import *

[norm0,norm1,norm2,norm3,norm4,init_matrix,des_matrix,dict_list,files] = import_data(1)
data_cl = dict_list[0]
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
N = 600

x_start = 0
y_start = 0
vx_start = 80/3.6 # this is also the desired velocity
vy_start = 0
psi_start = 0
psi_dot_start = 0
width_road = 3

# Resampling and guesses
x_guess = signal.resample(data_cl['x_cl'],N+1).T
y_guess = signal.resample(data_cl['y_cl'],N+1).T
vx_guess = signal.resample(data_cl['vx_cl'],N+1).T
vy_guess = signal.resample(data_cl['vy_cl'],N+1).T
psi_guess = signal.resample(data_cl['yaw_cl'],N+1).T
psi_dot_guess = signal.resample(data_cl['r_cl'],N+1).T
throttle_guess = signal.resample(data_cl['throttle_cl'],N).T
# delta_guess = signal.resample(data_cl['steering_deg_cl']*plt.pi/180,N).T
time_guess = des_matrix[0,2]

# Comfort cost function: t0*ax**2+t1*ay**2+t2*jy**2+t3*(vx-vdes)**2+t4*(y-ydes)**2
# Normalization numbers are taken from the non-linear tracking algorithm --> take the inherentely difference in order of size into account.
theta = plt.array([2,5,6,2,4]) # deze wegingsfactoren dienen achterhaald te worden.

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
# opti.set_initial(delta,delta_guess)
opti.set_initial(T, time_guess)

##
# -----------------------------------------------
#    objective
# -----------------------------------------------
# Objective: The total accelerations in the local axis are considered!
time_list = []
for i in range(N): # These derivatives of the states of a point less: N instead of N+1
    time_list.append(i*T/N)
time_vector = plt.array(time_list)

anx_list = []
for k in range(N): # These derivatives of the states of a point less: N instead of N+1
    anx_list.append(-vy[k]*psi_dot[k])

any_list = []
for k in range(N): # These derivatives of the states of a point less: N instead of N+1
    any_list.append(vx[k]*psi_dot[k])

ax_list = []
for k in range(N): # These derivatives of the states of a point less: N instead of N+1
    ax_list.append(f(X[:, k], U[:,k])[2])
ay_list = []
for k in range(N):
    ay_list.append(f(X[:, k], U[:,k])[3])

ax_tot = plt.array(ax_list) + plt.array(anx_list)
ay_tot = plt.array(ay_list) + plt.array(any_list)

# calculation lateral jerk
jy_list = []
for i in plt.arange(0, len(ay_tot), 1):
    if i == 0:
        jy_list.append((ay_tot[i + 1]-ay_tot[i])/(T/N))
    elif i == len(ay_tot)-1:
        jy_list.append((ay_tot[i]-ay_tot[i-1])/(T/N))
    else:
        jy_list.append((ay_tot[i + 1] - ay_tot[i - 1]) / (2 * (T/N)))

vx_des_list = []
for k in range(N): # These derivatives of the states of a point less: N instead of N+1
    vx_des_list.append(vx[k]-vx_start)

y_des_list = []
for k in range(N): # These derivatives of the states of a point less: N instead of N+1
    y_des_list.append(y[k]-width_road)

# Extra constraints on acceleration and jerk:
opti.subject_to(ax_tot[0] == 0)
opti.subject_to(ay_tot[0] == 0)
opti.subject_to(jy_list[0] == 0)


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
integrand = plt.array(jy_list)** 2
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


# Comfort cost function: t0*ax**2+t1*ay**2+t2*jy**2+t3*an**2+t4*(vx-vdes)**2+t5*(y-ydes)**2
# opti.minimize(theta[1]/norm1*f1_cal+theta[2]/norm2*f2_cal+theta[3]/norm3*f3_cal)
opti.minimize(theta[0]/norm0*f0_cal+theta[1]/norm1*f1_cal+theta[2]/norm2*f2_cal+theta[3]/norm3*f3_cal+theta[4]/norm4*f4_cal)
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
for k in range(N): # These derivatives of the states of a point less: N instead of N+1
    ax_list.append(sol.value(f(X[:, k], U[:,k])[2]))
ax_sol = plt.array(ax_list)
anx_list = []
for k in range(N):
    anx_list.append(sol.value(-vy[k]*psi_dot[k]))
anx_sol = plt.array(anx_list)
ax_tot_sol = ax_sol + anx_sol

ay_list = []
for k in range(N):
    ay_list.append(sol.value(f(X[:, k], U[:,k])[3]))
ay_sol = plt.array(ay_list)
any_list = []
for k in range(N):
    any_list.append(sol.value(vx[k]*psi_dot[k]))
any_sol = plt.array(any_list)
ay_tot_sol = ay_sol + any_sol

jx_sol = jerk(ax_tot_sol,dt_sol)
jy_sol = jerk(ay_tot_sol,dt_sol)


define_plots("1",x_sol,y_sol,vx_sol,vy_sol,ax_tot_sol,ay_tot_sol,jx_sol,jy_sol,psi_sol,psi_dot_sol,throttle_sol,delta_sol,T_sol)

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
# path = "written_data\ N_600_V_80.csv"
# file = open(path,'w',newline= "")
# writer = csv.writer(file)
# writer.writerow(["time","x","y","vx","vy","ax","ay","jx","jy","psi","psi_dot","throttle","delta"])
#
# for i in range(N+1):
#     if i == N:
#         writer.writerow([i * dt_sol, x_sol[i], y_sol[i], vx_sol[i], vy_sol[i], ax_sol[i - 1], ay_sol[i - 1], jx_sol[i - 1],jy_sol[i - 1], psi_sol[i], psi_dot_sol[i], throttle_sol[i - 1], delta_sol[i - 1]])
#     else:
#         writer.writerow([i * dt_sol, x_sol[i], y_sol[i], vx_sol[i], vy_sol[i], ax_sol[i], ay_sol[i], jx_sol[i], jy_sol[i],psi_sol[i], psi_dot_sol[i], throttle_sol[i], delta_sol[i]])
#
# file.close()

# ----------------------------------
#    Show
# ----------------------------------
plt.show()