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
import scipy
from scipy import integrate
import pylab as plt
from casadi import *

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
# Comfort cost function: t0*ax**2+t1*ay**2+t2*jy**2+t3*an**2+t4*(vx-vdes)**2+t5*(y-ydes)**2
# Normalization numbers are taken from the non-linear tracking algorithm --> take the inherentely difference in order of size into account.
theta = plt.array([2,])
# normalization information
# integrand = plt.squeeze(ax** 2)
norm0 = 0.3072585725919982
# integrand = plt.squeeze(ay** 2)
norm1 = 2.0824922161681316
# integrand = plt.squeeze(jy ** 2)
norm2 = 41.4796020568006
# integrand = plt.squeeze((-vy*dpsi) ** 2 + (vx*dpsi)**2)
norm3 = 35.511231053915
# integrand = plt.squeeze((desired_speed - vx) ** 2)
norm4 = 0.2172162321065016
# integrand = plt.squeeze((delta_lane - y) ** 2)
norm5 = 13.158914673920528


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
opti.subject_to(opti.bounded(-2.618,delta,2.618)) # Limit on steeringwheelangle (150°)
opti.subject_to(opti.bounded(-width_road/2,y,width_road*3/2)) # Stay on road

# Initial constraints
# states: x, y, vx, vy, psi, psi_dot
opti.subject_to(X[:,0]== vertcat(x_start,y_start,vx_start,vy_start,psi_start,psi_dot_start))

# Terminal constraints
opti.subject_to(y[-1] == width_road) # should move 3 m lateral
opti.subject_to(vy[-1] == 0) # lane change is completed
opti.subject_to(psi[-1] == 0) # assuming straight road
opti.subject_to(psi_dot[-1] == 0)

#  Set guesses
opti.set_initial(T, 1)

##
# -----------------------------------------------
#    objective
# -----------------------------------------------
# Objective: (need to be normalized?)
time_list = []
for i in range(N):
    time_list.append(i*dt)
time_vector = plt.array(time_list)

anx_list = []
for k in range(N):
    anx_list.append(-vy[k]*psi_dot[k])

any_list = []
for k in range(N): # one point less taken in comparison of the total amount
    any_list.append(vx[k]*psi_dot[k])

ax_list = []
for k in range(N): # These derivatives of the states of a point less: N instead of N+1
    ax_list.append(f(X[:, k], U[:,k])[2])
ay_list = []
for k in range(N):
    ay_list.append(f(X[:, k], U[:,k])[3])

# calculation lateral jerk
jy_list = []
for i in plt.arange(0, len(ay_list), 1):
    if i == 0:
        jy_list.append((ay_list[i + 1]-ay_list[i])/dt)
    elif i == len(ay_list)-1:
        jy_list.append((ay_list[i]-ay_list[i-1])/dt)
    else:
        jy_list.append((ay_list[i + 1] - ay_list[i - 1]) / (2 * dt))

vx_des_list = []
for k in range(N): # These derivatives of the states of a point less: N instead of N+1
    vx_des_list.append(vx[k]-vx_start)

y_des_list = []
for k in range(N): # These derivatives of the states of a point less: N instead of N+1
    y_des_list.append(y[k]-width_road)

# k1 = (f(states, controls)[2])**2+(f(states, controls)[3])**2
# k2 = (f(states + dt/2 * k1, controls)[2])**2+(f(states + dt/2 * k1, controls)[3])**2
# k3 = (f(states + dt/2 * k2, controls)[2])**2+(f(states + dt/2 * k2, controls)[3])**2
# k4 = (f(states + dt * k3, controls)[2])**2+(f(states + dt * k3, controls)[3])**2
# integrand_extra = dt/6*(k1 +2*k2 +2*k3 +k4)
# F1 = Function('F1', [states, controls, dt], [integrand_extra],['states','controls','dt'],['integrand_next'])

# F1_int = 0
# for k in range(N):
#     F1_int = F1_int + F1(X[:, k], U[:,k],T/N)

# Comfort cost function: t0*ax**2+t1*ay**2+t2*jy**2+t3*an**2+t4*(vx-vdes)**2+t5*(y-ydes)**2

# f0: longitudinal acceleration
integrand = plt.array(ax_list)** 2
f0_cal = scipy.integrate.simps(integrand,plt.array(time_list))

# f1: lateral acceleration
integrand = plt.array(ay_list)** 2
f1_cal = scipy.integrate.simps(integrand,plt.array(time_list))
# print('f1: ',f1_cal)

# f2: lateral jerk
integrand = plt.array(jy_list)** 2
f2_cal = scipy.integrate.simps(integrand,plt.array(time_list))

# f3: centriputal force
integrand = plt.array(anx_list)** 2+plt.array(any_list)** 2
f3_cal = scipy.integrate.simps(integrand,plt.array(time_list))

# f4: desired velocity
integrand = plt.array(vx_des_list)** 2
f4_cal = scipy.integrate.simps(integrand,plt.array(time_list))

# f5: desired lane change
integrand = plt.array(y_des_list)** 2
f5_cal = scipy.integrate.simps(integrand,plt.array(time_list))

opti.minimize(theta[0]/norm0*f0_cal+theta[1]/norm1*f1_cal+theta[2]/norm2*f2_cal+theta[3]*f3_cal+theta[4]*f4_cal+theta[5]*f5_cal)





