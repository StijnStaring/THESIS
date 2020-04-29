import pylab as plt
from derivative import derivative
from define_plots import define_plots
from casadi import *

# Parameters vehicle model
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
N = 100
# T_guess = 6
vx_delta = 6
x_start = 0
y_start = 0
vx_start = 22.22
vy_start = 0
psi_start = 0
psi_dot_start = 0

# Resampling and guesses: shape 1x501
# time_guess = plt.linspace(0,T_guess,N+1)
# time_guess = time_guess[plt.newaxis,:]
# x_guess = plt.zeros((1,len(time_guess)))
# vx_guess = plt.zeros((1,len(time_guess)))
# for x in time_guess:
#     x_guess = -1.5 / 108 * (x - 3) ** 4 + 1.5 / 2 * x ** 2 + 1.125
#     vx_guess = -1.5 / 27 * (x - 3) ** 3 + 1.5 * x - 1.5 + vx_start
# x_guess = x_guess[plt.newaxis,:]
# vx_guess = vx_guess[plt.newaxis,:]
# y_guess = 1e-5*plt.ones((1,N+1))
# vy_guess = 1e-5*plt.ones((1,N+1))
# psi_guess = 1e-5*plt.ones((1,N+1))
# psi_dot_guess = 1e-5*plt.ones((1,N+1))
# throttle_guess = 1e-5*plt.ones((1,N))
# delta_guess = 1e-5*plt.ones((1,N))

# plt.figure('xguess')
# plt.plot(plt.squeeze(time_guess),plt.squeeze(x_guess))
# plt.figure('vxguess')
# plt.plot(plt.squeeze(time_guess),plt.squeeze(vx_guess))

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

# H = Function('H', [delta,throttle,vx, vy,psi_dot], [ddx],['delta','throttle','vx','vy','psi_dot'],['ddx'])
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
F = Function('F', [states, controls,dt], [states_next],['states','controls','dt'],['states_next'])

##
# -----------------------------------------------
#    Optimal control problem, multiple shooting
# -----------------------------------------------

opti = casadi.Opti()
X = opti.variable(nx, N + 1)
T = opti.variable()
# opti.set_value(T,6)

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
# ref = opti.parameter(2,N+1)
# opti.set_value(ref[0,:],x_guess[0,:])
# opti.set_value(ref[1,:],vx_guess[0,:])

# Gap-closing shooting constraints
for k in range(N):
    opti.subject_to(X[:, k + 1] == F(X[:, k], U[:,k], T / N))
for i in range(N):
    ddx = f(X[:, i], U[:, i])
    opti.minimize()

opti.subject_to(X[:,0]== vertcat(x_start,y_start,vx_start,vy_start,psi_start,psi_dot_start))
for i in range(N+1):
    opti.subject_to(vy[i] == 0)
    opti.subject_to(psi_dot[i] == 0)
    opti.subject_to(delta[i] == 0)

# opti.subject_to(psi_dot[1:] == plt.zeros((1,N)))
# opti.subject_to(delta[1:] == plt.zeros((1,N-1)))
# opti.subject_to(opti.bounded(-1,throttle,1)) # local axis [m/s^2]
# for i in plt.arange(1,len(atx_list),1):
#     opti.subject_to(atx_list[i]<=3.5)
#     opti.subject_to(atx_list[i] >= -3.5)

# tolerance = 2
v_des = vx_start+vx_delta
# v_min = vx_start - tolerance
# v_max = v_des + tolerance
# opti.subject_to(opti.bounded(v_min,vx,v_max))
# opti.subject_to(opti.bounded(-1,y,1))
# opti.subject_to(opti.bounded(-1,vy,1))
# opti.subject_to(opti.bounded(-10*pi/180,psi,10*pi/180))
# opti.subject_to(opti.bounded(-10*pi/180,psi_dot,10*pi/180))
# opti.subject_to(x[0,1:]>=0)
opti.subject_to(vx[-1] == v_des)

# Objective: The total accelerations in the local axis are considered!
# time_list = []
# for i in range(N + 1):
#     time_list.append(i * T / N)
# time_vector = plt.array(time_list)
#
# # normal lateral accelleration
# anx_list = []
# for k in range(N + 1):
#     anx_list.append(-vy[k] * psi_dot[k])
#
# # tangential longitudinal accelleration
# atx_list = []
# for i in plt.arange(0, len(time_list), 1):
#     if i == 0:
#         atx_list.append((vx[i + 1] - vx[i]) / (T / N))
#     elif i == len(time_list) - 1:
#         atx_list.append((vx[i] - vx[i - 1]) / (T / N))
#     else:
#         atx_list.append((vx[i + 1] - vx[i - 1]) / (2 * (T / N)))
#
# ax_tot = plt.array(atx_list) + plt.array(anx_list)
#
#
# # yaw_acceleration
# psi_ddot_list = []
# for i in plt.arange(0, len(time_list), 1):
#     if i == 0:
#         psi_ddot_list.append((psi_dot[i + 1] - psi_dot[i]) / (T / N))
#     elif i == len(time_list) - 1:
#         psi_ddot_list.append((psi_dot[i] - psi_dot[i - 1]) / (T / N))
#     else:
#         psi_ddot_list.append((psi_dot[i + 1] - psi_dot[i - 1]) / (2 * (T / N)))
#
# # calculation longitudinal jerk -> jerk is calculated from the total acceleration!
# # Implementation of second order scheme
# jx_list_t = []
# for i in plt.arange(0, len(time_list), 1):
#     if i == 0:
#         jx_list_t.append((atx_list[i + 1] - atx_list[i]) / (T / N))
#     elif i == len(time_list) - 1:
#         jx_list_t.append((atx_list[i] - atx_list[i - 1]) / (T / N))
#     else:
#         # jx_list.append((ax_tot[i + 1] - ax_tot[i - 1]) / (2 * (T/N)))
#         jx_list_t.append((vx[i + 1] - 2 * vx[i] + vx[i - 1]) / ((T / N) ** 2))
#
# jx_list_n = []
# for k in range(N + 1):
#     jx_list_n.append(psi_dot[k] * atx_list[k] + vx[k] * psi_ddot_list[k])
# jx_tot = plt.array(jx_list_t) + plt.array(jx_list_n)
#
# for i in plt.arange(0,len(jx_tot),1):
#     opti.minimize(jx_tot[i]*jx_tot[i])
#     # opti.minimize(ax_tot[i] * ax_tot[i])


# opti.minimize(sumsqr(x)+sumsqr(y)+100*sumsqr(vx-ref[0,0:N+1])+sumsqr(vy)+sumsqr(psi)+sumsqr(psi_dot))
# for i in plt.arange(Cr0,N+1,1):
    # opti.minimize(H(delta[i],throttle[i],vx[i], vy[i],psi_dot[i]))

# Set guesses
# x_guess = time_guess**2/2+22.22*time_guess
# opti.set_initial(x,x_guess)
# opti.set_initial(y,y_guess)
vx_guess = plt.linspace(vx_start,v_des,N+1)
opti.set_initial(vx,vx_guess)
# opti.set_initial(vy,vy_guess)
# opti.set_initial(psi,psi_guess)
# opti.set_initial(psi_dot,psi_dot_guess)
# opti.set_initial(throttle,throttle_guess)
# opti.set_initial(delta,delta_guess)
# opti.set_initial(T,6)

opti.solver('ipopt')
sol = opti.solve()

###############################
# Post-processing
###############################
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

width = 0
speed = plt.around(vx_start, 2)
define_plots("1", x_sol, y_sol, vx_sol, vy_sol, ax_tot_sol, ay_tot_sol, jx_tot_sol, jy_tot_sol, psi_sol, psi_dot_sol,
             psi_ddot_sol, throttle_sol, delta_sol, T_sol, aty_sol, any_sol, speed, width)

plt.show()
