"""
stijnstaring@hotmail.com
"""
# Importations
# local functions
from define_plots import define_plots
from guess_states import guess_states
from jerk_vehicle_model import jerk_vehicle_model
# global functions
import pylab as plt
from rockit import *
# init_matrix = [x,vx,ax,jx,y,vy,ay,jy]
# des_matrix = [delta_lane, desired speed, time_lane_change]
def optim_weights(theta,init_matrix,des_matrix,dict_list,files,theta_iter,plot,f_obs,axcom1a,axcom1b,axcom2,axcom3a,axcom3b,axcom4a,axcom4b,axcom5a,axcom5b,axcom6a,axcom6b,axcom7):

    # Opti variables
    ################
    # 200 samples --> manoeuvre +- 4 s --> dt = 0.02s --> +- 0.5 m per sample

    data_cl = dict_list[0]
    CP = len(data_cl['x_cl'])
    IP = 1
    amount = len(dict_list)
    if plot == 1:
        [ax1a,ax1b,ax2,ax3a,ax3b,ax4a,ax4b,ax5a,ax5b,ax6a,ax6b,ax8] = define_plots(theta_iter)

    his_x = plt.zeros((init_matrix.shape[0],CP+1))
    his_vx = plt.zeros((init_matrix.shape[0], CP + 1))
    his_ax = plt.zeros((init_matrix.shape[0], CP + 1))
    his_jx = plt.zeros((init_matrix.shape[0], CP + 1))

    his_y = plt.zeros((init_matrix.shape[0], CP + 1))
    his_vy = plt.zeros((init_matrix.shape[0], CP + 1))
    his_ay = plt.zeros((init_matrix.shape[0], CP + 1))
    his_jy = plt.zeros((init_matrix.shape[0], CP + 1))

    his_time_cal_lc = plt.zeros((amount,2))


    for k in plt.arange(0,amount,1):


    # Optimization
    ##############
        ocp = Ocp(T=FreeTime(des_matrix[k,2]))

        # Parameters
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

        # Optimization variables
        # States
        x = ocp.state()  # in global axis
        y = ocp.state()  # in global axis
        vx = ocp.state()  # in local axis
        vy = ocp.state()  # in local axis
        psi = ocp.state()  # yaw  angle in radials
        dpsi = ocp.state()  # rate of yaw angle in radials per second

        # Controls
        throttle = ocp.control()  # bounded between -1 and 1
        delta = ocp.control(order= 1)  # steerwheelangle in radians

        # equations of the model
        vx_glob = vx * plt.cos(psi) - vy * plt.sin(psi)
        vy_glob = vx * plt.sin(psi) + vy * plt.cos(psi)
        slipangle_f = plt.arctan2(vy + dpsi * a, vx) - delta
        slipangle_r = plt.arctan2(vy - dpsi * b, vx)
        Fxf = throttle * Tmax / (2 * rw)
        Fxr = Fxf
        Fyf = -2 * Kyf * slipangle_f
        Fyr = -2 * Kyr * slipangle_r
        F_d = Cr0 + Cr2 * vx * vx
        # Local accelerations --> not total ones wrt global axis!
        ax_con = (plt.cos(delta) * Fxf - plt.sin(delta) * Fyf + Fxr - F_d) / M + vy * dpsi
        ay_con = (plt.sin(delta) * Fxf + plt.cos(delta) * Fyf + Fyr) / M - vx * dpsi
        ddpsi = (plt.sin(delta) * Fxf * a + plt.cos(delta) * Fyf * a - b * Fyr) / Izz

        # Path closing constraints
        ocp.set_der(x, vx_glob)
        ocp.set_der(y, vy_glob)
        ocp.set_der(vx, ax_con)
        ocp.set_der(vy, ay_con)
        ocp.set_der(psi, dpsi)
        ocp.set_der(dpsi, ddpsi)

        # delta_lane = des_matrix[k, 0]
        desired_speed = des_matrix[k,1]

        # Define a placeholder for concrete waypoints to be defined on edges of the control grid
        waypoints = ocp.parameter(2, grid='control')

        ocp.add_objective(ocp.integral((x - waypoints[0]) ** 2 + (y - waypoints[1]) ** 2, grid='control'))

        waypoints_num = plt.squeeze(plt.array([data_cl['x_cl'],data_cl['y_cl']]))
        ocp.set_value(waypoints, waypoints_num)

        # # Lagrange objective term
        # # theta = plt.array([total_acc, lateral_acc, total_jerk, lat_jerk, curvature, speed_feature, lane_change_feature])
        # # ocp.add_objective(theta[0, 0]* ocp.integral((ax ** 2 + ay ** 2)) + theta[1, 0] * ocp.integral(ay ** 2) + theta[2, 0] * ocp.integral((jx ** 2 + jy ** 2)) + theta[3, 0] * ocp.integral(jy ** 2) + theta[4, 0] * ocp.integral((vx * ay - vy * ax) ** 2 / (vx ** 2 + vy ** 2) ** 3) + theta[5, 0] * ocp.integral((desired_speed - vx) ** 2) + theta[6, 0] * ocp.integral((delta_lane - y) ** 2))
        # ocp.add_objective(theta[0, 0] * 1 / f_obs[0, 0] * ocp.integral(ax ** 2 + ay ** 2) + theta[1, 0] * 1 / f_obs[1, 0] * ocp.integral(ay ** 2) + theta[2, 0] * 1 / f_obs[2, 0] * ocp.integral(jx ** 2 + jy ** 2) + theta[3, 0] * 1 / f_obs[3, 0] * ocp.integral(jy ** 2) + theta[4, 0] * 1 / f_obs[4, 0] * ocp.integral((vx * ay - vy * ax) ** 2 / (vx ** 2 + vy ** 2) ** 3) + theta[5, 0] * 1 / f_obs[5, 0] * ocp.integral((desired_speed - vx) ** 2)+theta[6, 0] * 1 / f_obs[6, 0]  * ocp.integral((delta_lane - y) ** 2))
        # # THE VX FEATURE IS REMOVED!!
        # # ocp.add_objective(theta[0, 0] * 1 / f_obs[0, 0] * ocp.integral((ax ** 2 + ay ** 2)) + theta[1, 0] * 1 / f_obs[1, 0] * ocp.integral(ay ** 2) + theta[2, 0] * 1 / f_obs[2, 0] * ocp.integral((jx ** 2 + jy ** 2)) + theta[3, 0] * 1 / f_obs[3, 0] * ocp.integral(jy ** 2) + theta[4, 0] * 1 / f_obs[4, 0] * ocp.integral((vx * ay - vy * ax) ** 2 / (vx ** 2 + vy ** 2) ** 3)+theta[6, 0] * 1 / f_obs[6, 0]  * ocp.integral((delta_lane - y) ** 2))

        # ocp.subject_to(-1 <= (y <= 4.5))
        # # ocp.subject_to(-1 <= (u <= 1 ))
        #
        # # Boundary on the max curvature of the vehicle
        # curv = (vx * ay - vy * ax) / (vx ** 2 + vy ** 2) ** (3 / 2)
        # ocp.subject_to(-0.030 <= (curv <= 0.030))
        # ocp.subject_to(-2 <= (ax <= 2))
        # ocp.subject_to(-20 <= (ay <= 20))
        # ocp.subject_to(-10 <= (jx <= 10))
        # ocp.subject_to(-60 <= (jy <= 60))


        # Boundary constraints
        ocp.subject_to(ocp.at_t0(x) == init_matrix[k,0])
        ocp.subject_to(ocp.at_t0(vx) == init_matrix[k,1])
        ocp.subject_to(ocp.at_t0(psi) == init_matrix[k,2])
        ocp.subject_to(ocp.at_t0(dpsi) == init_matrix[k,3])
        ocp.subject_to(ocp.at_t0(y) == init_matrix[k,4])
        ocp.subject_to(ocp.at_t0(vy) == init_matrix[k,5])


        # ocp.subject_to(ocp.at_tf(y) == delta_lane)
        # ocp.subject_to(ocp.at_tf(vy) == 0)
        # ocp.subject_to(ocp.at_tf(ay) == 0)
        # ocp.subject_to(ocp.at_tf(jy) == 0)

        # DIT MOET NU WEL AANGEZET OMDAT NU GEEN SOFT CONSTRAINT MEER
        # ocp.subject_to(ocp.at_tf(vx) == des_matrix[k,1])

        # Guess the solution
        ####################
        [x_guess, y_guess, vx_guess, vy_guess, ax_guess,ay_guess,jx_guess, jy_guess] = guess_states(dict_list[k],desired_speed,CP)

        # Can improve to give here a guess for the yaw and the yaw_rate
        ocp.set_initial(x,x_guess)
        ocp.set_initial(vx,vx_guess)

        ocp.set_initial(y,y_guess)
        ocp.set_initial(vy,vy_guess)

        # Solving the problem
        # -------------------

        # Pick an NLP solver backend
        #  (CasADi `nlpsol` plugin):
        ocp.solver('ipopt')

        # Pick a solution method
        #  N -- number of control intervals
        #  M -- number of integration steps per control interval
        method = MultipleShooting(N=CP, M=IP, intg='rk')
        #method = DirectCollocation(N=10, M=2)
        ocp.method(method)

        # Solve
        sol = ocp.solve()

        # Post-processing
        #################

        # Sampeling the optimal solutions

        # theta(t)/theta(t)/s
        tpsi_i, psi_i = sol.sample(psi, grid='integrator')
        tdpsi_i, dpsi_i = sol.sample(dpsi, grid='integrator')

        # X(t)/Y(t)
        tx_i, x_i = sol.sample(x, grid='integrator')
        ty_i, y_i = sol.sample(y, grid='integrator')

        # Vx(t)/Vy(t)
        tvx_i, vx_i_loc = sol.sample(vx, grid='integrator')
        tvy_i, vy_i_loc = sol.sample(vy, grid='integrator')

        vx_i = plt.cos(psi_i) * vx_i_loc - plt.sin(psi_i) * vy_i_loc
        vy_i = plt.sin(psi_i) * vx_i_loc + plt.cos(psi_i) * vy_i_loc

        # Calcultion of Ax(t)/Ay(t)
        tax_i, ax_i_loc = sol.sample(ax_con, grid='integrator')
        tay_i, ay_i_loc = sol.sample(ay_con, grid='integrator')

        # Calcultion of Ax(t)/Ay(t)
        ax_loc_tot = ax_i_loc - dpsi_i * vy_i_loc
        ay_loc_tot = ay_i_loc + dpsi_i * vx_i_loc

        ax_i = plt.cos(psi_i) * ax_loc_tot - plt.sin(psi_i) * ay_loc_tot
        ay_i = plt.sin(psi_i) * ax_loc_tot + plt.cos(psi_i) * ay_loc_tot

        # Calcultion of Jx(t)/Jy(t)
        [jy_i, jx_i] = jerk_vehicle_model(tx_i, ax_i, ay_i, 1)
        jx_i = plt.squeeze(jx_i)
        jy_i = plt.squeeze(jy_i)

        # curvature with global axis information
        curv = (vx_i * ay_i -  vy_i* ax_i) / (vx_i** 2 + vy_i** 2) ** (3 / 2)

        # storage of calculated path
        his_x[k,:] = x_i
        his_vx[k,:] = vx_i
        his_ax[k,:] = ax_i
        his_jx[k,:] = jx_i

        his_y[k,:] = y_i
        his_vy[k,:] = vy_i
        his_ay[k,:] = ay_i
        his_jy[k,:] = jy_i

        his_time_cal_lc[k,0] = tx_i[-1]
        his_time_cal_lc[k, 1] = tx_i[1] - tx_i[0]

        # Plotting in figures

        # Plotting over iterations --> only for the first dataset!
        if k == 0:
            axcom1a.plot(tx_i, x_i, '.-', label="iteration: "+theta_iter, linewidth=3.0)
            axcom1b.plot(ty_i, y_i, '.-', label="iteration: "+theta_iter, linewidth=3.0)
            axcom2.plot(x_i, y_i, '.-',   label="iteration: "+theta_iter, linewidth=3.0)
            axcom3a.plot(tvx_i, vx_i, '.-', label="iteration: "+theta_iter, linewidth=3.0)
            axcom3b.plot(tvy_i, vy_i, '.-', label="iteration: "+theta_iter, linewidth=3.0)
            axcom4a.plot(tax_i, ax_i, '.-', label="iteration: "+theta_iter, linewidth=3.0)
            axcom4b.plot(tay_i, ay_i, '.-', label="iteration: "+theta_iter, linewidth=3.0)
            axcom5a.plot(tx_i, jx_i, '.-', label="iteration: "+theta_iter, linewidth=3.0)
            axcom5b.plot(ty_i, jy_i, '.-', label="iteration: "+theta_iter, linewidth=3.0)
            axcom7.plot(tx_i, curv, '.-',   label="iteration: "+theta_iter, linewidth=3.0)

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
            axcom7.legend()

        # Plotting per iteration
        if plot == 1:

            # states
            ax1a.plot(tx_i, x_i, '.-',label = files[k][10:],linewidth = 3.0)
            ax1b.plot(ty_i, y_i, '.-',label = files[k][10:],linewidth = 3.0)
            ax2.plot(x_i, y_i, '.-',label = files[k][10:],linewidth = 3.0)
            ax3a.plot(tvx_i, vx_i, '.-',label = files[k][10:],linewidth = 3.0)
            ax3b.plot(tvy_i, vy_i, '.-',label = files[k][10:],linewidth = 3.0)
            ax4a.plot(tax_i, ax_i, '.-',label = files[k][10:],linewidth = 3.0)
            ax4b.plot(tay_i, ay_i, '.-',label = files[k][10:],linewidth = 3.0)
            ax5a.plot(tx_i, jx_i, '.-',label = files[k][10:],linewidth = 3.0)
            ax5b.plot(ty_i, jy_i, '.-',label = files[k][10:],linewidth = 3.0)

            ax8.plot(tx_i, curv, '.-', label=files[k][10:], linewidth=3.0)

            ax1a.legend()
            ax1b.legend()
            ax2.legend()
            ax3a.legend()
            ax3b.legend()
            ax4a.legend()
            ax4b.legend()
            ax5a.legend()
            ax5b.legend()
            ax6a.legend()
            ax6b.legend()
            ax8.legend()


    return his_x, his_vx, his_ax, his_jx, his_y, his_vy, his_ay, his_jy, his_time_cal_lc