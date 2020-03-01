"""
stijnstaring@hotmail.com
"""
# Importations
# local functions
from define_plots import define_plots
from guess_states import guess_states
from calc_jerk import calc_jerk
# global functions
import pylab as plt
import math as m
from rockit import *
# init_matrix = [x,vx,ax,jx,y,vy,ay,jy]
# des_matrix = [delta_lane, desired speed, time_lane_change]
def optim_weights(theta,init_matrix,des_matrix,dict_list,files,theta_iter,plot,f_obs,axcom1a,axcom1b,axcom2,axcom3a,axcom3b,axcom4a,axcom4b,axcom5a,axcom5b,axcom6,axcom7a,axcom7b,axcom8a,axcom8b):

    # Opti variables
    ################
    CP = 70
    IP = 1
    amount = len(dict_list)
    if plot == 1:
        [ax1a,ax1b,ax2,ax3a,ax3b,ax4a,ax4b,ax5a,ax5b,ax6,ax7a,ax7b,ax8a,ax8b] = define_plots(theta_iter,dict_list)

    his_x = plt.zeros((init_matrix.shape[0],CP+1))
    his_vx = plt.zeros((init_matrix.shape[0], CP + 1))
    his_ax = plt.zeros((init_matrix.shape[0], CP + 1))
    his_jx = plt.zeros((init_matrix.shape[0], CP + 1))

    his_y = plt.zeros((init_matrix.shape[0], CP + 1))
    his_vy = plt.zeros((init_matrix.shape[0], CP + 1))
    his_ay = plt.zeros((init_matrix.shape[0], CP + 1))
    his_jy = plt.zeros((init_matrix.shape[0], CP + 1))

    his_dpsi = plt.zeros((init_matrix.shape[0], CP + 1))

    his_time_cal_lc = plt.zeros((amount,2))


    for k in plt.arange(0,amount,1):

        # Optimization
        ##############
        ocp = Ocp(T=FreeTime(des_matrix[k,2]))
        # ocp = Ocp(T=des_matrix[k,2])

         # Non - Linear Bicycle model
        ##############################

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
        x = ocp.state() #in global axis
        y = ocp.state() # in global axis
        vx = ocp.state() # in local axis
        vy = ocp.state() # in local axis
        # ax = ocp.state()  # in local axis
        # ay = ocp.state()  # in local axis
        # jx = ocp.state()  # in local axis
        # jy = ocp.state()  # in local axis
        psi = ocp.state() # yaw  angle in radials
        dpsi = ocp.state() # rate of yaw angle in radials per second

        # Controls
        throttle = ocp.control() # bounded between -1 and 1
        delta = ocp.control() # steerwheelangle in radians

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
        # ocp.set_der(vx, ax)
        # ocp.set_der(vy, ay)
        # ocp.set_der(ax, jx)
        # ocp.set_der(ay, jy)
        ocp.set_der(psi,dpsi)
        ocp.set_der(dpsi, ddpsi)

        # Extra variables - jerks in the local axis! (ook nog aanpassen bij het bereken van de features data)
        jx = ocp.der(ax_con)
        jy = ocp.der(ay_con)

        delta_lane = des_matrix[k, 0]
        desired_speed = des_matrix[k,1]

        # Lagrange objective term
        # theta = plt.array([total_acc, lateral_acc, total_jerk, lat_jerk, curvature, speed_feature, lane_change_feature])
        # ocp.add_objective(theta[0, 0]* ocp.integral((ax ** 2 + ay ** 2)) + theta[1, 0] * ocp.integral(ay ** 2) + theta[2, 0] * ocp.integral((jx ** 2 + jy ** 2)) + theta[3, 0] * ocp.integral(jy ** 2) + theta[4, 0] * ocp.integral((vx * ay - vy * ax) ** 2 / (vx ** 2 + vy ** 2) ** 3) + theta[5, 0] * ocp.integral((desired_speed - vx) ** 2) + theta[6, 0] * ocp.integral((delta_lane - y) ** 2))
        # THE VX FEATURE IS REMOVED!!
        # ocp.add_objective(theta[0, 0] * 1 / f_obs[0, 0] * ocp.integral((ax ** 2 + ay ** 2)) + theta[1, 0] * 1 / f_obs[1, 0] * ocp.integral(ay ** 2) + theta[2, 0] * 1 / f_obs[2, 0] * ocp.integral((jx ** 2 + jy ** 2)) + theta[3, 0] * 1 / f_obs[3, 0] * ocp.integral(jy ** 2) + theta[4, 0] * 1 / f_obs[4, 0] * ocp.integral((vx * ay - vy * ax) ** 2 / (vx ** 2 + vy ** 2) ** 3) + theta[5, 0] * 1 / f_obs[5, 0] * ocp.integral((desired_speed - vx) ** 2)+theta[6, 0] * 1 / f_obs[6, 0]  * ocp.integral((delta_lane - y) ** 2))
        ocp.add_objective(theta[0, 0] * 1 / f_obs[0, 0] * ocp.integral((ax_con ** 2 + ay_con ** 2)) + theta[1, 0] * 1 / f_obs[1, 0] * ocp.integral(ay_con ** 2) + theta[2, 0] * 1 / f_obs[2, 0] * ocp.integral(dpsi**2) +theta[3, 0] * 1 / f_obs[3, 0] * ocp.integral((desired_speed - vx) ** 2)+theta[4, 0] * 1 / f_obs[4, 0]  * ocp.integral((delta_lane - y) ** 2))

    # LAST FEATURE IS REMOVED - y(t) is lane change desired
    #     ocp.add_objective(theta[0, 0] *1/f_obs[0,0]* ocp.integral((ax ** 2 + ay ** 2)) + theta[1, 0] *1/f_obs[1,0]* ocp.integral(ay ** 2) + theta[2, 0] * 1/f_obs[2,0]*ocp.integral((jx ** 2 + jy ** 2)) + theta[3, 0] * 1/f_obs[3,0]*ocp.integral(jy ** 2) + theta[4, 0] * 1/f_obs[4,0]*ocp.integral((vx * ay - vy * ax) ** 2 / (vx ** 2 + vy ** 2) ** 3) + theta[5, 0] * 1/f_obs[5,0]*ocp.integral((desired_speed - vx) ** 2))

    # Path constraints
        #  (must be valid on the whole time domain running from `t0` to `tf=t0+T`,
        #   grid options available such as `grid='inf'`)

        # The same constraints as taken into the dataset (except for the throttle)
        ocp.subject_to(-1 <= (y <= 4.5 ))
        # ocp.subject_to(-1 <= (u <= 1 ))

        # Boundary on the max curvature of the vehicle
        # ax_diff = ax - ax_con
        # ay_diff = ay - ay_con
        # curv = (vx * ay - vy * ax) / (vx ** 2 + vy ** 2) ** (3/2)
        # ocp.subject_to(-0.025<=(curv <= 0.025))
        # ocp.subject_to(-1<=(ax <= 1))
        # ocp.subject_to(0 <= (ax_diff <= 0))
        # ocp.subject_to(-20 <= (ay <= 20))
        # ocp.subject_to(0 <= (ay_diff <= 0))
        # ocp.subject_to(-1<=(jx <= 10))
        # ocp.subject_to(-60<=(jy <= 40))

        # Boundary constraints
        # x,vx,y,vy,yaw,r,throttle,steering_deg
        ocp.subject_to(ocp.at_t0(x) == init_matrix[k,0])
        ocp.subject_to(ocp.at_t0(vx) == init_matrix[k,1])
        # ocp.subject_to(ocp.at_t0(ax) == init_matrix[k,2])
        # ocp.subject_to(ocp.at_t0(jx) == init_matrix[k,3])
        ocp.subject_to(ocp.at_t0(y) == init_matrix[k,2])
        ocp.subject_to(ocp.at_t0(vy) == init_matrix[k,3])
        # ocp.subject_to(ocp.at_t0(ay) == init_matrix[k,6])
        # ocp.subject_to(ocp.at_t0(jy) == init_matrix[k,7])
        ocp.subject_to(ocp.at_t0(psi) == init_matrix[k, 4])
        ocp.subject_to(ocp.at_t0(dpsi) == init_matrix[k, 5])
        ocp.subject_to(ocp.at_t0(throttle) == init_matrix[k, 6])
        ocp.subject_to(ocp.at_t0(delta) == init_matrix[k, 7]*pi/180)

        ocp.subject_to(ocp.at_tf(y) == delta_lane)
        ocp.subject_to(ocp.at_tf(vy) == 0)
        # ocp.subject_to(ocp.at_tf(ay) == 0)
        # ocp.subject_to(ocp.at_tf(jy) == 0)

        ocp.subject_to(ocp.at_tf(psi) == 0)
        ocp.subject_to(ocp.at_tf(dpsi) == 0)

        # Limits of control inputs of the vehicle.
        ocp.subject_to(-1 <= (throttle <= 1))
        ocp.subject_to(-2.618 <= (delta <= 2.618))

        # # DIT MOET NU WEL AANGEZET OMDAT NU GEEN SOFT CONSTRAINT MEER
        # ocp.subject_to(ocp.at_tf(vx) == des_matrix[k,1])

        # Guess the solution
        ####################
        [x_guess, y_guess, vx_guess, vy_guess, psi_guess, dpsi_guess ,throttle_guess, delta_guess] = guess_states(dict_list[k],CP)

        ocp.set_initial(x,x_guess)
        ocp.set_initial(vx,vx_guess)
        # ocp.set_initial(ax,ax_guess)
        # ocp.set_initial(jx,jx_guess)

        ocp.set_initial(y,y_guess)
        ocp.set_initial(vy,vy_guess)
        # ocp.set_initial(ay,ay_guess)
        # ocp.set_initial(jy,jy_guess)

        ocp.set_initial(psi, psi_guess)
        ocp.set_initial(dpsi, dpsi_guess)

        ocp.set_initial(throttle, throttle_guess)
        ocp.set_initial(delta, delta_guess)

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

        # X(t)/Y(t)
        tx_i, x_i = sol.sample(x, grid='integrator')
        ty_i, y_i = sol.sample(y, grid='integrator')

        # Vx(t)/Vy(t)
        tvx_i, vx_i = sol.sample(vx, grid='integrator')
        tvy_i, vy_i = sol.sample(vy, grid='integrator')

        # Ax(t)/Ay(t)

        tax_i, ax_i = sol.sample(ax_con, grid='integrator')
        tay_i, ay_i = sol.sample(ay_con, grid='integrator')

        # Jx(t)/Jy(t)
        dict_sol = dict()
        dict_sol['time_cl'] = tx_i[:,plt.newaxis]
        dict_sol['ax_cl'] = ax_i[:,plt.newaxis]
        dict_sol['ay_cl'] = ay_i[:,plt.newaxis]
        [jx_i,jy_i] = calc_jerk(dict_sol,0)
        jx_i = plt.squeeze(jx_i)
        jy_i = plt.squeeze(jy_i)

        # yaw(t)/r(t)
        tax_i, psi_i = sol.sample(ax_con, grid='integrator')
        tay_i, dpsi_i = sol.sample(ay_con, grid='integrator')

        # curvature with global axis information
        # projection on the global axis
        vx_gl = plt.cos(psi_i) * vx_i - plt.sin(psi_i) * vy_i
        vy_gl = plt.sin(psi_i) * vx_i + plt.cos(psi_i) * vy_i

        ax_loc_tot = ax_i - dpsi_i * vy_i
        ay_loc_tot = ay_i + dpsi_i * vx_i
        ax_gl = plt.cos(psi_i) * ax_loc_tot - plt.sin(psi_i) * ay_loc_tot
        ay_gl = plt.sin(psi_i) * ax_loc_tot + plt.cos(psi_i) * ay_loc_tot

        curv = (vx_gl * ay_gl -  vy_gl* ax_gl) / (vx_gl** 2 + vy_gl** 2) ** (3 / 2)

        # Inputs
        tax_i, throttle_i = sol.sample(throttle, grid='integrator')
        tay_i, delta_i = sol.sample(delta, grid='integrator')

    # storage of calculated path
        his_x[k,:] = x_i
        his_vx[k,:] = vx_i
        his_ax[k,:] = ax_i
        his_jx[k,:] = jx_i

        his_y[k,:] = y_i
        his_vy[k,:] = vy_i
        his_ay[k,:] = ay_i
        his_jy[k,:] = jy_i

        his_dpsi[k, :] = dpsi_i

        his_time_cal_lc[k,0] = tx_i[-1]
        his_time_cal_lc[k, 1] = tx_i[1] - tx_i[0]

        # Plotting in figures

        # Plotting over iterations
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
            axcom6.plot(tx_i, curv, '.-', label="iteration: "+theta_iter, linewidth=3.0)
            axcom7a.plot(tx_i, psi_i, '.-',   label="iteration: "+theta_iter, linewidth=3.0)
            axcom7b.plot(tx_i, dpsi_i, '.-', label="iteration: " + theta_iter, linewidth=3.0)
            axcom8a.plot(tx_i, throttle_i, '.-', label="iteration: " + theta_iter, linewidth=3.0)
            axcom8b.plot(tx_i, delta_i*180/plt.pi, '.-', label="iteration: " + theta_iter, linewidth=3.0)

            axcom1a.legend()
            axcom1b.legend()
            axcom2.legend()
            axcom3a.legend()
            axcom3b.legend()
            axcom4a.legend()
            axcom4b.legend()
            axcom5a.legend()
            axcom5b.legend()
            axcom6.legend()
            axcom7a.legend()
            axcom7b.legend()
            axcom8a.legend()
            axcom8b.legend()

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
            ax6.plot(tx_i, curv, '.-', label=files[k][10:], linewidth=3.0)
            ax7a.plot(tx_i, psi_i, '.-', label=files[k][10:], linewidth=3.0)
            ax7b.plot(tx_i, dpsi_i, '.-', label=files[k][10:], linewidth=3.0)
            ax8a.plot(tx_i, throttle_i, '.-', label=files[k][10:], linewidth=3.0)
            ax8b.plot(tx_i, delta_i*180/plt.pi, '.-', label=files[k][10:], linewidth=3.0)

            ax1a.legend()
            ax1b.legend()
            ax2.legend()
            ax3a.legend()
            ax3b.legend()
            ax4a.legend()
            ax4b.legend()
            ax5a.legend()
            ax5b.legend()
            ax6.legend()
            ax7a.legend()
            ax7b.legend()
            ax8a.legend()
            ax8b.legend()

    return his_x, his_vx, his_ax, his_jx, his_y, his_vy, his_ay, his_jy,his_dpsi, his_time_cal_lc

