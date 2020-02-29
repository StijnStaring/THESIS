"""
stijnstaring@hotmail.com
"""
# Importations
# local functions
from define_plots import define_plots
from guess_states import guess_states
# global functions
import pylab as plt
from rockit import *
# init_matrix = [x,vx,ax,jx,y,vy,ay,jy]
# des_matrix = [delta_lane, desired speed, time_lane_change]
def optim_weights(theta,init_matrix,des_matrix,dict_list,files,theta_iter,plot,f_obs,axcom1a,axcom1b,axcom2,axcom3a,axcom3b,axcom4a,axcom4b,axcom5a,axcom5b,axcom6a,axcom6b,axcom7):

    # Opti variables
    ################
    # 200 samples --> manoeuvre +- 4 s --> dt = 0.02s --> +- 0.5 m per sample

    CP = 50
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
        # THE TIME IS SET FIXED!!!
        # ocp = Ocp(T=FreeTime(des_matrix[k,2]))
        ocp = Ocp(T=des_matrix[k,2])

        # States (global axis) --> path is 5th order as in paper
        x = ocp.state()
        y = ocp.state()
        vx = ocp.state()
        vy = ocp.state()
        ax = ocp.state()
        ay = ocp.state()
        jx = ocp.state()
        jy = ocp.state()

        # Controls --> jounce need to be control because want jerk a continuous function. (is second order)
        ux= ocp.control(order= 1)
        uy= ocp.control(order= 1)

        # Specify differential equations for states
        ocp.set_der(jx, ux)
        ocp.set_der(jy, uy)
        ocp.set_der(ax, jx)
        ocp.set_der(ay, jy)
        ocp.set_der(vx, ax)
        ocp.set_der(vy, ay)
        ocp.set_der(x, vx)
        ocp.set_der(y, vy)

        delta_lane = des_matrix[k, 0]
        desired_speed = des_matrix[k,1]


        # # Lagrange objective term
        # # theta = plt.array([total_acc, lateral_acc, total_jerk, lat_jerk, curvature, speed_feature, lane_change_feature])
        # # ocp.add_objective(theta[0, 0]* ocp.integral((ax ** 2 + ay ** 2)) + theta[1, 0] * ocp.integral(ay ** 2) + theta[2, 0] * ocp.integral((jx ** 2 + jy ** 2)) + theta[3, 0] * ocp.integral(jy ** 2) + theta[4, 0] * ocp.integral((vx * ay - vy * ax) ** 2 / (vx ** 2 + vy ** 2) ** 3) + theta[5, 0] * ocp.integral((desired_speed - vx) ** 2) + theta[6, 0] * ocp.integral((delta_lane - y) ** 2))
        # ocp.add_objective(theta[0, 0] * 1 / f_obs[0, 0] * ocp.integral(ax ** 2 + ay ** 2) + theta[1, 0] * 1 / f_obs[1, 0] * ocp.integral(ay ** 2) + theta[2, 0] * 1 / f_obs[2, 0] * ocp.integral(jx ** 2 + jy ** 2) + theta[3, 0] * 1 / f_obs[3, 0] * ocp.integral(jy ** 2) + theta[4, 0] * 1 / f_obs[4, 0] * ocp.integral((vx * ay - vy * ax) ** 2 / (vx ** 2 + vy ** 2) ** 3) + theta[5, 0] * 1 / f_obs[5, 0] * ocp.integral((desired_speed - vx) ** 2)+theta[6, 0] * 1 / f_obs[6, 0]  * ocp.integral((delta_lane - y) ** 2))
        # # THE VX FEATURE IS REMOVED!!
        # # THE Y(t) FEATURE IS REMOVED!!
        # ocp.add_objective(theta[0, 0] * 1 / f_obs[0, 0] * ocp.integral((ax ** 2 + ay ** 2)) + theta[1, 0] * 1 / f_obs[1, 0] * ocp.integral(ay ** 2) + theta[2, 0] * 1 / f_obs[2, 0] * ocp.integral((jx ** 2 + jy ** 2)) + theta[3, 0] * 1 / f_obs[3, 0] * ocp.integral(jy ** 2) + theta[4, 0] * 1 / f_obs[4, 0] * ocp.integral((vx * ay - vy * ax) ** 2 / (vx ** 2 + vy ** 2) ** 3))
        ocp.add_objective(theta[0, 0] * 1 / f_obs[0, 0] * ocp.integral((ax ** 2 + ay ** 2)) + theta[1, 0] * 1 / f_obs[1, 0] * ocp.integral(ay ** 2))


        ocp.subject_to(-1 <= (y <= 4.5))

        # Boundary on the max curvature of the vehicle
        # curv = (vx * ay - vy * ax) / (vx ** 2 + vy ** 2) ** (3 / 2)
        # ocp.subject_to(-0.030 <= (curv <= 0.030))
        ocp.subject_to(-5 <= ((vx-desired_speed) <= 5))
        ocp.subject_to(-2 <= (ax <= 2))
        ocp.subject_to(-20 <= (ay <= 20))
        ocp.subject_to(-10 <= (jx <= 10))
        ocp.subject_to(-60 <= (jy <= 60))


        # Boundary constraints
        ocp.subject_to(ocp.at_t0(x) == init_matrix[k,0])
        ocp.subject_to(ocp.at_t0(vx) == init_matrix[k,1])
        ocp.subject_to(ocp.at_t0(ax) == init_matrix[k,2])
        ocp.subject_to(ocp.at_t0(jx) == init_matrix[k,3])
        ocp.subject_to(ocp.at_t0(y) == init_matrix[k,4])
        ocp.subject_to(ocp.at_t0(vy) == init_matrix[k,5])
        ocp.subject_to(ocp.at_t0(ay) == init_matrix[k,6])
        ocp.subject_to(ocp.at_t0(jy) == init_matrix[k,7])

        ocp.subject_to(ocp.at_tf(y) == delta_lane)
        ocp.subject_to(ocp.at_tf(vy) == 0)
        ocp.subject_to(ocp.at_tf(ay) == 0)
        ocp.subject_to(ocp.at_tf(jy) == 0)

        # DIT MOET NU WEL AANGEZET OMDAT NU GEEN SOFT CONSTRAINT MEER
        ocp.subject_to(ocp.at_tf(vx) == des_matrix[k,1])

        # Guess the solution
        ####################
        [x_guess, y_guess, vx_guess, vy_guess, ax_guess,ay_guess,jx_guess, jy_guess] = guess_states(dict_list[k],desired_speed,CP)

        ocp.set_initial(x,x_guess)
        ocp.set_initial(vx,vx_guess)
        ocp.set_initial(ax,ax_guess)
        ocp.set_initial(jx,jx_guess)

        ocp.set_initial(y,y_guess)
        ocp.set_initial(vy,vy_guess)
        ocp.set_initial(ay,ay_guess)
        ocp.set_initial(jy,jy_guess)

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
        tax_i, ax_i = sol.sample(ax, grid='integrator')
        tay_i, ay_i = sol.sample(ay, grid='integrator')

        # Jx(t)/Jy(t)
        tjx_i, jx_i = sol.sample(jx, grid='integrator')
        tjy_i, jy_i = sol.sample(jy, grid='integrator')

        # Jouncex(t)/Jouncey(t)
        tjx_i, ux_i = sol.sample(ux, grid='integrator')
        tjy_i, uy_i = sol.sample(uy, grid='integrator')

        # Approximate space between samples by a quintic spline:
        # [x, vx, ax, jx, y, vy, ay, jy]  = spline_evalution(3, tx_i, x_i, vx_i, ax_i, jx_i, y_i, vy_i, ay_i, jy_i)

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
            axcom5a.plot(tjx_i, jx_i, '.-', label="iteration: "+theta_iter, linewidth=3.0)
            axcom5b.plot(tjy_i, jy_i, '.-', label="iteration: "+theta_iter, linewidth=3.0)
            axcom6a.plot(tjx_i, ux_i, '.-', label="iteration: "+theta_iter, linewidth=3.0)
            axcom6b.plot(tjy_i, uy_i, '.-', label="iteration: "+theta_iter, linewidth=3.0)
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
            ax5a.plot(tjx_i, jx_i, '.-',label = files[k][10:],linewidth = 3.0)
            ax5b.plot(tjy_i, jy_i, '.-',label = files[k][10:],linewidth = 3.0)
            ax6a.plot(tjx_i, ux_i, '.-', label=files[k][10:], linewidth=3.0)
            ax6b.plot(tjy_i, uy_i, '.-', label=files[k][10:], linewidth=3.0)

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