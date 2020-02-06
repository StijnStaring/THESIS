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
def optim_weights(theta,init_matrix,des_matrix,dict_list,files,theta_iter,plot):

    # Opti variables
    ################
    CP = 50
    IP = 1
    amount = len(dict_list)
    if plot == 1:
        [ax1a,ax1b,ax2,ax3a,ax3b,ax4a,ax4b,ax5a,ax5b,ax6a,ax6b] = define_plots(theta_iter)

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
        # ocp = Ocp(T=FreeTime(des_matrix[k,2]))
        ocp = Ocp(T=des_matrix[k,2])

        # States (global axis)
        x = ocp.state()
        y = ocp.state()
        vx = ocp.state()
        vy = ocp.state()
        ax = ocp.state()
        ay = ocp.state()
        jx = ocp.state()
        jy = ocp.state()

        # Controls
        ux= ocp.control()
        uy= ocp.control()

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

        # Lagrange objective term
        # theta = plt.array([total_acc, lateral_acc, total_jerk, lat_jerk, curvature, speed_feature, lane_change_feature])
        ocp.add_objective(theta[0, 0]* ocp.integral((ax ** 2 + ay ** 2)) + theta[1, 0] * ocp.integral(ay ** 2) + theta[2, 0] * ocp.integral((jx ** 2 + jy ** 2)) + theta[3, 0] * ocp.integral(jy ** 2) + theta[4, 0] * ocp.integral((vx * ay - vy * ax) ** 2 / (vx ** 2 + vy ** 2) ** 3) + theta[5, 0] * ocp.integral((desired_speed - vx) ** 2) + theta[6, 0] * ocp.integral((delta_lane - y) ** 2))

    # Path constraints
        #  (must be valid on the whole time domain running from `t0` to `tf=t0+T`,
        #   grid options available such as `grid='inf'`)

        # The same constraints as taken into the dataset (except for the throttle)
        ocp.subject_to(-1 <= (y <= 4.5 ))
        # ocp.subject_to(-1 <= (u <= 1 ))

        # Boundary on the max curvature of the vehicle
        curv_2 = (vx * ay - vy * ax) ** 2 / (vx ** 2 + vy ** 2) ** 3
        ocp.subject_to(curv_2 <= 6e-4)

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

        # ocp.subject_to(ocp.at_tf(vx) == des_matrix[k,1])   MOET DIT NIET AAN STAAN?? --> zit al verwerkt als softconstraint in objective.

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

            # controls
            ax6a.plot(tjx_i, ux_i, '.-', label=files[k][10:], linewidth=3.0)
            ax6b.plot(tjy_i, uy_i, '.-', label=files[k][10:], linewidth=3.0)

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

    return his_x, his_vx, his_ax, his_jx, his_y, his_vy, his_ay, his_jy, his_time_cal_lc

    # plt.show(block=True)
