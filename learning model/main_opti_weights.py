"""
stijnstaring@hotmail.com

The program automatically uses all the csv files stored in the 'used_data' folder.
Github ==> ideal test folder
"""
# import seaborn as sns
import pylab as plt
from comparing_features import comparing_features
from post_processing_plots import post_processing_plots
from RPROP import RPROP
from import_data2 import import_data2
from optim_weights_ideal import optim_weights_ideal
import glob
# import seaborn as sns

# Remarks
# ########
# No normalization necessary with respect of gradient --> size doesn't matter in RPROP implementation.
# Optimization objective is normalized in order to have dimensionless weights --> better start (+-equal size of optimization terms at start)

# Defining weights
##################
# sns.set_palette(sns.color_palette("hls", 15))
dict_sol_list = []
his_diff_theta = []
his_multi_grads = []
his_grad_current = []
his_weights = []
his_f_calc_rel = [] # procentual difference between the calculated features
amount_features = 5
rec = 1
plot_opti_weights = 1
N = 500 # amount of data points
# width_road = 3.46990715
# vx_start = 23.10159175
# time_guess = 4.01

# RPROP variables
del_0 = 0.1
exception = plt.zeros([amount_features,1])
del_theta_prev = plt.zeros([amount_features,1])
grad_curr = plt.zeros([amount_features,1])
grad_prev = plt.zeros([amount_features,1])
update = del_0*plt.ones([amount_features,1])
#################
# Comfort cost function: ax**2+t1*ay**2+t2*jy**2+t3*(vx-vdes)**2+t4*(y-ydes)**2
# theta = plt.array([[1.5],[1.5],[1.5],[1.5],[1.5]])
# theta = plt.array([[1.0],[1.0],[1.0],[1.0],[1.0]])
# theta = plt.array([[5.49749001e+02], [1.89525204e+00], [5.31749963e-01], [2.14306117e+01],[1.16706616e-01]])
theta = plt.array([[551], [1.93], [0.55], [23], [0.15]])

theta_tracker = []
theta_tracker.append(theta)
# theta = 1.5*plt.ones((amount_features,1))
his_weights.append([str(rec)+"//",theta])

# Import data
file_list = glob.glob("used_data/*.csv")
for file in file_list:
    print("The name of the file: ", file)
    [data_cl,f_data,width_road,vx_start] = import_data2(file,1)

    # theta = plt.array([4,5,6,1,2]) => goal

    # plotting
    [axf,acw, axfn, axcom1a,axcom1b,axcom2,axcom3a,axcom3b,axcom4a,axcom4b,axcom5a,axcom5b,axcom6a,axcom6b,axcom7a,axcom7b,axcom8a,axcom8b] = comparing_features(data_cl)
    axf.plot([1,1,1,1,1],'-', marker='*', markersize=6, label = "Observed features")
    axfn.plot([f_data[0],f_data[1],f_data[2],f_data[3],f_data[4]],'-', marker='*', markersize=6, label = "Observed features")
    acw.plot([theta[0],theta[1],theta[2],theta[3],theta[4]],'-', marker='o', markersize=6, label = "iter " + str(rec))
    ###########

    # Optimization loop
    # Change this to convergence criterium: df/dt = 0 or weights are accurately found.
    theta_chosen = plt.array([5.49749001e+02, 1.89525204e+00, 5.31749963e-01, 2.14306117e+01, 1.16706616e-01])

    # theta_chosen = plt.array([4,5,6,1,2])
    theta_chosen = theta_chosen[:,plt.newaxis]
    f_calc_rel = plt.array([[100],[100],[100],[100],[100]]) # just start value
    # while plt.sum(plt.absolute(grad_curr)) < 0.5:
    # while plt.sum(plt.absolute(theta_chosen - theta)) > 0.5:

    while any(plt.absolute(f_calc_rel-1) >= 1e-4):
        print('rec is: ',rec)
        plotting_calc = 0
        [data_s, f_calc] = optim_weights_ideal(theta,rec,N,plotting_calc,axcom1a,axcom1b,axcom2,axcom3a,axcom3b,axcom4a,axcom4b,axcom5a,axcom5b,axcom6a,axcom6b,axcom7a,axcom7b,axcom8a,axcom8b,file)
        dict_sol_list.append(data_s)
        # Normalization for plots
        f_calc_rel = f_calc/f_data
        his_f_calc_rel.append([str(rec) + "//", f_calc_rel])
        print("********************************************************************************************")

        # Update theta with RPROP
        ##########################
        grad_curr = f_data-f_calc
        print('This is the difference of theta: ', theta_chosen - theta)
        his_diff_theta.append([str(rec) + "//", theta_chosen - theta])
        print('This is grad current: ', plt.sum(plt.absolute(grad_curr)))
        axf.plot([f_calc_rel[0], f_calc_rel[1], f_calc_rel[2], f_calc_rel[3], f_calc_rel[4]], '-', marker='o', markersize=6,label="Calc Features iter: " + str(rec))
        axf.legend()
        axfn.plot([f_calc[0], f_calc[1], f_calc[2], f_calc[3], f_calc[4]], '-', marker='o', markersize=6,label="Calc Features iter: " + str(rec))
        axfn.legend()
        his_grad_current.append([str(rec) + "//", grad_curr])

        if any(plt.absolute(f_calc_rel - 1) >= 1e-4):
            [grad_prev, del_theta_prev, exception, theta, update, multi_grads] = RPROP(grad_curr,grad_prev,update,theta,del_theta_prev,exception)
            his_multi_grads.append([str(rec)+"//",multi_grads])
            rec = rec + 1
            his_weights.append([str(rec) + "//", theta])
            acw.plot([theta[0], theta[1], theta[2], theta[3], theta[4]], '-', marker='o', markersize=6,label="iter " + str(rec))
            acw.legend()

        ###########################

    # Post - processing
    theta_tracker.append(theta)

    print("This is the history of his_multi_grads.")
    print("------------------------------------------")
    print('\n')
    for i in plt.arange(0,len(his_multi_grads),1):
        print(his_multi_grads[i])

    print("This is the history of current_grads.")
    print("------------------------------------------")
    print('\n')
    for i in plt.arange(0,len(his_grad_current),1):
        print(his_grad_current[i])

    print("This is the history of f_calc_rel.")
    print("------------------------------------------")
    print('\n')
    for i in plt.arange(0,len(his_f_calc_rel),1):
        print(his_f_calc_rel[i])

    print("This is the history of the used weights.")
    print("------------------------------------------")
    print('\n')
    for i in plt.arange(0,len(his_weights),1):
        print(his_weights[i])

    print("This is the history of the update of the weights.")
    print("----------------------------------------------------")
    print('\n')
    if len(his_weights) != 1:
        for i in plt.arange(1,len(his_weights),1):
            print("This is update " + str(i))
            print(his_weights[i][1] - his_weights[i-1][1])

    post_processing_plots(his_f_calc_rel,his_weights,his_multi_grads,his_grad_current,his_diff_theta)

    # # Plotting end solution in comparinson
    data_s = dict_sol_list[-1]
    x_sol = data_s['x_s']
    y_sol = data_s['y_s']
    vx_sol = data_s['vx_s']
    vy_sol = data_s['vy_s']
    psi_sol = data_s['psi_s']
    psi_dot_sol = data_s['psi_dot_s']
    throttle_sol = data_s['throttle_s']
    delta_sol = data_s['delta_s']
    T_sol = data_s['T_s']
    dt_sol = data_s['dt_s']
    ax_tot_sol = data_s['ax_tot_s']
    ay_tot_sol = data_s['ay_tot_s']
    aty_sol = data_s['aty_s']
    any_sol = data_s['any_s']
    jx_sol = data_s['jx_s']
    jy_sol = data_s['jy_s']

    time_vector = plt.linspace(0, T_sol, len(x_sol))
    axcom1a.plot(time_vector, x_sol, '.-', linewidth=3.0, label="learned solution")
    axcom1b.plot(time_vector, y_sol, '.-', linewidth=3.0, label="learned solution")
    axcom2.plot(x_sol, y_sol, '.-', linewidth=3.0, label="learned solution")
    axcom3a.plot(time_vector, vx_sol, '.-', linewidth=3.0, label="learned solution")
    axcom3b.plot(time_vector, vy_sol, '.-', linewidth=3.0, label="learned solution")
    axcom4a.plot(time_vector, ax_tot_sol, '.-', linewidth=3.0, label="learned solution")
    axcom4b.plot(time_vector, ay_tot_sol, '.-', linewidth=3.0, label="learned solution")
    axcom5a.plot(time_vector, jx_sol, '.-', linewidth=3.0, label="learned solution")
    axcom5b.plot(time_vector, jy_sol, '.-', linewidth=3.0, label="learned solution")
    axcom6a.plot(time_vector, psi_sol * 180 / plt.pi, '.-', linewidth=3.0, label="learned solution")
    axcom6b.plot(time_vector, psi_dot_sol * 180 / plt.pi, '.-', linewidth=3.0, label="learned solution")
    axcom7a.plot(time_vector[0:-1], throttle_sol, '.-', linewidth=3.0, label="learned solution")
    axcom7b.plot(time_vector[0:-1], delta_sol * 180 / plt.pi, '.-', linewidth=3.0, label="learned solution")
    axcom8a.plot(time_vector, aty_sol, '.-', linewidth=3.0, label="learned solution")
    axcom8b.plot(time_vector, any_sol, '.-', linewidth=3.0, label="learned solution")

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
    axcom7a.legend()
    axcom7b.legend()
    axcom8a.legend()
    axcom8b.legend()
print('This is the theta_tracker: ',theta_tracker)
plt.show()
#####################
