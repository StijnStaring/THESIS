"""
stijnstaring@hotmail.com

The program automatically uses all the csv files stored in the 'used_data' folder.
Github stijn staring for more information
"""
import pylab as plt
import csv
from comparing_features import comparing_features
from post_processing_plots import post_processing_plots
from RPROP import RPROP
from figure_style_saving import figure_style_saving
from import_data2 import import_data2
from optim_weights_ideal import optim_weights_ideal
import glob

# Remarks
# ########
# No normalization necessary of gradient --> size doesn't matter in RPROP implementation.
# Optimization objective is normalized in order to have dimensionless weights --> better start (+-equal size of optimization terms at start)

# Defining weights
##################
his_diff_theta = []
his_multi_grads = []
his_grad_current = []
his_weights = []
his_exception = []
his_update = []
his_del_theta_prev = []
his_f_calc_rel = [] # procentual difference between the calculated features
amount_features = 6
rec = 1
N = 1000
tol = 1e-3
# Comfort cost function: ax**2+t1*ay**2+t2*jx**2+t3*jy**2+t4*(vx-vdes)**2+t5*(y-ydes)**2
# theta = plt.array([1.0,1.0,1.0,1.0,1.0,1.0])
theta = plt.array([4.0,5.0,1.0,6.0,1.0,2.0])
# RPROP variables
del_0 = 0.1
exception = plt.zeros([amount_features])
del_theta_prev = plt.zeros([amount_features])
update = del_0*plt.ones([amount_features])

# Plotting
######################
plotting_calc = 0
axcom1a =0; axcom1b=0; axcom2=0; axcom3a=0;axcom3b=0;axcom4a=0;axcom4b=0;axcom5a=0;axcom5b=0
axcom6a=0;axcom6b=0;axcom7a=0;axcom7b=0;axcom8a=0;axcom8b=0;axcom9a=0;axcom9b=0;axcom10=0;axcom11a=0;axcom11b=0;axf=0;axfn=0;acw=0

# Other
######################
theta_tracker = []
theta_tracker.append(theta)
his_weights.append([str(rec) + "//", theta[:,plt.newaxis]])
theta_chosen = plt.array([4.0,5.0,1.0,6.0,1.0,2.0]) # This is the theta used to generate the data
data_list = []
file_list_extern = glob.glob("used_data/*.csv")
for m in plt.arange(0,len(file_list_extern),1):
    file_list = []
    file_list.append(file_list_extern[m])

    for file in file_list:
        print("\n")
        print("A new dataset is selected!")
        print('*******************************')
        print("The name of the file: ", file)
        data_cl = import_data2(file, 1)
        data_list.append(data_cl)
        [axf, acw, axfn, axcom1a, axcom1b, axcom2, axcom3a, axcom3b, axcom4a, axcom4b, axcom5a, axcom5b, axcom6a, axcom6b,axcom7a, axcom7b, axcom8a, axcom8b, axcom9a,axcom9b,axcom10,axcom11a,axcom11b] = comparing_features(data_cl,file)
        # plotting
        # axf.plot([1, 1, 1, 1, 1, 1], '-', marker='*', markersize=6, label = file[16:-4])
        # axfn.plot([data_cl['features'][0], data_cl['features'][1], data_cl['features'][2], data_cl['features'][3], data_cl['features'][4], data_cl['features'][5]], '-', marker='*', markersize=6,label = file[15:-4])
        # acw.plot([theta[0], theta[1], theta[2], theta[3], theta[4], theta[5]], '-', marker='*', markersize=6)

    # Calculate the averaged
    av_features_data = plt.zeros(amount_features)
    for i in plt.arange(0,len(file_list),1):
        av_features_data = av_features_data + data_list[i]['features']
    av_features_data = av_features_data/len(file_list)

    solutions = []
    converged = 0
    grad_prev = plt.zeros([amount_features])

    # while rec <= 1:
    while converged != 1 and rec <= 300:
        converged = 0
        dict_sol_list = []
        print('Iteration: ', rec)
        diff_theta = theta_chosen - theta
        print('This is the difference of theta: ', diff_theta[:,plt.newaxis])
        his_diff_theta.append([str(rec) + "//", diff_theta[:,plt.newaxis]])

        for k in range(len(file_list)):
            file = file_list[k]
            curr_data = data_list[k]
            [data_s, f_calc,lambda_sol] = optim_weights_ideal(theta,curr_data,rec,N,plotting_calc,axcom1a,axcom1b,axcom2,axcom3a,axcom3b,axcom4a,axcom4b,axcom5a,axcom5b,axcom6a,axcom6b,axcom7a,axcom7b,axcom8a, axcom8b, axcom9a,axcom9b,axcom10,axcom11a,axcom11b,file)
            data_list[k]['lam_sol'] = lambda_sol
            dict_sol_list.append(data_s)

        # Calculating averaged calculated solution
        av_features_calc = plt.zeros(amount_features)
        for i in plt.arange(0, len(file_list), 1):
            av_features_calc = av_features_calc + dict_sol_list[i]['features']
        av_features_calc = av_features_calc / len(file_list)
        print('av_features_data: ',av_features_data)
        print('av_features_calc: ', av_features_calc)

        # Normalization for plots
        f_calc_rel = av_features_calc/av_features_data
        grad_curr = av_features_data - av_features_calc

        print('This is summed grad current: ', plt.sum(plt.absolute(grad_curr)))
        print('\n')
        axf.plot([f_calc_rel[0], f_calc_rel[1], f_calc_rel[2], f_calc_rel[3], f_calc_rel[4], f_calc_rel[5]], '-', marker='o', markersize=6)
        axfn.plot([av_features_calc[0], av_features_calc[1], av_features_calc[2], av_features_calc[3], av_features_calc[4],av_features_calc[4]], '-', marker='o', markersize=6)

        print("----------------------------------------------")
        print('This is f_calc_rel: ')
        print(str(rec) + "// ", f_calc_rel)

        his_f_calc_rel.append([str(rec) + "//", f_calc_rel[:,plt.newaxis]])
        his_grad_current.append([str(rec) + "//", grad_curr[:,plt.newaxis]])

        # Problem was: exception is specified as an specific global array (not for integers) and added each time to a list. When this global variable itself is updated --> changes all values in list because all point to the same global variable.
        # Can be solved by recalling the variable in the loop so that it now points to a local definition. (Again called in RPROP) Global var exception is updated by an array (local var) exception = plt.array([])..., no relation with previous value added to his_exception. Create a new point.
        # if make a list of exception and add an array and later what is on this place in exception add to his_exception and then later update this place in exception, will also change all variables in his_exception becaue they point to the same place in the excepion list.
        # In grad_curr adds two item to the list and below in code is refreshing the variable. Always adding, not pointing to the same spot. What is added is always local retreived.
        his_exception.append(exception)
        his_update.append(update)
        his_del_theta_prev.append(del_theta_prev)

        # Check if all features are converged or that the weights are not changing anymore
        if all(plt.absolute(f_calc_rel - 1) <= tol) or all(update <= 1e-4):
            converged = 1
            if all(update <= 1e-4):
                print('No change in theta detected anymore - learning terminated')
        print('Converged vector: ',converged)

        # Check what direction to go in optimization
        case = plt.ones(amount_features)
        for j in plt.arange(0,amount_features):
            print('*********')
            if exception[j] == 1 or grad_curr[j] * grad_prev[j] == 0:
                case[j] = 3
                print("case["+str(j)+")] is: ",case[j])
            elif grad_curr[j]*grad_prev[j]<0:
                case[j] = 2
                print("case["+str(j)+")] is: ", case[j])
            else:
                print("case["+str(j)+")] is: ", case[j])

        his_multi_grads.append([str(rec) + "//", case[:, plt.newaxis]])
        if converged != 1:

            [del_theta_prev, exception, theta, update] = RPROP(grad_curr,case,amount_features,update,theta,del_theta_prev,1)
            grad_prev = grad_curr
            rec = rec + 1
            his_weights.append([str(rec) + "//", theta[:,plt.newaxis]])
            acw.plot([theta[0], theta[1], theta[2], theta[3], theta[4],theta[5]], '-', marker='o', markersize=6)

        # solutions.append([str(rec) + "//", dict_sol_list])
        solutions.append(dict_sol_list)


        print("********************************************************************************************")

    # Post - processing
    theta_tracker.append([theta, "amount of iterations: "+str(rec)])

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
    if len(his_weights) != 1: # takes current iterate minus the previous one
        for i in plt.arange(1,len(his_weights),1):
            print("This is update " + his_weights[i][0])
            diff = plt.squeeze(his_weights[i][1]) - plt.squeeze(his_weights[i-1][1])
            print(diff[:,plt.newaxis])

    # Plotting end solution in comparinson
    for k in plt.arange(0,len(file_list),1):
        file = file_list[k]
        data_s = solutions[-1][k]
        x_sol = data_s['x_s']
        y_sol = data_s['y_s']
        vx_sol = data_s['vx_s']
        vy_sol = data_s['vy_s']
        psi_sol = data_s['psi_s']
        psi_dot_sol = data_s['psi_dot_s']
        throttle_sol = data_s['throttle_s']
        delta_sol = data_s['delta_s']
        throttle_dot_sol = data_s['throttle_dot_s']
        delta_dot_sol = data_s['delta_dot_s']
        T_sol = data_s['T_s']
        dt_sol = data_s['dt_s']
        ax_tot_sol = data_s['ax_tot_s']
        ay_tot_sol = data_s['ay_tot_s']
        aty_sol = data_s['aty_s']
        any_sol = data_s['any_s']
        atx_sol = data_s['atx_s']
        anx_sol = data_s['anx_s']
        jx_tot_sol = data_s['jx_s']
        jy_tot_sol = data_s['jy_s']
        psi_ddot_sol = data_s['psi_ddot_s']

        time_vector = plt.linspace(0, T_sol, len(x_sol))
        axcom1a.plot(time_vector, x_sol, '.-', linewidth=3.0, label="LS-"+file[15:-4])
        axcom1b.plot(time_vector, y_sol, '.-', linewidth=3.0, label="LS-"+file[15:-4])
        axcom2.plot(x_sol, y_sol, '.-', linewidth=3.0, label="LS-"+file[15:-4])
        axcom3a.plot(time_vector, vx_sol, '.-', linewidth=3.0, label="LS-"+file[15:-4])
        axcom3b.plot(time_vector, vy_sol, '.-', linewidth=3.0, label="LS-"+file[15:-4])
        axcom4a.plot(time_vector, ax_tot_sol, '.-', linewidth=3.0, label="LS-"+file[15:-4])
        axcom4b.plot(time_vector, ay_tot_sol, '.-', linewidth=3.0, label="LS-"+file[15:-4])
        axcom5a.plot(time_vector, jx_tot_sol, '.-', linewidth=3.0, label="LS-"+file[15:-4])
        axcom5b.plot(time_vector, jy_tot_sol, '.-', linewidth=3.0, label="LS-"+file[15:-4])
        axcom6a.plot(time_vector, psi_sol * 180 / plt.pi, '.-', linewidth=3.0, label="LS-"+file[15:-4])
        axcom6b.plot(time_vector, psi_dot_sol * 180 / plt.pi, '.-', linewidth=3.0, label="LS-"+file[15:-4])
        axcom7a.plot(time_vector, throttle_sol, '.-', linewidth=3.0, label="LS-"+file[15:-4])
        axcom7b.plot(time_vector, delta_sol * 180 / plt.pi, '.-', linewidth=3.0, label="LS-"+file[15:-4])
        axcom8a.plot(time_vector, atx_sol, '.-', linewidth=3.0, label="LS-"+file[15:-4])
        axcom8b.plot(time_vector, anx_sol, '.-', linewidth=3.0, label="LS-"+file[15:-4])
        axcom9a.plot(time_vector, aty_sol, '.-', linewidth=3.0, label="LS-" + file[15:-4])
        axcom9b.plot(time_vector, any_sol, '.-', linewidth=3.0, label="LS-" + file[15:-4])
        axcom10.plot(time_vector, psi_ddot_sol* 180 / plt.pi, '.-', linewidth=3.0, label="LS-"+file[15:-4])
        axcom11a.plot(time_vector[0:N], throttle_dot_sol, '.-', linewidth=3.0, label="LS-" + file[15:-4])
        axcom11b.plot(time_vector[0:N], delta_dot_sol * 180 / plt.pi, '.-', linewidth=3.0, label="LS-" + file[15:-4])

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
        axcom9a.legend()
        axcom9b.legend()
        axcom10.legend()
        axcom11a.legend()
        axcom11b.legend()

        ########################
        # Writing csv file
        ########################

        path = "results\Av_It" + str(rec) + "D" + str(len(file_list)) +"F"+file[15:-4]+ ".csv"
        file = open(path, 'w', newline="")
        writer = csv.writer(file)
        writer.writerow(["time", "x", "y", "vx", "vy", "ax", "ay", "jx", "jy", "psi", "psi_dot", "psi_ddot", "throttle", "delta","throttle_dot", "delta_dot", "aty", "a_ny", "atx", "anx"])

        for i in range(N + 1):
            if i == N:  # last control point has no physical meaning
                writer.writerow([i * dt_sol, x_sol[i], y_sol[i], vx_sol[i], vy_sol[i], ax_tot_sol[i], ay_tot_sol[i], jx_tot_sol[i],jy_tot_sol[i], psi_sol[i], psi_dot_sol[i], psi_ddot_sol[i], throttle_sol[i], delta_sol[i],throttle_dot_sol[i - 1], delta_dot_sol[i - 1], aty_sol[i], any_sol[i], atx_sol[i], anx_sol[i]])
            else:
                writer.writerow([i * dt_sol, x_sol[i], y_sol[i], vx_sol[i], vy_sol[i], ax_tot_sol[i], ay_tot_sol[i], jx_tot_sol[i],jy_tot_sol[i], psi_sol[i], psi_dot_sol[i], psi_ddot_sol[i], throttle_sol[i], delta_sol[i],throttle_dot_sol[i], delta_dot_sol[i], aty_sol[i], any_sol[i], atx_sol[i], anx_sol[i]])
        file.close()
        print('\n')

    print("")
    for i in plt.arange(0,len(file_list),1):
        file = file_list[i]
        final_calculated = solutions[-1][i]['features']
        final_data = data_list[i]['features']
        print("This is f_cal_rel of file ",file[15:-4],": ", + final_calculated/final_data)
        print("")

    print('This is the theta_tracker: ',theta_tracker)

post_processing_plots(his_f_calc_rel,his_weights,his_multi_grads,his_grad_current,his_diff_theta)

#########################################
# Saving figures
#########################################
figure_style_saving()

#####################
plt.show()
#####################
