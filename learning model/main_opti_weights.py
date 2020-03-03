"""
stijnstaring@hotmail.com

The program automatically uses all the csv files stored in the 'used_data' folder.
Github ==> ideal test folder
"""
# import seaborn as sns
import pylab as plt
from import_data import import_data
from optim_weights import optim_weights
from calc_features import calc_features
from comparing_features import comparing_features
from post_processing_plots import post_processing_plots
from RPROP import RPROP

# Remarks
# ########
# No normalization necessary with respect of gradient --> size doesn't matter in RPROP implementation.
# Optimization objective is normalized in order to have dimensionless weights --> better start (+-equal size of optimization terms at start)

# Defining weights
##################
# sns.set_palette(sns.color_palette("hls", 20))
his_multi_grads = []
his_weights = []
his_f_calc_rel = []
amount_features = 7
rec = 1
plot_datasets = 1
plot_opti_weights = 0
calculated_features = plt.zeros([amount_features,1])



# des_matrix = delta_lane,speed_desired, time_lane_change
[f1_o,f2_o,f3_o,f4_o,f5_o,f6_o,init_matrix,des_matrix,dict_list,files] = import_data(plot_datasets)

# RPROP variables
del_0 = 0.1
f_calculated = plt.zeros([amount_features,1])
exception = plt.zeros([amount_features,1])
del_theta_prev = plt.zeros([amount_features,1])
grad_curr = plt.zeros([amount_features,1])
grad_prev = plt.zeros([amount_features,1])
update = del_0*plt.ones([7,1])

#################

# theta = plt.array([total_acc, lateral_acc, total_jerk, lat_jerk, curvature, speed_feature, lane_change_feature])
theta = 1*plt.ones((amount_features,1))
his_weights.append([str(rec)+"//",theta])

# plotting
[axf,acw, axfn, axcom1a,axcom1b,axcom2,axcom3a,axcom3b,axcom4a,axcom4b,axcom5a,axcom5b,axcom6a,axcom6b,axcom7] = comparing_features(dict_list)
axf.plot([f1_o/f1_o,f2_o/f2_o,f3_o/f3_o,f4_o/f4_o,f5_o/f5_o,f6_o/f6_o,f7_o/f7_o],'-', marker='*', markersize=6, label = "Observed features")
axfn.plot([f1_o,f2_o,f3_o,f4_o,f5_o,f6_o,f7_o],'-', marker='*', markersize=6, label = "Observed features")
acw.plot([theta[0],theta[1],theta[2],theta[3],theta[4],theta[5],theta[6]],'-', marker='o', markersize=6, label = "iter " + str(rec))
f_obs = plt.array([f1_o,f2_o,f3_o,f4_o,f5_o,f6_o,f7_o])
f_obs = f_obs[:,plt.newaxis]
###########

# Optimization loop
# Change this to convergence of feature calc array
while rec < 20:
    print('rec is: ',rec)
    [his_x, his_vx, his_ax, his_jx, his_y, his_vy, his_ay, his_jy, his_time_cal_lc] = optim_weights(theta, init_matrix,des_matrix,dict_list, files,str(rec),plot_opti_weights,f_obs,axcom1a,axcom1b,axcom2,axcom3a,axcom3b,axcom4a,axcom4b,axcom5a,axcom5b,axcom6a,axcom6b,axcom7)
    # his_time_cal[time_lane_change, dt]
    [f1, f2, f3, f4, f5, f6, f7] = calc_features(his_x, his_vx, his_ax, his_jx, his_y, his_vy, his_ay, his_jy,his_time_cal_lc, des_matrix)

    f_calculated = plt.array([f1, f2, f3, f4, f5, f6, f7])
    f_calculated = f_calculated[:, plt.newaxis]
    # Normalization for plots
    f_calc_rel = plt.array([f1/f1_o, f2/f2_o, f3/f3_o, f4/f4_o, f5/f5_o, f6/f6_o, f7/f7_o])
    f_calc_rel = f_calc_rel[:,plt.newaxis]
    his_f_calc_rel.append([str(rec) + "//", f_calc_rel])
    print("********************************************************************************************")

    # Update theta with RPROP
    ##########################
    grad_curr = f_obs-f_calculated
    [grad_prev, del_theta_prev, exception, theta, update, multi_grads] = RPROP(grad_curr,grad_prev,update,theta,del_theta_prev,exception)
    his_multi_grads.append([str(rec)+"//",multi_grads])

    # Plots in the for loop
    axf.plot([f_calc_rel[0], f_calc_rel[1], f_calc_rel[2], f_calc_rel[3], f_calc_rel[4], f_calc_rel[5], f_calc_rel[6]],
             '-', marker='o', markersize=6, label="Calc Features iter: " + str(rec))
    axf.legend()
    calculated_features = plt.array([f1, f2, f3, f4, f5, f6, f7])

    axfn.plot([f1, f2, f3, f4, f5, f6, f7],
             '-', marker='o', markersize=6, label="Calc Features iter: " + str(rec))
    axfn.legend()
    rec = rec + 1
    his_weights.append([str(rec) + "//", theta])
    acw.plot([theta[0], theta[1], theta[2], theta[3], theta[4], theta[5], theta[6]], '-', marker='o', markersize=6,label="iter " + str(rec))
    acw.legend()
    ###########################

# Post - processing
print("This is the history of his_multi_grads.")
print("------------------------------------------")
print('\n')
for i in plt.arange(0,len(his_multi_grads),1):
    print(his_multi_grads[i])

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
for i in plt.arange(1,len(his_weights),1):
    print("This is update " +str(i))
    print(his_weights[i][1] - his_weights[i-1][1])

post_processing_plots(his_f_calc_rel,his_weights,his_multi_grads)

plt.show()
#####################
