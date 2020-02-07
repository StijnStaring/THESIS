"""
stijnstaring@hotmail.com

The program automatically uses all the csv files stored in the 'used_data' folder.
"""
import pylab as plt
from import_data import import_data
from optim_weights import optim_weights
from calc_features import calc_features
from comparing_features import comparing_features

# Remarks
# ########
# 21-01-20
# Need also to use a yaw angle in the point model --> bad path planning if don't know the orientation of the vehicle.
# Best to follow thesis lay out of example thesis --> know how to start.
# Waarom is het nodig om zo een simpel point model te gebruiken? Het is toch een offline berekening --> kan ook
# een complexer model gebruiken?

# Defining weights
##################
his_f_diff_perc = []
his_weights = []
amount_features = 7
rec = 1
plot_datasets = 1
# alpha = 0.2*1e-5 # learing rate between 0 and 1
alpha = 0.5
# theta = plt.array([total_acc, lateral_acc, total_jerk, lat_jerk, curvature, speed_feature, lane_change_feature])
theta = plt.ones((amount_features,1))
his_weights.append([str(rec)+"//",theta])
[f1_o,f2_o,f3_o,f4_o,f5_o,f6_o,f7_o,init_matrix,des_matrix,dict_list,files] = import_data(plot_datasets)
[axf,ax7,acw] = comparing_features(str(rec),dict_list)
axf.plot([f1_o,f2_o,f3_o,f4_o,f5_o,f6_o,f7_o],'-', marker='o', markersize=6, label = "Observed features")
acw.plot([theta[0],theta[1],theta[2],theta[3],theta[4],theta[5],theta[6]],'-', marker='o', markersize=6, label = "iter " + str(rec))
f_obs = plt.array([f1_o,f2_o,f3_o,f4_o,f5_o,f6_o,f7_o])

# Optimization loop
# Change this to convergence of feature calc array
while rec < 10:
    [his_x, his_vx, his_ax, his_jx, his_y, his_vy, his_ay, his_jy, his_time_cal_lc] = optim_weights(theta, init_matrix,des_matrix,dict_list, files,str(rec),1,ax7)
    [f1, f2, f3, f4, f5, f6, f7] = calc_features(his_x, his_vx, his_ax, his_jx, his_y, his_vy, his_ay, his_jy,his_time_cal_lc, des_matrix)
    # don't plot the second iterate --> has very big feature values
    # if rec != 2:
    #     axf.plot([f1,f2,f3,f4,f5,f6,f7],'-', marker='o', markersize=6,label = "Calc Features iter: " + str(rec))
    #     axf.legend()
    axf.plot([f1,f2,f3,f4,f5,f6,f7],'-', marker='o', markersize=6,label = "Calc Features iter: " + str(rec))
    axf.legend()
    f_calc = plt.array([f1, f2, f3, f4, f5, f6, f7])
    print("********************************************************************************************")

    # Update theta
    theta = theta + alpha*(f_calc - f_obs)
    # for i in plt.arange(0,len(theta),1):
    #     if theta[i] < 0:
    #         theta[i] = 0

    rec = rec + 1
    his_weights.append([str(rec) + "//", theta])
    acw.plot([theta[0], theta[1], theta[2], theta[3], theta[4], theta[5], theta[6]], '-', marker='o', markersize=6,label="iter " + str(rec))
    acw.legend()
print("This is the history of the used weights.")
print("------------------------------------------")
print('\n')
for i in plt.arange(0,len(his_weights),1):
    print(his_weights[i])

plt.show()


# # Post-processing
# print('The optimal weights are: ', theta)
# plt.figure("Convergens algorithm")
# axs = plt.gca()
# plt.xlabel("iteration [-]", fontsize=14)
# plt.ylabel("Feature difference", fontsize=14)
# plt.grid(True)
# plt.title('Convergence', fontsize=14)
#
# his_f1_diff_perc = plt.zeros((len(his_f_diff_perc)))
# his_f2_diff_perc = plt.zeros((len(his_f_diff_perc)))
# his_f3_diff_perc = plt.zeros((len(his_f_diff_perc)))
# his_f4_diff_perc = plt.zeros((len(his_f_diff_perc)))
# his_f5_diff_perc = plt.zeros((len(his_f_diff_perc)))
# his_f6_diff_perc = plt.zeros((len(his_f_diff_perc)))
# his_f7_diff_perc = plt.zeros((len(his_f_diff_perc)))
#
# lst = []
# for i in plt.arange(0,len(his_f_diff_perc),1):
#     lst.append(plt.linalg.norm(his_f_diff_perc[i], ord=2))
#     his_f1_diff_perc[i] = his_f_diff_perc[i][0]
#     his_f2_diff_perc[i] = his_f_diff_perc[i][1]
#     his_f3_diff_perc[i] = his_f_diff_perc[i][2]
#     his_f4_diff_perc[i] = his_f_diff_perc[i][3]
#     his_f5_diff_perc[i] = his_f_diff_perc[i][4]
#     his_f6_diff_perc[i] = his_f_diff_perc[i][5]
#     his_f7_diff_perc[i] = his_f_diff_perc[i][6]
#
# axs.plot(lst,'o',markersize=6,label = "feature difference total")
# axs.plot(his_f1_diff_perc,'o',markersize=6,label = "feature difference f1")
# axs.plot(his_f2_diff_perc,'o',markersize=6,label = "feature difference f2")
# axs.plot(his_f3_diff_perc,'o',markersize=6,label = "feature difference f3")
# axs.plot(his_f4_diff_perc,'o',markersize=6,label = "feature difference f4")
# axs.plot(his_f5_diff_perc,'o',markersize=6,label = "feature difference f5")
# axs.plot(his_f6_diff_perc,'o',markersize=6,label = "feature difference f6")
# axs.plot(his_f7_diff_perc,'o',markersize=6,label = "feature difference f7")
#
# axs.legend()

