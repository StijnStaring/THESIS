"""
stijnstaring@hotmail.com

The program automatically uses all the csv files stored in the same folder.
"""
import pylab as plt
from import_data import import_data
from optim_weights import optim_weights
from calc_features import calc_features

# Remarks
# ########
# 21-01-20
# Need also to use a yaw angle in the point model --> bad path planning if don't know the orientation of the vehicle.
# Best to follow thesis lay out of example thesis --> know how to start.
# Waarom is het nodig om zo een simpel point model te gebruiken? Het is toch een offline berekening --> kan ook
# een complexer model gebruiken?
#
# Defining weights
##################
his_f_diff_perc = []

amount_features = 7
rec = 1
plot_datasets = 1
alpha = 1 # learing rate between 0 and 1
# theta = plt.array([total_acc, lateral_acc, total_jerk, lat_jerk, curvature, speed_feature, lane_change_feature])
theta = plt.ones((amount_features,1))
theta = theta / plt.linalg.norm(theta, ord=2)

# Define plot to compare features
plt.figure("Comparing Features")
axf = plt.gca()
plt.xlabel("Feature number", fontsize=14)
plt.ylabel("Feature value", fontsize=14)
plt.title("Comparing Features", fontsize=14)
plt.grid(True)

[f1_o,f2_o,f3_o,f4_o,f5_o,f6_o,f7_o,init_matrix,des_matrix,dict_list,files] = import_data(plot_datasets)
axf.plot([f1_o,f2_o,f3_o,f4_o,f5_o,f6_o,f7_o],'-', marker='o', markersize=6, label = "Observed features")

########### Dit kan binnen de for loop gezet worden --> overbodig

[his_x, his_vx, his_ax, his_jx, his_y, his_vy, his_ay, his_jy,his_time_cal_lc] = optim_weights(theta,init_matrix,des_matrix,dict_list,files,str(rec),0)
[f1,f2,f3,f4,f5,f6,f7] = calc_features(his_x, his_vx, his_ax, his_jx, his_y, his_vy, his_ay, his_jy,his_time_cal_lc,des_matrix)
axf.plot([f1,f2,f3,f4,f5,f6,f7],'-', marker='o', markersize=6,label = "Calc Features iter: " + str(rec))
axf.legend()

# update theta
f_obs = plt.array([f1_o,f2_o,f3_o,f4_o,f5_o,f6_o,f7_o])
f_calc = plt.array([f1,f2,f3,f4,f5,f6,f7])
f_obs_perc = f_obs/sum(f_obs)
f_calc_perc = f_calc/sum(f_calc)
f_diff_perc = f_calc_perc-f_obs_perc
his_f_diff_perc.append(f_diff_perc) #store the progress of the algorithm

delta_theta = plt.zeros((amount_features,1))
# theta_norm = 1
for i in plt.arange(0,amount_features,1):
    if f_diff_perc[i]>0:
        delta_theta[i] = delta_theta[i]+f_diff_perc[i]
    elif f_diff_perc[i]<0:
        for k in plt.arange(0,len(theta),1):
            if k!= i:
                delta_theta[k] = delta_theta[k] - f_diff_perc[i]

theta = theta + alpha*delta_theta
theta = theta / plt.linalg.norm(theta, ord=2)


# Optimization loop
# while plt.linalg.norm(delta_theta, ord=2) > 0.1:
# Change this to convergence of feature calc array
while rec < 9:
    rec = rec + 1
    [his_x, his_vx, his_ax, his_jx, his_y, his_vy, his_ay, his_jy, his_time_cal_lc] = optim_weights(theta, init_matrix,des_matrix,dict_list, files,str(rec),0)
    [f1, f2, f3, f4, f5, f6, f7] = calc_features(his_x, his_vx, his_ax, his_jx, his_y, his_vy, his_ay, his_jy,his_time_cal_lc, des_matrix)
    axf.plot([f1,f2,f3,f4,f5,f6,f7],'-', marker='o', markersize=6,label = "Calc Features iter: " + str(rec))
    axf.legend()

    # Update theta
    f_calc = plt.array([f1, f2, f3, f4, f5, f6, f7])
    f_calc_perc = f_calc / sum(f_calc)
    f_diff_perc = f_calc_perc - f_obs_perc
    his_f_diff_perc.append(f_diff_perc)  # store the progress of the algorithm

    delta_theta = plt.zeros((amount_features, 1))
    # theta_norm = 1

    for i in plt.arange(0, amount_features, 1):
        if f_diff_perc[i] > 0:
            delta_theta[i] = delta_theta[i] + f_diff_perc[i]
        elif f_diff_perc[i] < 0:
            for k in plt.arange(0, len(theta), 1):
                if k != i:
                    delta_theta[k] = delta_theta[k] - f_diff_perc[i]

    theta = theta + alpha * delta_theta
    theta = theta / plt.linalg.norm(theta, ord=2)

print('The optimal weights are: ', theta)

plt.figure("Convergens algorithm")
axs = plt.gca()
plt.xlabel("iteration [-]", fontsize=14)
plt.ylabel("Feature difference", fontsize=14)
plt.grid(True)
plt.title('Convergence', fontsize=14)

his_f1_diff_perc = plt.zeros((len(his_f_diff_perc)))
his_f2_diff_perc = plt.zeros((len(his_f_diff_perc)))
his_f3_diff_perc = plt.zeros((len(his_f_diff_perc)))
his_f4_diff_perc = plt.zeros((len(his_f_diff_perc)))
his_f5_diff_perc = plt.zeros((len(his_f_diff_perc)))
his_f6_diff_perc = plt.zeros((len(his_f_diff_perc)))
his_f7_diff_perc = plt.zeros((len(his_f_diff_perc)))

lst = []
for i in plt.arange(0,len(his_f_diff_perc),1):
    lst.append(plt.linalg.norm(his_f_diff_perc[i], ord=2))
    his_f1_diff_perc[i] = his_f_diff_perc[i][0]
    his_f2_diff_perc[i] = his_f_diff_perc[i][1]
    his_f3_diff_perc[i] = his_f_diff_perc[i][2]
    his_f4_diff_perc[i] = his_f_diff_perc[i][3]
    his_f5_diff_perc[i] = his_f_diff_perc[i][4]
    his_f6_diff_perc[i] = his_f_diff_perc[i][5]
    his_f7_diff_perc[i] = his_f_diff_perc[i][6]

axs.plot(lst,'o',markersize=6,label = "feature difference total")
axs.plot(his_f1_diff_perc,'o',markersize=6,label = "feature difference f1")
axs.plot(his_f2_diff_perc,'o',markersize=6,label = "feature difference f2")
axs.plot(his_f3_diff_perc,'o',markersize=6,label = "feature difference f3")
axs.plot(his_f4_diff_perc,'o',markersize=6,label = "feature difference f4")
axs.plot(his_f5_diff_perc,'o',markersize=6,label = "feature difference f5")
axs.plot(his_f6_diff_perc,'o',markersize=6,label = "feature difference f6")
axs.plot(his_f7_diff_perc,'o',markersize=6,label = "feature difference f7")

axs.legend()
plt.show()
