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
import seaborn as sns

# Remarks
# ########
# No normalization necessary with respect of gradient --> size doesn't matter in RPROP implementation.
# Optimization objective is normalized in order to have dimensionless weights --> better start (+-equal size of optimization terms at start)

# Defining weights
##################
# sns.set_palette(sns.color_palette("hls", 15))
his_multi_grads = []
his_grad_current = []
his_weights = []
his_f_calc_rel = [] # procentual difference between the calculated features
amount_features = 5
rec = 1
plot_opti_weights = 1
width_road = 3.46990715
vx_start = 23.10159175
time_guess = 4.01
# Import data
[data_cl,f_data] = import_data2(width_road,vx_start,1)
# theta = plt.array([4,5,6,1,2]) => goal

# RPROP variables
del_0 = 0.1
exception = plt.zeros([amount_features,1])
del_theta_prev = plt.zeros([amount_features,1])
grad_curr = 1000*plt.ones([amount_features,1])
grad_prev = plt.zeros([amount_features,1])
update = del_0*plt.ones([amount_features,1])
#################
# Comfort cost function: ax**2+t1*ay**2+t2*jy**2+t3*(vx-vdes)**2+t4*(y-ydes)**2
theta = plt.array([[1.5],[1.5],[1.5],[1.5],[1.5]])
# theta = 1.5*plt.ones((amount_features,1))
his_weights.append([str(rec)+"//",theta])

# plotting
[axf,acw, axfn, axcom1a,axcom1b,axcom2,axcom3a,axcom3b,axcom4a,axcom4b,axcom5a,axcom5b,axcom6a,axcom6b,axcom7a,axcom7b,axcom8a,axcom8b] = comparing_features(data_cl)
axf.plot([1,1,1,1,1],'-', marker='*', markersize=6, label = "Observed features")
axfn.plot([f_data[0],f_data[1],f_data[2],f_data[3],f_data[4]],'-', marker='*', markersize=6, label = "Observed features")
acw.plot([theta[0],theta[1],theta[2],theta[3],theta[4]],'-', marker='o', markersize=6, label = "iter " + str(rec))
###########

# Optimization loop
# Change this to convergence criterium: df/dt = 0 or weights are accurately found.
theta_chosen = plt.array([4,5,6,1,2])
theta_chosen = theta_chosen[:,plt.newaxis]
# while plt.sum(plt.absolute(grad_curr)) < 0.5:
# while plt.sum(plt.absolute(theta_chosen - theta)) > 0.5:
while rec < 30:

    print('rec is: ',rec)
    plotting_calc = 0
    [data_s, f_calc] = optim_weights_ideal(theta,width_road,vx_start,time_guess,str,500,plotting_calc,axcom1a,axcom1b,axcom2,axcom3a,axcom3b,axcom4a,axcom4b,axcom5a,axcom5b,axcom6a,axcom6b,axcom7a,axcom7b,axcom8a,axcom8b)

    # Normalization for plots
    f_calc_rel = f_calc/f_data
    his_f_calc_rel.append([str(rec) + "//", f_calc_rel])
    print("********************************************************************************************")

    # Update theta with RPROP
    ##########################
    grad_curr = f_data-f_calc
    print('This is the difference of theta: ', plt.sum(plt.absolute(theta_chosen - theta)))
    print('This is grad current: ', plt.sum(plt.absolute(grad_curr)))
    [grad_prev, del_theta_prev, exception, theta, update, multi_grads] = RPROP(grad_curr,grad_prev,update,theta,del_theta_prev,exception)
    his_multi_grads.append([str(rec)+"//",multi_grads])
    his_grad_current.append([str(rec) + "//", grad_prev])

    # Plots in the for loop
    axf.plot([f_calc_rel[0], f_calc_rel[1], f_calc_rel[2], f_calc_rel[3], f_calc_rel[4]],'-', marker='o', markersize=6, label="Calc Features iter: " + str(rec))
    axf.legend()
    axfn.plot([f_calc[0],f_calc[1],f_calc[2],f_calc[3],f_calc[4]],'-', marker='o', markersize=6, label="Calc Features iter: " + str(rec))
    axfn.legend()
    rec = rec + 1

    his_weights.append([str(rec) + "//", theta])
    acw.plot([theta[0], theta[1], theta[2], theta[3], theta[4]], '-', marker='o', markersize=6,label="iter " + str(rec))
    acw.legend()
    ###########################

# Post - processing
print("This is the history of his_multi_grads.")
print("------------------------------------------")
print('\n')
for i in plt.arange(0,len(his_multi_grads),1):
    print(his_multi_grads[i])

print("This is the history of his_multi_grads.")
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
for i in plt.arange(1,len(his_weights),1):
    print("This is update " + str(i))
    print(his_weights[i][1] - his_weights[i-1][1])

post_processing_plots(his_f_calc_rel,his_weights,his_multi_grads,his_grad_current)

plt.show()
#####################
