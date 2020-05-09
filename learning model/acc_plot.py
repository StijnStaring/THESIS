import pylab as plt
fontsize = 14

def accuracy(data,amount_features):
    sum5 = plt.zeros([amount_features])
    ref = plt.ones([amount_features])
    for k in plt.arange(len(data)):
        temp = plt.array([data[k][1],data[k][3],data[k][5]])
        sum5 = sum5 + plt.absolute(ref - temp)
    sum5 = sum5 * 100
    return sum5 / len(data)

def weight_error(weight,chosen_weight):
    diff = plt.absolute(weight - chosen_weight)
    result = 100 * diff/chosen_weight
    return result



data_1 = [[0.99392706 ,1.00026585, 0.99113348 ,1.00156341 ,1.00087249, 0.9998164 ],
[0.99130714 ,0.99930115, 0.98681464, 0.99992436 ,0.99854521 ,1.00014452],
[1.00032749 ,1.00015496, 1.00320535 ,1.00143575 ,0.99408281 ,0.99984878]]


data_2 = [[0.99415498 ,1.00066251, 0.9919431  ,1.00275525 ,1.00344368 ,0.99963846]
,[0.99140488, 0.99964902 ,0.98739677 ,1.0010289 , 1.00098453 ,0.99998331],[1.00050761, 1.00054615 ,1.00403272 ,1.00262082, 0.99647765 ,0.99967238]
,[0.99714813, 0.99918703, 0.99878292, 1.00049973, 0.99324712, 1.00011837]
,[1.00416984 ,1.00045086 ,1.01246433 ,1.00250957, 0.99220275, 0.99970029],[1.00026685, 0.99880961 ,1.00657738, 1.00006219 ,0.98835049, 1.00022935],
[1.00220874, 0.99850317 ,1.01218071 ,0.99969988 ,0.98489006 ,1.00032017]]



acc_1 = accuracy(data_1, 3)
acc_2 = accuracy(data_2, 3)

# Accuracy figure - f_calc
plt.figure("Average error between f_calculated and f_data")
acc = plt.gca()
plt.xlabel("feature [-]", fontsize=14)
plt.ylabel("error [%]", fontsize=14)
plt.title("Average error between f_calculated and f_data", fontsize=14)
plt.grid(True)


features = [1, 3, 5]
acc.plot(features, acc_1, '-', marker='*', linewidth=3.0)
acc.plot(features, acc_2, '-', marker='s', linewidth=3.0)

plt.legend(['3 Datasets', '7 Datasets'])


for tick in acc.xaxis.get_major_ticks():
    tick.label1.set_fontsize(fontsize)
    tick.label1.set_fontweight('bold')
for tick in acc.yaxis.get_major_ticks():
    tick.label1.set_fontsize(fontsize)
    tick.label1.set_fontweight('bold')







# # Accuracy figure - learned weights
# plt.figure("Relative error of learned weights")
# awe = plt.gca()
# plt.xlabel("weight", fontsize=14)
# plt.ylabel("error [%]", fontsize=14)
# plt.title("Relative error of learned weights", fontsize=14)
# plt.grid(True)
#
#
# features = [1, 3, 5]
# awe.plot(features, we_1, '-', marker='*', linewidth=3.0)
# awe.plot(features, we_2, '-', marker='o', linewidth=3.0)
#
# plt.legend(['Conflict method', 'Average method'])
#
#
# for tick in awe.xaxis.get_major_ticks():
#     tick.label1.set_fontsize(fontsize)
#     tick.label1.set_fontweight('bold')
# for tick in awe.yaxis.get_major_ticks():
#     tick.label1.set_fontsize(fontsize)
#     tick.label1.set_fontweight('bold')



plt.show()



