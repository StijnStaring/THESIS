import pylab as plt
fontsize = 18
font_label = 20
plt.rc('legend', fontsize=font_label)
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


# 3 datasets averaging
# data_1 = [[0.99997562, 1.00023764 ,0.98602149 ,1.00118025 ,0.99722565 ,0.99985419],[0.99727138 ,0.99924905 ,0.98160448 ,0.99949518, 0.99483734 ,1.00019095],[1.00635125 ,1.00012406, 0.99829497 ,1.00104949, 0.99022146, 0.99988737]]

# 3 datasets conflict
# data_1 = [[0.99392706 ,1.00026585 ,0.99113348, 1.00156341 ,1.00087249 ,0.9998164 ],[0.99130714, 0.99930115 ,0.98681464 ,0.99992436 ,0.99854521 ,1.00014452],[1.00032749 ,1.00015496 ,1.00320535 ,1.00143575 ,0.99408281 ,0.99984878]]

# 7 datasets averaging
data_1 = [[0.99378696 ,1.00082118 ,0.98113166 ,1.00151117 ,1.00689718 ,0.99971199],[0.99104754 ,0.99981456 ,0.9766457 , 0.99979352 ,1.00444035 ,1.00005493],[0.99993103, 1.00070552, 0.99307717 ,1.00137845, 0.99968991 ,0.99974565],[0.99658997 ,0.99935588 ,0.9878989 , 0.99926943 ,0.99646608 ,1.00018896],[1.00348416, 1.00061082 ,1.00143213 ,1.00126844 ,0.99528605 ,0.99977337],[0.99960563 ,0.99898119 ,0.995626 ,  0.99883583 ,0.99144376 ,1.0002991 ],[1.00148525 ,0.99867696 ,1.00120059 ,0.99847659 ,0.98790976 ,1.00038925]]

# 7 datasets conflict
data_2 =[[0.99415498, 1.00066251 ,0.9919431 , 1.00275525 ,1.00344368 ,0.99963846],[0.99140488 ,0.99964902 ,0.98739677 ,1.0010289 , 1.00098453 ,0.99998331],[1.00050761 ,1.00054615 ,1.00403272, 1.00262082 ,0.99647765 ,0.99967238],[0.99714813, 0.99918703 ,0.99878292 ,1.00049973 ,0.99324712 ,1.00011837],[1.00416984, 1.00045086 ,1.01246433 ,1.00250957 ,0.99220275 ,0.99970029],[1.00026685 ,0.99880961 ,1.00657738 ,1.00006219 ,0.98835049 ,1.00022935],[1.00220874 ,0.99850317 ,1.01218071 ,0.99969988 ,0.98489006 ,1.00032017]]

acc_1 = accuracy(data_1, 3)
acc_2 = accuracy(data_2, 3)

# Accuracy figure - f_calc
plt.figure("Average error between f_calculated and f_data")
acc = plt.gca()
plt.xlabel("feature [-]", fontsize=font_label,fontweight='bold')
plt.ylabel("error [%]", fontsize=font_label,fontweight='bold')
# plt.title("Average error between f_calculated and f_data", fontsize=14)
plt.grid(True)


features = [1, 3, 5]
acc.plot(features, acc_1, '-', marker='*', linewidth=3.0)
acc.plot(features, acc_2, '-', marker='s', linewidth=3.0)
plt.legend(['Average method', 'Conflict method'])


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



