import pylab as plt
fontsize = 16

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


# 3 datasets
data_1 = [[0.99997562, 1.00023764 ,0.98602149 ,1.00118025 ,0.99722565 ,0.99985419],[0.99727138 ,0.99924905 ,0.98160448 ,0.99949518, 0.99483734 ,1.00019095],[1.00635125 ,1.00012406, 0.99829497 ,1.00104949, 0.99022146, 0.99988737]]

# 5 datasets
data_2 = [[0.99378696 ,1.00082118 ,0.98113166 ,1.00151117 ,1.00689718 ,0.99971199],[0.99104754 ,0.99981456 ,0.9766457 , 0.99979352 ,1.00444035 ,1.00005493],[0.99993103, 1.00070552, 0.99307717 ,1.00137845, 0.99968991 ,0.99974565],[0.99658997 ,0.99935588 ,0.9878989 , 0.99926943 ,0.99646608 ,1.00018896],[1.00348416, 1.00061082 ,1.00143213 ,1.00126844 ,0.99528605 ,0.99977337],[0.99960563 ,0.99898119 ,0.995626 ,  0.99883583 ,0.99144376 ,1.0002991 ],[1.00148525 ,0.99867696 ,1.00120059 ,0.99847659 ,0.98790976 ,1.00038925]]



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



