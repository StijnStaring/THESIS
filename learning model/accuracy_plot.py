import pylab as plt

# data_5 = ['14//', [plt.array([[0.9963245 ],
#    [1.00779448],
#    [1.03313514],
#    [0.98553052],
#    [0.99785629]]), plt.array([[0.99275782],
#    [1.02545127],
#    [1.05708292],
#    [0.96348837],
#    [0.99477323]]), plt.array([[0.99951578],
#    [0.99409866],
#    [1.01380527],
#    [0.99797314],
#    [1.00138409]]), plt.array([[0.99312771],
#    [1.02172643],
#    [1.05593979],
#    [0.96978943],
#    [0.9953709 ]]), plt.array([[1.00301675],
#    [0.98033718],
#    [0.99297743],
#    [1.01017053],
#    [1.0053102 ]])]]
#
# data_3 = ['17//', [plt.array([[0.99889349],
#        [0.99992747],
#        [1.01140094],
#        [0.99539895],
#        [1.00052086]]), plt.array([[1.00268378],
#        [0.98423694],
#        [0.98948475],
#        [1.00947011],
#        [1.00485724]]), plt.array([[0.99456194],
#        [1.01872307],
#        [1.04055215],
#        [0.9758468 ],
#        [0.99630615]])]]

# data_average = ['111//', [plt.array([[1.00003019],
#        [0.99974506],
#        [0.99989975],
#        [1.00011264],
#        [1.0000876 ]]),  plt.array([[0.99990027],
#        [1.0002674 ],
#        [1.00059783],
#        [0.99948536],
#        [0.99997304]]),  plt.array([[1.00011468],
#        [0.99940836],
#        [0.99942557],
#        [1.0004103 ],
#        [1.00017899]])]]
#
# data_conflict = ['17//', [plt.array([[0.99889349],
#        [0.99992747],
#        [1.01140094],
#        [0.99539895],
#        [1.00052086]]), plt.array([[1.00268378],
#        [0.98423694],
#        [0.98948475],
#        [1.00947011],
#        [1.00485724]]), plt.array([[0.99456194],
#        [1.01872307],
#        [1.04055215],
#        [0.9758468 ],
#        [0.99630615]])]]
#
# data_seq = ['1//', [plt.array([[0.99372266],
#  [1.0364602 ],
#  [1.05047609],
#  [0.96796123],
#  [0.99053099]]), plt.array([[1.01466279],
#  [0.94177966],
#  [0.91670696],
#  [1.05395378],
#  [1.01738989]])]]
#
#
# acc_a = accuracy(data_average,5)
# acc_c = accuracy(data_conflict,5)
# acc_s = accuracy(data_seq,5)


# # Accuracy figure
# plt.figure("Average error between f_calculated and f_data")
# acc = plt.gca()
# plt.xlabel("feature [-]",fontsize=14)
# plt.ylabel("error [%]",fontsize=14)
# plt.title("Average error between f_calculated and f_data",fontsize=14)
# plt.grid(True)
#
# features = [0,1,2,3,4]
# acc.plot(features, acc_5, '-',marker = '*',linewidth=3.0)
# acc.plot(features, acc_3, '-', marker = 'o', linewidth=3.0)
# plt.legend(['5 datasets','3 datasets'])
# plt.show()

def accuracy(data,amount_features):
    ref = plt.ones([amount_features,1])
    sum5 = plt.zeros([amount_features, 1])
    for k in plt.arange(0, len(data[1]), 1):
        sum5 = sum5 + plt.absolute(ref - data[1][k])
    sum5 = sum5 * 100

    return sum5 / len(data[1])

data_V22_L6 = ['1//', [plt.array([[0.99372266],
       [1.0364602 ],
       [1.05047609],
       [0.96796123],
       [0.99053099]])]]

data_V25_L3 = ['1//', [plt.array([[1.01466279],
       [0.94177966],
       [0.91670696],
       [1.05395378],
       [1.01738989]])]]

data_V22_L6_a = ['1//', [plt.array([[0.99324901],
       [1.03941249],
       [1.05436148],
       [0.96550211],
       [0.98981001]])]]

data_V25_L3_a = ['1//', [plt.array([[1.01502664],
       [0.94043785],
       [0.91469953],
       [1.05530596],
       [1.01782593]])]]

acc_V22_L6 = accuracy(data_V22_L6 ,5)
acc_V25_L3 = accuracy(data_V25_L3 ,5)
acc_V22_L6_a = accuracy(data_V22_L6_a ,5)
acc_V25_L3_a = accuracy(data_V25_L3_a ,5)

# Accuracy figure
plt.figure("Average error between f_calculated and f_data")
acc = plt.gca()
plt.xlabel("feature [-]",fontsize=14)
plt.ylabel("error [%]",fontsize=14)
plt.title("Comparison average error between different methods",fontsize=14)
plt.grid(True)

features = [0,1,2,3,4]
acc.plot(features, acc_V22_L6, '-',marker = '*',linewidth=3.0)
acc.plot(features, acc_V25_L3, '-',marker = '*',linewidth=3.0)
acc.plot(features, acc_V22_L6_a, '-',marker = 'o',linewidth=3.0)
acc.plot(features, acc_V25_L3_a, '-',marker = 'o',linewidth=3.0)
plt.legend(['data_V22_L6','data_V25_L3','data_V22_L6_a','data_V25_L3_a'])
plt.show()

