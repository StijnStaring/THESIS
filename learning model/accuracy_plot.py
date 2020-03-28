import pylab as plt

def accuracy(data,amount_features):
    ref = plt.ones([amount_features,1])
    sum5 = plt.zeros([amount_features, 1])
    for k in plt.arange(0, len(data[1]), 1):
        sum5 = sum5 + plt.absolute(ref - data[1][k])
    sum5 = sum5 * 100

    return sum5 / len(data[1])


data_5 = ['14//', [plt.array([[0.9963245 ],
   [1.00779448],
   [1.03313514],
   [0.98553052],
   [0.99785629]]), plt.array([[0.99275782],
   [1.02545127],
   [1.05708292],
   [0.96348837],
   [0.99477323]]), plt.array([[0.99951578],
   [0.99409866],
   [1.01380527],
   [0.99797314],
   [1.00138409]]), plt.array([[0.99312771],
   [1.02172643],
   [1.05593979],
   [0.96978943],
   [0.9953709 ]]), plt.array([[1.00301675],
   [0.98033718],
   [0.99297743],
   [1.01017053],
   [1.0053102 ]])]]

data_3 = ['17//', [plt.array([[0.99889349],
       [0.99992747],
       [1.01140094],
       [0.99539895],
       [1.00052086]]), plt.array([[1.00268378],
       [0.98423694],
       [0.98948475],
       [1.00947011],
       [1.00485724]]), plt.array([[0.99456194],
       [1.01872307],
       [1.04055215],
       [0.9758468 ],
       [0.99630615]])]]

acc_5 = accuracy(data_5,5)
acc_3 = accuracy(data_3,5)


# Accuracy figure
plt.figure("Average error between f_calculated and f_data")
acc = plt.gca()
plt.xlabel("feature [-]",fontsize=14)
plt.ylabel("error [%]",fontsize=14)
plt.title("Average error between f_calculated and f_data",fontsize=14)
plt.grid(True)

features = [0,1,2,3,4]
acc.plot(features, acc_5, '-',marker = '*',linewidth=3.0)
acc.plot(features, acc_3, '-', marker = 'o', linewidth=3.0)
plt.legend(['5 datasets','3 datasets'])
plt.show()







