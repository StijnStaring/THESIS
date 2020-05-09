import pylab as plt
fontsize = 14

def accuracy(data,amount_features):
    sum5 = plt.zeros([amount_features])
    ref = plt.ones([amount_features])
    for k in plt.arange(len(data)):
        sum5 = sum5 + plt.absolute(ref - plt.squeeze(data[k]))
    sum5 = sum5 * 100
    return sum5 / len(data)

def weight_error(weight,chosen_weight):
    diff = plt.absolute(weight - chosen_weight)
    result = 100 * diff/chosen_weight
    return result

data_1 = []
weight_1 = plt.array([1,1,1])
chosen_weight = plt.array([2,2,3])
print(weight_error(weight_1,chosen_weight))
data_2 = []
weight_2 = plt.array([1,1,1])

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
acc.plot(features, acc_2, '-', marker='o', linewidth=3.0)

plt.legend(['Conflict method', 'Average method'])


for tick in acc.xaxis.get_major_ticks():
    tick.label1.set_fontsize(fontsize)
    tick.label1.set_fontweight('bold')
for tick in acc.yaxis.get_major_ticks():
    tick.label1.set_fontsize(fontsize)
    tick.label1.set_fontweight('bold')



# Accuracy figure - learned weights
plt.figure("Relative error of learned weights")
awe = plt.gca()
plt.xlabel("weight", fontsize=14)
plt.ylabel("error [%]", fontsize=14)
plt.title("Relative error of learned weights", fontsize=14)
plt.grid(True)


features = [1, 3, 5]
awe.plot(features, we_1, '-', marker='*', linewidth=3.0)
awe.plot(features, we_2, '-', marker='o', linewidth=3.0)

plt.legend(['Conflict method', 'Average method'])


for tick in awe.xaxis.get_major_ticks():
    tick.label1.set_fontsize(fontsize)
    tick.label1.set_fontweight('bold')
for tick in awe.yaxis.get_major_ticks():
    tick.label1.set_fontsize(fontsize)
    tick.label1.set_fontweight('bold')



plt.show()



