import pylab as plt
fontsize = 14

def accuracy(data,amount_features):
    sum5 = plt.zeros([amount_features])
    ref = plt.ones([amount_features])
    for k in plt.arange(len(data)):
        sum5 = sum5 + plt.absolute(ref - plt.squeeze(data[k]))
    sum5 = sum5 * 100
    return sum5 / len(data)

data_1 = [plt.array([[0.98812153],
       [0.99849737],
       [0.99935298],
       [0.99891749],
       [0.99172111],
       [1.00038716]]), plt.array([0.99762178, 0.99935854, 1.01619369, 1.00042546, 0.98773897 ,1.00009021])]

data_2 = [[0.99392706 ,1.00026585, 0.99113348, 1.00156341, 1.00087249, 0.9998164 ] , [0.99130714 ,0.99930115 ,0.98681464 ,0.99992436, 0.99854521 ,1.00014452], [1.00032749 ,1.00015496 ,1.00320535, 1.00143575 ,0.99408281 ,0.99984878]]
data_3 = [[0.99997562 ,1.00023764 ,0.98602149 ,1.00118025 ,0.99722565 ,0.99985419],[0.99727138 ,0.99924905, 0.98160448, 0.99949518 ,0.99483734, 1.00019095],[1.00635125 ,1.00012406 ,0.99829497, 1.00104949, 0.99022146, 0.99988737]]

acc_1 = accuracy(data_1, 6)
acc_2 = accuracy(data_2, 6)
acc_3 = accuracy(data_3, 6)


# Accuracy figure
plt.figure("Average error between f_calculated and f_data")
acc = plt.gca()
plt.xlabel("feature [-]", fontsize=14)
plt.ylabel("error [%]", fontsize=14)
plt.title("Average error between f_calculated and f_data", fontsize=14)
plt.grid(True)


features = [0, 1, 2, 3, 4, 5]
acc.plot(features, acc_1, '-', marker='*', linewidth=3.0)
acc.plot(features, acc_2, '-', marker='o', linewidth=3.0)
acc.plot(features, acc_3, '-', marker='s', linewidth=3.0)
plt.legend(['Sequential method', 'Conflict method', 'Average method'])


for tick in acc.xaxis.get_major_ticks():
    tick.label1.set_fontsize(fontsize)
    tick.label1.set_fontweight('bold')
for tick in acc.yaxis.get_major_ticks():
    tick.label1.set_fontsize(fontsize)
    tick.label1.set_fontweight('bold')

plt.show()



