def RPROP(grad_curr,grad_prev,update,w_curr,del_w_prev,exception):
    import pylab as plt
    # Expects grad in shape n x 1
    # definition of parameters:
    n_pos = 1.2
    n_neg = 0.5
    del_max = 1.0
    del_min = 1e-6
    w_new = plt.zeros([len(grad_curr),1])
    del_w = plt.zeros([len(grad_curr),1])


    # algorithm
    for i in plt.arange(0,len(grad_curr),1):
        print('i = ',i)
        print('\n')
        if exception[i] == 1:
            grad_prev[i] = 0
            print('exception is used')

        if grad_curr[i]*grad_prev[i] > 0:
            update[i] = min(update[i]*n_pos,del_max)
            del_w[i] = -plt.sign(grad_curr[i])*update[i]
            w_new[i] = w_curr[i] + del_w[i]
            exception[i] = 0
            print('2x pos grad')

        elif grad_curr[i]*grad_prev[i] < 0:
            update[i] = max(update[i] * n_neg, del_min)
            del_w[i] = - del_w_prev[i]
            w_new[i] = w_curr[i] +del_w[i]
            exception[i] = 1
            print('2x neg grad')

        elif grad_curr[i]*grad_prev[i] == 0:
            del_w[i] = -plt.sign(grad_curr[i]) * update[i]
            w_new[i] = w_curr[i] + del_w[i]
            exception[i] = 0
            print('2x grad = 0')


    return grad_curr, del_w, exception, w_new, update