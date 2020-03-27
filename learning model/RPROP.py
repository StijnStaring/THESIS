def RPROP(grad_curr,n_neg,case,length,update,w_curr,del_w_prev):
    import pylab as plt
    # Expects grad in shape n x 1
    # definition of parameters:
    n_pos = 1.2
    # n_neg = 0.5
    del_max = 1.0
    del_min = 1e-7
    update_out = plt.zeros([length,1]) # if not have been initialized, would directely change global var and all vars in his_list/ if change something in array --> still pointing to same location and not a new point is made.
    w_new = plt.zeros([length,1])
    del_w = plt.zeros([length,1])
    exception = plt.zeros([length,1])
    # multi_grads = plt.zeros([length,1])

    # algorithm
    for i in plt.arange(0,length,1):
        print('feature in RPROP  = ',i)
        print('\n')

        # storage of mutli_grads
        # multi_grads[i] = grad_curr[i]*grad_prev[i]

        if case[i] == 1:
            update[i] = min(update[i]*n_pos,del_max)
            del_w[i] = -plt.sign(grad_curr[i])*update[i]
            w_new[i] = w_curr[i] + del_w[i]
            exception[i] = 0
            print('2x pos grad')

        elif case[i] == 2:
            update[i] = max(update[i] * n_neg, del_min)
            del_w[i] = - del_w_prev[i]
            w_new[i] = w_curr[i] +del_w[i]
            exception[i] = 1
            print('2x neg grad')

        elif case[i] == 3:
            del_w[i] = -plt.sign(grad_curr[i]) * update[i]
            w_new[i] = w_curr[i] + del_w[i]
            exception[i] = 0
            print('2x grad = 0')

    for i in plt.arange(0,len(update),1):
        update_out[i] = update[i]


    return del_w, exception, w_new, update_out