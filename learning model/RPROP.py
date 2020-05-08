def RPROP(grad_curr,case,length,update,w_curr,del_w_prev,conflict_flags,index_fixed_value):
    import pylab as plt

    # definition of parameters:
    n_pos = 1.2
    n_neg = 0.5
    del_max = 1.0
    del_min = 1e-7
    update_out = plt.zeros([length]) # if not have been initialized, would directely change global var and all vars in his_list/ if change something in array --> still pointing to same location and not a new point is made.
    w_new = plt.zeros([length])
    del_w = plt.zeros([length])
    exception = plt.zeros([length])

    # algorithm
    for i in plt.arange(0,length,1):
        if conflict_flags[i] == 1:
            del_w[i] = 0
            w_new[i] = w_curr[i]
            exception[i] = 1  # Stands for in conflict - after is solved --> go to case 3

        else:
            if case[i] == 1:
                update[i] = min(update[i]*n_pos,del_max)
                del_w[i] = -plt.sign(grad_curr[i])*update[i]
                w_new[i] = w_curr[i] + del_w[i]
                exception[i] = 0


            elif case[i] == 2:
                update[i] = max(update[i] * n_neg, del_min)
                del_w[i] = - del_w_prev[i]
                w_new[i] = w_curr[i] +del_w[i]
                exception[i] = 1


            elif case[i] == 3:
                del_w[i] = -plt.sign(grad_curr[i]) * update[i]
                w_new[i] = w_curr[i] + del_w[i]
                exception[i] = 0


    for i in plt.arange(0,len(update),1):
        update_out[i] = update[i] # now is pointing to local defined here
    # update_out = update --> if use this is still pointing to original update (changes all in list)

    return del_w, exception, w_new, update_out