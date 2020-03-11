def post_processing_plots(his_f_calc_rel,his_weights,his_multi_grads,his_grad_current,his_diff_theta):
    """"
    Theta_iter is a string
    """
    import pylab as plt

    # Plot 1: normalized calculated features
    length = len(his_f_calc_rel)
    iterations = plt.arange(1,length+1,1)
    iterations = iterations[:,plt.newaxis]

    # Plotting convergence of normalized features
    plt.figure("Convergence of features",figsize=(10, 4))

    plt.subplot(2, 3, 1)
    ac1 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Normalized calculated feature 1 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Convergence of features 1',fontsize=14)

    plt.subplot(2, 3, 2)
    ac2 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Normalized calculated feature 2 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Convergence of features 2', fontsize=14)

    plt.subplot(2, 3, 3)
    ac3 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Normalized calculated feature 3 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Convergence of features 3', fontsize=14)

    plt.subplot(2, 3, 4)
    ac4 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Normalized calculated feature 4 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Convergence of features 4', fontsize=14)

    plt.subplot(2, 3, 5)
    ac5 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Normalized calculated feature 5 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Convergence of features 5', fontsize=14)

    # plt.subplot(2, 4, 6)
    # ac6 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Normalized calculated feature 6 [-]", fontsize=14)
    # plt.grid(True)
    # plt.title('Convergence of features 6', fontsize=14)
    #
    # plt.subplot(2, 4, 7)
    # ac7 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Normalized calculated feature 7 [-]", fontsize=14)
    # plt.grid(True)
    # plt.title('Convergence of features 7', fontsize=14)

    for i in plt.arange(0,length,1):
        ac1.plot(iterations[i], his_f_calc_rel[i][1][0], 'o', label= "iteration "+str(i+1), linewidth=3.0)
        ac2.plot(iterations[i], his_f_calc_rel[i][1][1], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac3.plot(iterations[i], his_f_calc_rel[i][1][2], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac4.plot(iterations[i], his_f_calc_rel[i][1][3], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac5.plot(iterations[i], his_f_calc_rel[i][1][4], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        # ac6.plot(iterations[i], his_f_calc_rel[i][1][5], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        # ac7.plot(iterations[i], his_f_calc_rel[i][1][6], 'o', label="iteration " + str(i + 1), linewidth=3.0)


# Weights over iterations
    plt.figure("Weights",figsize=(10, 4))

    plt.subplot(2, 3, 1)
    ac1 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Weight of feature 1',fontsize=14)

    plt.subplot(2, 3, 2)
    ac2 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Weight of feature 2', fontsize=14)

    plt.subplot(2, 3, 3)
    ac3 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Weight of feature 3', fontsize=14)

    plt.subplot(2, 3, 4)
    ac4 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Weight of feature 4', fontsize=14)

    plt.subplot(2, 3, 5)
    ac5 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Weight of feature 5', fontsize=14)

    # plt.subplot(2, 4, 6)
    # ac6 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Weight value [-]", fontsize=14)
    # plt.grid(True)
    # plt.title('Weight of feature 6', fontsize=14)
    #
    # plt.subplot(2, 4, 7)
    # ac7 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Weight value [-]", fontsize=14)
    # plt.grid(True)
    # plt.title('Weight of feature 7', fontsize=14)

    iterations_w = plt.arange(1, len(his_weights) + 1, 1)
    iterations_w = iterations_w[:, plt.newaxis]
    for i in plt.arange(0,len(his_weights),1):
        ac1.plot(iterations_w[i], his_weights[i][1][0], 'o', label= "iteration "+str(i+1), linewidth=3.0)
        ac2.plot(iterations_w[i], his_weights[i][1][1], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac3.plot(iterations_w[i], his_weights[i][1][2], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac4.plot(iterations_w[i], his_weights[i][1][3], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac5.plot(iterations_w[i], his_weights[i][1][4], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        # ac6.plot(iterations_w[i], his_weights[i][1][5], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        # ac7.plot(iterations_w[i], his_weights[i][1][6], 'o', label="iteration " + str(i + 1), linewidth=3.0)


    # Plotting update of theta
    plt.figure("delta theta", figsize=(10, 4))
    plt.subplot(2, 3, 1)
    ac1 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("delta weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Update of theta 1', fontsize=14)

    plt.subplot(2, 3, 2)
    ac2 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("delta weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Update of theta 2', fontsize=14)

    plt.subplot(2, 3, 3)
    ac3 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("delta weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Update of theta 3', fontsize=14)

    plt.subplot(2, 3, 4)
    ac4 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("delta weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Update of theta 4', fontsize=14)

    plt.subplot(2, 3, 5)
    ac5 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("delta weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Update of theta 5', fontsize=14)

    # plt.subplot(2, 4, 6)
    # ac6 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("delta weight value [-]", fontsize=14)
    # plt.grid(True)
    # plt.title('Update of theta 6', fontsize=14)
    #
    # plt.subplot(2, 4, 7)
    # ac7 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("delta weight value [-]", fontsize=14)
    # plt.grid(True)
    # plt.title('Update of theta 7', fontsize=14)

    for i in plt.arange(1, length, 1):
        ac1.plot(iterations[i], his_weights[i][1][0]-his_weights[i-1][1][0], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac2.plot(iterations[i], his_weights[i][1][1]-his_weights[i-1][1][1], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac3.plot(iterations[i], his_weights[i][1][2]-his_weights[i-1][1][2], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac4.plot(iterations[i], his_weights[i][1][3]-his_weights[i-1][1][3], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac5.plot(iterations[i], his_weights[i][1][4]-his_weights[i-1][1][4], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        # ac6.plot(iterations[i], his_weights[i][1][5]-his_weights[i-1][1][5], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        # ac7.plot(iterations[i], his_weights[i][1][6]-his_weights[i-1][1][6], 'o', label="iteration " + str(i + 1), linewidth=3.0)

    # Multiplication history of the gradients
    plt.figure("Multiplication history of the gradients", figsize=(10, 4))

    plt.subplot(2, 3, 1)
    ac1 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Multi grad feature 1 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Multi grad of features 1', fontsize=14)

    plt.subplot(2, 3, 2)
    ac2 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Multi gradfeature 2 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Multi grad of features 2', fontsize=14)

    plt.subplot(2, 3, 3)
    ac3 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Multi grad feature 3 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Multi grad of features 3', fontsize=14)

    plt.subplot(2, 3, 4)
    ac4 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Multi grad feature 4 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Multi grad of features 4', fontsize=14)

    plt.subplot(2, 3, 5)
    ac5 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Multi grad feature 5 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Multi grad of features 5', fontsize=14)

    # plt.subplot(2, 4, 6)
    # ac6 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Multi grad feature 6 [-]", fontsize=14)
    # plt.grid(True)
    # plt.title('Multi grad of features 6', fontsize=14)
    #
    # plt.subplot(2, 4, 7)
    # ac7 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Multi grad feature 7 [-]", fontsize=14)
    # plt.grid(True)
    # plt.title('Multi grad of features 7', fontsize=14)

    for i in plt.arange(0, len(his_multi_grads), 1):
        ac1.plot(iterations[i], his_multi_grads[i][1][0], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac2.plot(iterations[i], his_multi_grads[i][1][1], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac3.plot(iterations[i], his_multi_grads[i][1][2], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac4.plot(iterations[i], his_multi_grads[i][1][3], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac5.plot(iterations[i], his_multi_grads[i][1][4], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        # ac6.plot(iterations[i], his_multi_grads[i][1][5], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        # ac7.plot(iterations[i], his_multi_grads[i][1][6], 'o', label="iteration " + str(i + 1), linewidth=3.0)

    # history of current gradient
    plt.figure("History of current gradient", figsize=(10, 4))

    plt.subplot(2, 3, 1)
    ac1 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("grad feature 1 [-]", fontsize=14)
    plt.grid(True)
    plt.title('grad of features 1', fontsize=14)

    plt.subplot(2, 3, 2)
    ac2 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("gradfeature 2 [-]", fontsize=14)
    plt.grid(True)
    plt.title('grad of features 2', fontsize=14)

    plt.subplot(2, 3, 3)
    ac3 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("grad feature 3 [-]", fontsize=14)
    plt.grid(True)
    plt.title('grad of features 3', fontsize=14)

    plt.subplot(2, 3, 4)
    ac4 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("grad feature 4 [-]", fontsize=14)
    plt.grid(True)
    plt.title('grad of features 4', fontsize=14)

    plt.subplot(2, 3, 5)
    ac5 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("grad feature 5 [-]", fontsize=14)
    plt.grid(True)
    plt.title('grad of features 5', fontsize=14)

    # plt.subplot(2, 4, 6)
    # ac6 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Multi grad feature 6 [-]", fontsize=14)
    # plt.grid(True)
    # plt.title('Multi grad of features 6', fontsize=14)
    #
    # plt.subplot(2, 4, 7)
    # ac7 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Multi grad feature 7 [-]", fontsize=14)
    # plt.grid(True)
    # plt.title('Multi grad of features 7', fontsize=14)

    for i in plt.arange(0, len(his_grad_current), 1):
        ac1.plot(iterations[i], his_grad_current[i][1][0], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac2.plot(iterations[i], his_grad_current[i][1][1], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac3.plot(iterations[i], his_grad_current[i][1][2], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac4.plot(iterations[i], his_grad_current[i][1][3], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac5.plot(iterations[i], his_grad_current[i][1][4], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        # ac6.plot(iterations[i], his_grad_current[i][1][5], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        # ac7.plot(iterations[i], his_grad_current[i][1][6], 'o', label="iteration " + str(i + 1), linewidth=3.0)

    # history of difference theta
    plt.figure("History of diff theta", figsize=(10, 4))

    plt.subplot(2, 3, 1)
    ac1 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("diff theta 1 [-]", fontsize=14)
    plt.grid(True)
    plt.title('diff theta 1', fontsize=14)

    plt.subplot(2, 3, 2)
    ac2 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("diff theta 2 [-]", fontsize=14)
    plt.grid(True)
    plt.title('diff theta 2', fontsize=14)

    plt.subplot(2, 3, 3)
    ac3 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("diff theta 3 [-]", fontsize=14)
    plt.grid(True)
    plt.title('diff theta 3', fontsize=14)

    plt.subplot(2, 3, 4)
    ac4 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("diff theta 4 [-]", fontsize=14)
    plt.grid(True)
    plt.title('diff theta 4', fontsize=14)

    plt.subplot(2, 3, 5)
    ac5 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("diff theta 5 [-]", fontsize=14)
    plt.grid(True)
    plt.title('diff theta 5', fontsize=14)

    # plt.subplot(2, 4, 6)
    # ac6 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Multi grad feature 6 [-]", fontsize=14)
    # plt.grid(True)
    # plt.title('Multi grad of features 6', fontsize=14)
    #
    # plt.subplot(2, 4, 7)
    # ac7 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Multi grad feature 7 [-]", fontsize=14)
    # plt.grid(True)
    # plt.title('Multi grad of features 7', fontsize=14)

    for i in plt.arange(0, len(his_diff_theta), 1):
        ac1.plot(iterations[i], his_diff_theta[i][1][0], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac2.plot(iterations[i], his_diff_theta[i][1][1], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac3.plot(iterations[i], his_diff_theta[i][1][2], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac4.plot(iterations[i], his_diff_theta[i][1][3], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac5.plot(iterations[i], his_diff_theta[i][1][4], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        # ac6.plot(iterations[i], his_diff_theta[i][1][5], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        # ac7.plot(iterations[i], his_diff_theta[i][1][6], 'o', label="iteration " + str(i + 1), linewidth=3.0)
