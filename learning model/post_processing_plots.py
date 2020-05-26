def post_processing_plots(his_f_calc_rel,his_weights,his_multi_grads,his_grad_current,his_diff_theta):
    """"
    Theta_iter is a string
    """
    import pylab as plt
    plt.rc('axes', linewidth=2)
    plt.rc('legend',fontsize=20) # using a size in points
    # plt.rcParams["legend.loc"] = 'best'
    fontsize = 20
    font = 22

    # Plot 1: normalized calculated features
    # Plotting convergence of normalized features
    plt.figure("Convergence of features", figsize=(24, 12))
    plt.tight_layout()
    plt.subplot(2, 3, 1)
    acf1 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Normalized calculated feature 1 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Convergence of feature 1', fontsize=font, fontweight = 'bold')

    for tick in acf1.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acf1.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 2)
    acf2 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Normalized calculated feature 2 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Convergence of feature 2', fontsize=font, fontweight = 'bold')

    for tick in acf2.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acf2.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 3)
    acf3 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Normalized calculated feature 3 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Convergence of feature 3', fontsize=font, fontweight = 'bold')

    for tick in acf3.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acf3.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 4)
    acf4 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Normalized calculated feature 4 [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('Convergence of feature 4', fontsize=font, fontweight = 'bold')

    for tick in acf4.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acf4.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 5)
    acf5 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Normalized calculated feature 5 [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('Convergence of feature 5', fontsize=font, fontweight = 'bold')

    for tick in acf5.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acf5.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 6)
    acf6 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Normalized calculated feature 5 [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('Convergence of feature 5', fontsize=font, fontweight='bold')

    for tick in acf6.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acf6.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    iterations = plt.arange(1, len(his_f_calc_rel) + 1, 1)
    for i in plt.arange(0, len(iterations), 1):
        acf1.plot(iterations[i], his_f_calc_rel[i][1][0], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acf2.plot(iterations[i], his_f_calc_rel[i][1][1], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acf3.plot(iterations[i], his_f_calc_rel[i][1][2], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acf4.plot(iterations[i], his_f_calc_rel[i][1][3], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acf5.plot(iterations[i], his_f_calc_rel[i][1][4], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acf6.plot(iterations[i], his_f_calc_rel[i][1][5], 'o', label="iteration " + str(i + 1), linewidth=3.0)


    # history of current gradient
    plt.figure("History of current gradient", figsize=(24, 12))
    plt.tight_layout()
    plt.subplot(2, 3, 1)
    acgc1 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("grad feature 1 [-]", fontsize=14)
    plt.grid(True)
    plt.title('grad of feature 1', fontsize=font, fontweight = 'bold')

    for tick in acgc1.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acgc1.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 2)
    acgc2 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("gradfeature 2 [-]", fontsize=14)
    plt.grid(True)
    plt.title('grad of feature 2', fontsize=font, fontweight = 'bold')

    for tick in acgc2.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acgc2.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 3)
    acgc3 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("grad feature 3 [-]", fontsize=14)
    plt.grid(True)
    plt.title('grad of feature 3', fontsize=font, fontweight = 'bold')

    for tick in acgc3.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acgc3.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 4)
    acgc4 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("grad feature 4 [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('grad of feature 4', fontsize=font, fontweight = 'bold')

    for tick in acgc4.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acgc4.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 5)
    acgc5 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("grad feature 5 [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('grad of feature 5', fontsize=font, fontweight = 'bold')

    for tick in acgc5.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acgc5.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 6)
    acgc6 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("grad feature 5 [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('grad of feature 6', fontsize=font, fontweight='bold')

    for tick in acgc6.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acgc6.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    iterations = plt.arange(1, len(his_grad_current) + 1, 1)
    for i in plt.arange(0, len(iterations), 1):
        acgc1.plot(iterations[i], his_grad_current[i][1][0], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acgc2.plot(iterations[i], his_grad_current[i][1][1], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acgc3.plot(iterations[i], his_grad_current[i][1][2], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acgc4.plot(iterations[i], his_grad_current[i][1][3], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acgc5.plot(iterations[i], his_grad_current[i][1][4], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acgc6.plot(iterations[i], his_grad_current[i][1][5], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        # ac7.plot(iterations[i], his_grad_current[i][1][6], 'o', label="iteration " + str(i + 1), linewidth=3.0)

    # Weights over iterations
    plt.figure("Weights",figsize=(24, 12))
    plt.tight_layout()
    plt.subplot(2, 3, 1)
    acw1 = plt.gca()
    # acw1 = plt.subplots(figsize=())
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Weight of feature 1',fontsize=font, fontweight = 'bold')

    for tick in acw1.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acw1.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 2)
    acw2 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Weight of feature 2', fontsize=font, fontweight = 'bold')

    for tick in acw2.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acw2.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 3)
    acw3 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Weight of feature 3', fontsize=font, fontweight = 'bold')

    for tick in acw3.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acw3.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 4)
    acw4 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Weight value [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('Weight of feature 4', fontsize=font, fontweight = 'bold')

    for tick in acw4.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acw4.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 5)
    acw5 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Weight value [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('Weight of feature 5', fontsize=font, fontweight = 'bold')

    for tick in acw5.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acw5.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 6)
    acw6 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Weight value [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('Weight of feature 6', fontsize=font, fontweight='bold')

    for tick in acw6.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acw6.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    iterations = plt.arange(1, len(his_weights) + 1, 1)
    for i in plt.arange(0,len(his_weights),1):
        acw1.plot(iterations[i], his_weights[i][1][0], 'o', label= "iteration "+str(i+1), linewidth=3.0)
        acw2.plot(iterations[i], his_weights[i][1][1], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acw3.plot(iterations[i], his_weights[i][1][2], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acw4.plot(iterations[i], his_weights[i][1][3], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acw5.plot(iterations[i], his_weights[i][1][4], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acw6.plot(iterations[i], his_weights[i][1][5], 'o', label="iteration " + str(i + 1), linewidth=3.0)


    # Plotting update of theta
    plt.figure("delta theta", figsize=(24, 12))
    plt.tight_layout()
    plt.subplot(2, 3, 1)
    acu1 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("delta weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Update of theta 1', fontsize=font, fontweight = 'bold')

    for tick in acu1.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acu1.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 2)
    acu2 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("delta weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Update of theta 2', fontsize=font, fontweight = 'bold')

    for tick in acu2.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acu2.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 3)
    acu3 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("delta weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Update of theta 3', fontsize=font, fontweight = 'bold')

    for tick in acu3.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acu3.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 4)
    acu4 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("delta weight value [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('Update of theta 4', fontsize=font, fontweight = 'bold')

    for tick in acu4.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acu4.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 5)
    acu5 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("delta weight value [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('Update of theta 5', fontsize=font, fontweight = 'bold')

    for tick in acu5.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acu5.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 6)
    acu6 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("delta weight value [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('Update of theta 6', fontsize=font, fontweight='bold')

    for tick in acu6.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acu6.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    iterations = plt.arange(2, len(his_weights)+1 + 1, 1)
    if len(his_weights) != 1:
        for i in plt.arange(1, len(his_weights), 1):
            acu1.plot(iterations[i], his_weights[i][1][0]-his_weights[i-1][1][0], 'o', label="iteration " + str(i + 1), linewidth=3.0)
            acu2.plot(iterations[i], his_weights[i][1][1]-his_weights[i-1][1][1], 'o', label="iteration " + str(i + 1), linewidth=3.0)
            acu3.plot(iterations[i], his_weights[i][1][2]-his_weights[i-1][1][2], 'o', label="iteration " + str(i + 1), linewidth=3.0)
            acu4.plot(iterations[i], his_weights[i][1][3]-his_weights[i-1][1][3], 'o', label="iteration " + str(i + 1), linewidth=3.0)
            acu5.plot(iterations[i], his_weights[i][1][4]-his_weights[i-1][1][4], 'o', label="iteration " + str(i + 1), linewidth=3.0)
            acu6.plot(iterations[i], his_weights[i][1][5]-his_weights[i-1][1][5], 'o', label="iteration " + str(i + 1), linewidth=3.0)
            # ac7.plot(iterations[i], his_weights[i][1][6]-his_weights[i-1][1][6], 'o', label="iteration " + str(i + 1), linewidth=3.0)

    # Multiplication history of the gradients
    plt.figure("Multiplication history of the gradients", figsize=(24, 12))
    plt.tight_layout()
    plt.subplot(2, 3, 1)
    acm1 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Multi grad feature 1 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Multi grad of feature 1', fontsize=font, fontweight = 'bold')

    for tick in acm1.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acm1.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 2)
    acm2 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Multi gradfeature 2 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Multi grad of feature 2', fontsize=font, fontweight = 'bold')

    for tick in acm2.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acm2.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 3)
    acm3 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Multi grad feature 3 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Multi grad of feature 3', fontsize=font, fontweight = 'bold')

    for tick in acm3.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acm3.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 4)
    acm4 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Multi grad feature 4 [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('Multi grad of feature 4', fontsize=font, fontweight = 'bold')

    for tick in acm4.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acm4.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 5)
    acm5 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Multi grad feature 5 [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('Multi grad of feature 5', fontsize=font, fontweight = 'bold')

    for tick in acm5.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acm5.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 6)
    acm6 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("Multi grad feature 5 [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('Multi grad of feature 6', fontsize=font, fontweight='bold')

    for tick in acm6.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acm6.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

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

    iterations = plt.arange(1, len(his_multi_grads) + 1 + 1, 1)
    for i in plt.arange(0, len(his_multi_grads), 1):
        acm1.plot(iterations[i], his_multi_grads[i][1][0], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acm2.plot(iterations[i], his_multi_grads[i][1][1], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acm3.plot(iterations[i], his_multi_grads[i][1][2], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acm4.plot(iterations[i], his_multi_grads[i][1][3], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acm5.plot(iterations[i], his_multi_grads[i][1][4], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acm6.plot(iterations[i], his_multi_grads[i][1][5], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        # ac7.plot(iterations[i], his_multi_grads[i][1][6], 'o', label="iteration " + str(i + 1), linewidth=3.0)

    # history of difference theta
    plt.figure("History of diff theta", figsize=(24, 12))
    plt.tight_layout()
    plt.subplot(2, 3, 1)
    acdw1 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("diff theta 1 [-]", fontsize=14)
    plt.grid(True)
    plt.title('diff theta 1', fontsize=font, fontweight = 'bold')

    for tick in acdw1.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acdw1.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 2)
    acdw2 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("diff theta 2 [-]", fontsize=14)
    plt.grid(True)
    plt.title('diff theta 2', fontsize=font, fontweight = 'bold')

    for tick in acdw2.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acdw2.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 3)
    acdw3 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("diff theta 3 [-]", fontsize=14)
    plt.grid(True)
    plt.title('diff theta 3', fontsize=font, fontweight = 'bold')

    for tick in acdw3.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acdw3.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 4)
    acdw4 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("diff theta 4 [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('diff theta 4', fontsize=font, fontweight = 'bold')

    for tick in acdw4.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acdw4.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 5)
    acdw5 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("diff theta 5 [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('diff theta 5', fontsize=font, fontweight = 'bold')

    for tick in acdw5.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acdw5.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(2, 3, 6)
    acdw6 = plt.gca()
    # plt.xlabel("Iteration [-]", fontsize=14)
    # plt.ylabel("diff theta 5 [-]", fontsize=14)
    plt.grid(True)
    plt.xlabel('diff theta 5', fontsize=font, fontweight='bold')

    for tick in acdw6.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acdw6.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

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
    iterations = plt.arange(1, len(his_diff_theta) + 1, 1)
    for i in plt.arange(0, len(his_diff_theta), 1):
        acdw1.plot(iterations[i], his_diff_theta[i][1][0], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acdw2.plot(iterations[i], his_diff_theta[i][1][1], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acdw3.plot(iterations[i], his_diff_theta[i][1][2], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acdw4.plot(iterations[i], his_diff_theta[i][1][3], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acdw5.plot(iterations[i], his_diff_theta[i][1][4], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        acdw6.plot(iterations[i], his_diff_theta[i][1][5], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        # ac7.plot(iterations[i], his_diff_theta[i][1][6], 'o', label="iteration " + str(i + 1), linewidth=3.0)
