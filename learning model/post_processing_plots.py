def post_processing_plots(his_f_calc_rel,his_weights):
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

    plt.subplot(2, 4, 1)
    ac1 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Normalized calculated feature 1 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Convergence of features 1',fontsize=14)

    plt.subplot(2, 4, 2)
    ac2 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Normalized calculated feature 2 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Convergence of features 2', fontsize=14)

    plt.subplot(2, 4, 3)
    ac3 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Normalized calculated feature 3 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Convergence of features 3', fontsize=14)

    plt.subplot(2, 4, 4)
    ac4 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Normalized calculated feature 4 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Convergence of features 4', fontsize=14)

    plt.subplot(2, 4, 5)
    ac5 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Normalized calculated feature 5 [-]", fontsize=14)
    plt.grid(True)
    plt.title('Convergence of features 5', fontsize=14)

    for i in plt.arange(0,length,1):
        ac1.plot(iterations[i], his_f_calc_rel[i][1][0], 'o', label= "iteration "+str(i+1), linewidth=3.0)
        ac2.plot(iterations[i], his_f_calc_rel[i][1][1], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac3.plot(iterations[i], his_f_calc_rel[i][1][2], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac4.plot(iterations[i], his_f_calc_rel[i][1][3], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac5.plot(iterations[i], his_f_calc_rel[i][1][4], 'o', label="iteration " + str(i + 1), linewidth=3.0)



# Weights over iterations
    plt.figure("Weights",figsize=(10, 4))

    plt.subplot(2, 4, 1)
    ac1 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Weight of feature 1',fontsize=14)

    plt.subplot(2, 4, 2)
    ac2 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Weight of feature 2', fontsize=14)

    plt.subplot(2, 4, 3)
    ac3 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Weight of feature 3', fontsize=14)

    plt.subplot(2, 4, 4)
    ac4 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Weight of feature 4', fontsize=14)

    plt.subplot(2, 4, 5)
    ac5 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("Weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Weight of feature 5', fontsize=14)

    for i in plt.arange(0,length,1):
        ac1.plot(iterations[i], his_weights[i][1][0], 'o', label= "iteration "+str(i+1), linewidth=3.0)
        ac2.plot(iterations[i], his_weights[i][1][1], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac3.plot(iterations[i], his_weights[i][1][2], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac4.plot(iterations[i], his_weights[i][1][3], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac5.plot(iterations[i], his_weights[i][1][4], 'o', label="iteration " + str(i + 1), linewidth=3.0)


    # Plotting update of theta
    plt.figure("delta theta", figsize=(10, 4))
    plt.subplot(2, 4, 1)
    ac1 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("delta weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Update of theta 1', fontsize=14)

    plt.subplot(2, 4, 2)
    ac2 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("delta weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Update of theta 2', fontsize=14)

    plt.subplot(2, 4, 3)
    ac3 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("delta weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Update of theta 3', fontsize=14)

    plt.subplot(2, 4, 4)
    ac4 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("delta weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Update of theta 4', fontsize=14)

    plt.subplot(2, 4, 5)
    ac5 = plt.gca()
    plt.xlabel("Iteration [-]", fontsize=14)
    plt.ylabel("delta weight value [-]", fontsize=14)
    plt.grid(True)
    plt.title('Update of theta 5', fontsize=14)



    for i in plt.arange(1, length, 1):
        ac1.plot(iterations[i], his_weights[i][1][0]-his_weights[i-1][1][0], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac2.plot(iterations[i], his_weights[i][1][1]-his_weights[i-1][1][1], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac3.plot(iterations[i], his_weights[i][1][2]-his_weights[i-1][1][2], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac4.plot(iterations[i], his_weights[i][1][3]-his_weights[i-1][1][3], 'o', label="iteration " + str(i + 1), linewidth=3.0)
        ac5.plot(iterations[i], his_weights[i][1][4]-his_weights[i-1][1][4], 'o', label="iteration " + str(i + 1), linewidth=3.0)


