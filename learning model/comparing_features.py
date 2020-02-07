def comparing_features(theta_iter,dict_list):
    import pylab as plt

    data_cl = dict_list[0]
    # Define plot to compare features
    plt.figure("Comparing Features")
    axf = plt.gca()
    plt.xlabel("Feature number", fontsize=14)
    plt.ylabel("Feature value", fontsize=14)
    plt.title("Comparing Features", fontsize=14)
    plt.grid(True)

    # comparison of data path and calculated path

    plt.figure("Comparison paths", figsize=(10, 4))
    ax7 = plt.gca()
    ax7.plot(data_cl['x_cl'], data_cl['y_cl'], '-', label="path 1 (data)", linewidth=3.0)
    plt.xlabel("x [m]", fontsize=14)
    plt.ylabel("y [m]", fontsize=14)
    plt.title('"Comparison dataset 1 with calculated path 1', fontsize=14)
    plt.grid(True)

    # comparison of used weights

    plt.figure("Comparison weights", figsize=(10, 4))
    acw = plt.gca()
    plt.xlabel("Weight number", fontsize=14)
    plt.ylabel("Weight value", fontsize=14)
    plt.title("Comparison of weights over iterations", fontsize=14)
    plt.grid(True)

    return axf, ax7, acw