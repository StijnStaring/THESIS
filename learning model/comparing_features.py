def comparing_features():
    import pylab as plt

    # Define plot to compare features
    plt.figure("Comparing Features")
    axf = plt.gca()
    plt.xlabel("Feature number", fontsize=14)
    plt.ylabel("Feature value", fontsize=14)
    plt.title("Comparing Features", fontsize=14)
    plt.grid(True)

    return axf