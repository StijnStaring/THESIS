def figure_style_saving():

    import pylab as plt
    # Save figures and CSV file - still to be implemented (use solutions array for csv file)
    # call figure again and then save it
    plt.figure("Comparing Normalized Features")
    fname = "results/norm_features.png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w',orientation='portrait', papertype=None, format=None,transparent=False, bbox_inches=None, pad_inches=0.1,metadata=None)

    plt.figure("Comparing Features")
    fname = "results/features.png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None,
                transparent=False, bbox_inches=None, pad_inches=0.1, metadata=None)

    plt.figure("Comparison Weights")
    fname = "results/comparison_weights.png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None,
                transparent=False, bbox_inches=None, pad_inches=0.1, metadata=None)

    plt.figure("Comparison X(t) and Y(t)")
    fname = "results/x(t)_y(t).png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None,
                transparent=False, bbox_inches=None, pad_inches=0.1, metadata=None)

    plt.figure("Comparison paths")
    fname = "results/path.png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None,
                transparent=False, bbox_inches=None, pad_inches=0.1, metadata=None)

    plt.figure("Comparison velocities")
    fname = "results/vx(t)_vy(t).png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None,
                transparent=False, bbox_inches=None, pad_inches=0.1, metadata=None)

    plt.figure("Comparison accelerations")
    fname = "results/ax(t)_ay(t).png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None,
                transparent=False, bbox_inches=None, pad_inches=0.1, metadata=None)

    plt.figure("Comparison jerks")
    fname = "results/jX(t)_jY(t).png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None,
                transparent=False, bbox_inches=None, pad_inches=0.1, metadata=None)

    plt.figure("Comparison Yaw angle")
    fname = "results/psi(t)_psi_dot(t).png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None,
                transparent=False, bbox_inches=None, pad_inches=0.1, metadata=None)

    plt.figure("Comparison throttle")
    fname = "results/inputs.png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None,
                transparent=False, bbox_inches=None, pad_inches=0.1, metadata=None)

    plt.figure("Ayt")
    fname = "results/ay(t).png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None,
                transparent=False, bbox_inches=None, pad_inches=0.1, metadata=None)

    plt.figure("Yaw acceleration")
    fname = "results/psi_ddot(t).png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None,
                transparent=False, bbox_inches=None, pad_inches=0.1, metadata=None)

    plt.figure("Convergence of features")
    fname = "results/fdiff_rel.png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None,
                transparent=False, bbox_inches=None, pad_inches=0.1, metadata=None)

    plt.figure("History of current gradient")
    fname = "results/fdiff_abs.png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None,
                transparent=False, bbox_inches=None, pad_inches=0.1, metadata=None)

    plt.figure("Weights")
    fname = "results/weights.png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None,
                transparent=False, bbox_inches=None, pad_inches=0.1, metadata=None)

    plt.figure("Weights")
    fname = "results/delta_weight.png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None,
                transparent=False, bbox_inches=None, pad_inches=0.1, metadata=None)

    plt.figure("Multiplication history of the gradients")
    fname = "results/RPROP_case.png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None,
                transparent=False, bbox_inches=None, pad_inches=0.1, metadata=None)

    plt.figure("History of diff theta")
    fname = "results/diff_chosen_weights.png"
    plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w', orientation='portrait', papertype=None, format=None,
                transparent=False, bbox_inches=None, pad_inches=0.1, metadata=None)

    # plt.rc('axes', linewidth=2)
    # plt.rc('legend',fontsize=12) # using a size in points
    # # plt.rcParams["legend.loc"] = 'best'
    # fontsize = 16
    # plt.figure("Thickness test", figsize=(10, 4))
    # ax = plt.gca()
    # plt.xlabel('xlable',fontsize = 16, fontweight = 'bold')
    # plt.ylabel('ylable',fontsize = 16, fontweight = 'bold')
    # ax.grid(True)
    # ax.plot([0,1],[0,1],'-',linewidth = 3.0,label = 'example_8')
    # for tick in ax.xaxis.get_major_ticks():
    #     tick.label1.set_fontsize(fontsize)
    #     tick.label1.set_fontweight('bold')
    # for tick in ax.yaxis.get_major_ticks():
    #     tick.label1.set_fontsize(fontsize)
    #     tick.label1.set_fontweight('bold')
    # ax.legend()

