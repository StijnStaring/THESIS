def define_plots(theta_iter):
    """"
    Theta_iter is a string
    """
    import pylab as plt

    # X(t)/Y(t)
    plt.figure("Path vs Time: iter " + theta_iter,figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax1a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal distance [m]", fontsize=14)
    plt.grid(True)
    plt.title('x(t) calculated',fontsize=14)

    plt.subplot(1, 2, 2)
    ax1b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical distance [m]", fontsize=14)
    plt.title('y(t) calculated',fontsize=14)
    plt.grid(True)

    # sphinx_gallery_thumbnail_number = 2

    plt.figure("Path: iter "+ theta_iter)
    ax2 = plt.gca()
    plt.xlabel("x [m]",fontsize=14)
    plt.ylabel("y [m]",fontsize=14)
    plt.title('Path [m] calculated',fontsize=14)
    plt.grid(True)

    # Vx(t)/Vy(t)

    plt.figure("Speeds: iter "+ theta_iter,figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax3a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal velocity [m/s]", fontsize=14)
    plt.grid(True)
    plt.title('Vx(t) calculated',fontsize=14)

    plt.subplot(1, 2, 2)
    ax3b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical velocity [m/s]", fontsize=14)
    plt.title('Vy(t) calculated',fontsize=14)
    plt.grid(True)


    # Ax(t)/Ay(t)
    plt.figure("Accelerations: iter "+ theta_iter,figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax4a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal acceleration [m/s^2]", fontsize=14)
    plt.grid(True)
    plt.title('Ax(t) calculated',fontsize=14)

    plt.subplot(1, 2, 2)
    ax4b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical acceleration [m/s^2]", fontsize=14)
    plt.title('Ay(t) calculated',fontsize=14)
    plt.grid(True)

    # Jx(t)/Jy(t)

    plt.figure("Jerks: iter "+ theta_iter,figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax5a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal jerk [m/s^3]", fontsize=14)
    plt.grid(True)
    plt.title('jx(t) calculated',fontsize=14)

    plt.subplot(1, 2, 2)
    ax5b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical jerk [m/s^3]", fontsize=14)
    plt.title('jy(t) calculated',fontsize=14)
    plt.grid(True)

    return ax1a,ax1b,ax2,ax3a,ax3b,ax4a,ax4b,ax5a,ax5b
