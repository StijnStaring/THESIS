def plotting_datasets():
    """"
        Theta_iter is a string
    """
    import pylab as plt

    # X(t)/Y(t)
    plt.figure("Path vs Time" , figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax1a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal distance [m]", fontsize=14)
    plt.grid(True)
    plt.title('x(t) dataset (global)', fontsize=14)

    plt.subplot(1, 2, 2)
    ax1b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical distance [m]", fontsize=14)
    plt.title('y(t) dataset (global)', fontsize=14)
    plt.grid(True)

    # Path
    plt.figure("Path")
    ax2 = plt.gca()
    plt.xlabel("x [m]", fontsize=14)
    plt.ylabel("y [m]", fontsize=14)
    plt.title('Path [m] dataset (global)', fontsize=14)
    plt.grid(True)

    # Vx(t)/Vy(t)
    plt.figure("Speeds", figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax3a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal velocity [m/s]", fontsize=14)
    plt.grid(True)
    plt.title('Vx(t) dataset (global)', fontsize=14)

    plt.subplot(1, 2, 2)
    ax3b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical velocity [m/s]", fontsize=14)
    plt.title('Vy(t) dataset (global)', fontsize=14)
    plt.grid(True)

    # Ax(t)/Ay(t)
    plt.figure("Accelerations", figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax4a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal acceleration [m/s^2]", fontsize=14)
    plt.grid(True)
    plt.title('Ax(t) dataset (global)', fontsize=14)

    plt.subplot(1, 2, 2)
    ax4b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical acceleration [m/s^2]", fontsize=14)
    plt.title('Ay(t) dataset (global)', fontsize=14)
    plt.grid(True)

    # Jx(t)/Jy(t)

    plt.figure("Jerks", figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax5a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal jerk [m/s^3]", fontsize=14)
    plt.grid(True)
    plt.title('jx(t) dataset (global)', fontsize=14)

    plt.subplot(1, 2, 2)
    ax5b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical jerk [m/s^3]", fontsize=14)
    plt.title('jy(t) dataset (global)', fontsize=14)
    plt.grid(True)

    # yaw(t)/r(t) -> radians should be transformed to degrees
    plt.figure("Yaws vs Time" , figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax6a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("[degrees]", fontsize=14)
    plt.grid(True)
    plt.title('yaw(t) dataset', fontsize=14)

    plt.subplot(1, 2, 2)
    ax6b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("[degrees/s]", fontsize=14)
    plt.title('yaw(t)/s dataset', fontsize=14)
    plt.grid(True)

    # steerwheelangle -> is already given in degrees
    plt.figure("Steerwheelangle vs Time" , figsize=(10, 4))
    ax7 = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("[degrees]", fontsize=14)
    plt.grid(True)
    plt.title('Steerwheelangle dataset', fontsize=14)

    # Curvature
    plt.figure("Curvature")
    ax8 = plt.gca()
    plt.xlabel("t [s]", fontsize=14)
    plt.ylabel("Curvature [1/m]", fontsize=14)
    plt.title('Curvature of the dataset (global)', fontsize=14)
    plt.grid(True)

    return ax1a, ax1b, ax2, ax3a, ax3b, ax4a, ax4b, ax5a, ax5b, ax6a, ax6b, ax7, ax8

