def define_plots_PR():
    """"
    Theta_iter is a string
    """
    import pylab as plt


    # X(t)/Y(t)
    plt.figure("Path vs Time: iter " ,figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax1a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal distance [m]", fontsize=14)
    plt.grid(True)
    plt.title('x(t) global',fontsize=14)

    plt.subplot(1, 2, 2)
    ax1b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical distance [m]", fontsize=14)
    plt.title('y(t) global',fontsize=14)
    plt.grid(True)

    # path
    plt.figure("Path: iter ")
    ax2 = plt.gca()
    plt.xlabel("x [m]",fontsize=14)
    plt.ylabel("y [m]",fontsize=14)
    plt.title('Path global [m] ',fontsize=14)
    plt.grid(True)

    # Vx(t)/Vy(t)

    plt.figure("Speeds: iter ",figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax3a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal velocity [m/s]", fontsize=14)
    plt.grid(True)
    plt.title('Vx(t) local',fontsize=14)

    plt.subplot(1, 2, 2)
    ax3b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical velocity [m/s]", fontsize=14)
    plt.title('Vy(t) local',fontsize=14)
    plt.grid(True)


    # Ax(t)/Ay(t)
    plt.figure("Accelerations: iter ",figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax4a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal acceleration [m/s^2]", fontsize=14)
    plt.grid(True)
    plt.title('Ax(t) local total',fontsize=14)

    plt.subplot(1, 2, 2)
    ax4b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical acceleration [m/s^2]", fontsize=14)
    plt.title('Ay(t) local total',fontsize=14)
    plt.grid(True)

    # Jx(t)/Jy(t)

    plt.figure("Jerks: iter ",figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax5a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal jerk [m/s^3]", fontsize=14)
    plt.grid(True)
    plt.title('jx(t) local total',fontsize=14)

    plt.subplot(1, 2, 2)
    ax5b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical jerk [m/s^3]", fontsize=14)
    plt.title('jy(t) local total',fontsize=14)
    plt.grid(True)

    # yaw(t)/yaw/s(t)

    plt.figure("Yaw angle: iter ",figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax6a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Yaw angle [degrees]", fontsize=14)
    plt.grid(True)
    plt.title('Yaw angle ',fontsize=14)

    plt.subplot(1, 2, 2)
    ax6b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Yaw angle/s [degrees/s]", fontsize=14)
    plt.title('Yaw angle/s ',fontsize=14)
    plt.grid(True)

    # Inputs
    plt.figure("throttle: iter " , figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax7a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("throttle [-]", fontsize=14)
    plt.grid(True)
    plt.title('throttle ', fontsize=14)

    plt.subplot(1, 2, 2)
    ax7b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("delta [degrees]", fontsize=14)
    plt.title('delta local', fontsize=14)
    plt.grid(True)

    # Different parts of the local lateral acceleration
    plt.figure("Ayt: iter " , figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax8a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Ayt [m/s^2]", fontsize=14)
    plt.grid(True)
    plt.title('Ayt ', fontsize=14)

    plt.subplot(1, 2, 2)
    ax8b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Ayn [m/s^2]", fontsize=14)
    plt.grid(True)
    plt.title('Ayn', fontsize=14)

    # X(t)/Y(t)
    plt.figure("error x/y ", figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ex1a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal distance error [m]", fontsize=14)
    plt.grid(True)
    plt.title('x(t) global', fontsize=14)

    plt.subplot(1, 2, 2)
    ex1b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical distance error [m]", fontsize=14)
    plt.title('y(t) global', fontsize=14)
    plt.grid(True)

    # Vx(t)/Vy(t)

    plt.figure("error vx/vy", figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ex3a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal velocity error [m/s]", fontsize=14)
    plt.grid(True)
    plt.title('Vx(t) local', fontsize=14)

    plt.subplot(1, 2, 2)
    ex3b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical velocity error [m/s]", fontsize=14)
    plt.title('Vy(t) local', fontsize=14)
    plt.grid(True)

    # yaw(t)/yaw/s(t)

    plt.figure("Yaw angle/ yaw rate error ",figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ex6a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Yaw angle error [degrees]", fontsize=14)
    plt.grid(True)
    plt.title('Yaw angle ',fontsize=14)

    plt.subplot(1, 2, 2)
    ex6b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Yaw angle/s error [degrees/s]", fontsize=14)
    plt.title('Yaw angle/s ',fontsize=14)
    plt.grid(True)

    # time_vector = plt.linspace(0,T_sol,len(x_sol))

    return ax1a,ax1b,ax2,ax3a,ax3b,ax4a,ax4b,ax5a,ax5b,ax6a,ax6b,ax7a,ax7b,ax8a,ax8b, ex1a,ex1b,ex3a,ex3b,ex6a,ex6b
