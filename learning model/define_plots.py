def define_plots(theta_iter,x_sol,y_sol,vx_sol,vy_sol,ax_sol,ay_sol,jx_sol,jy_sol,psi_sol,psi_dot_sol,throttle_sol,delta_sol,T_sol):
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
    plt.title('x(t) global',fontsize=14)

    plt.subplot(1, 2, 2)
    ax1b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical distance [m]", fontsize=14)
    plt.title('y(t) global',fontsize=14)
    plt.grid(True)

    # path
    plt.figure("Path: iter "+ theta_iter)
    ax2 = plt.gca()
    plt.xlabel("x [m]",fontsize=14)
    plt.ylabel("y [m]",fontsize=14)
    plt.title('Path global [m] ',fontsize=14)
    plt.grid(True)

    # Vx(t)/Vy(t)

    plt.figure("Speeds: iter "+ theta_iter,figsize=(10, 4))
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
    plt.figure("Accelerations: iter "+ theta_iter,figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax4a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal acceleration [m/s^2]", fontsize=14)
    plt.grid(True)
    plt.title('Ax(t) local',fontsize=14)

    plt.subplot(1, 2, 2)
    ax4b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical acceleration [m/s^2]", fontsize=14)
    plt.title('Ay(t) local',fontsize=14)
    plt.grid(True)

    # Jx(t)/Jy(t)

    plt.figure("Jerks: iter "+ theta_iter,figsize=(10, 4))
    plt.subplot(1, 2, 1)
    ax5a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal jerk [m/s^3]", fontsize=14)
    plt.grid(True)
    plt.title('jx(t) local',fontsize=14)

    plt.subplot(1, 2, 2)
    ax5b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical jerk [m/s^3]", fontsize=14)
    plt.title('jy(t) local',fontsize=14)
    plt.grid(True)

    # yaw(t)/yaw/s(t)

    plt.figure("Yaw angle: iter "+ theta_iter,figsize=(10, 4))
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
    plt.figure("throttle: iter " + theta_iter, figsize=(10, 4))
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

    time_vector = plt.linspace(0,T_sol,len(x_sol))

    # states
    ax1a.plot(time_vector, x_sol, '.-', linewidth=3.0)
    ax1b.plot(time_vector, y_sol, '.-',  linewidth=3.0)
    ax2.plot(x_sol, y_sol, '.-',  linewidth=3.0)
    ax3a.plot(time_vector, vx_sol, '.-',  linewidth=3.0)
    ax3b.plot(time_vector, vy_sol, '.-',  linewidth=3.0)
    ax4a.plot(time_vector[0:-1], ax_sol, '.-',  linewidth=3.0)
    ax4b.plot(time_vector[0:-1], ay_sol, '.-',  linewidth=3.0)
    ax5a.plot(time_vector[0:-1], jx_sol, '.-', linewidth=3.0)
    ax5b.plot(time_vector[0:-1], jy_sol, '.-', linewidth=3.0)
    ax6a.plot(time_vector, psi_sol*180/plt.pi, '.-', linewidth=3.0)
    ax6b.plot(time_vector, psi_dot_sol*180/plt.pi, '.-', linewidth=3.0)
    ax7a.plot(time_vector[0:-1], throttle_sol, '.-', linewidth=3.0)
    ax7b.plot(time_vector[0:-1], delta_sol*180/plt.pi, '.-', linewidth=3.0)


    # ax1a.legend()
    # ax1b.legend()
    # ax2.legend()
    # ax3a.legend()
    # ax3b.legend()
    # ax4a.legend()
    # ax4b.legend()
    # ax5a.legend()
    # ax5b.legend()
    # ax6a.legend()
    # ax6b.legend()
    # ax7a.legend()
    # ax7b.legend()

    print("\n")
    print("Lane change duration of: ",T_sol," [s]")
