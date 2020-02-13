def comparing_features(dict_list):
    import pylab as plt

    data_cl = dict_list[0]
    # Define plot to compare normalized features
    plt.figure("Comparing Normalized Features")
    axf = plt.gca()
    plt.xlabel("Feature number", fontsize=14)
    plt.ylabel("Normalized Feature value", fontsize=14)
    plt.title("Comparing Features", fontsize=14)
    plt.grid(True)

    # Define plot to compare not normalized features
    plt.figure("Comparing Features")
    axfn = plt.gca()
    plt.xlabel("Feature number", fontsize=14)
    plt.ylabel("Feature value", fontsize=14)
    plt.title("Comparing Features", fontsize=14)
    plt.grid(True)

    # comparison of used weights

    plt.figure("Comparison weights", figsize=(10, 4))
    acw = plt.gca()
    plt.xlabel("Weight number", fontsize=14)
    plt.ylabel("Weight value", fontsize=14)
    plt.title("Comparison of weights over iterations", fontsize=14)
    plt.grid(True)

    # comparison of data and calculations
    #####################################

    # X(t)/Y(t)
    plt.figure("Comparison X(t) and Y(t)", figsize=(10, 4))
    plt.subplot(1, 2, 1)
    axcom1a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal distance [m]", fontsize=14)
    plt.grid(True)
    plt.title('Comparison dataset 1 with calculated x(t) 1', fontsize=14)

    plt.subplot(1, 2, 2)
    axcom1b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical distance [m]", fontsize=14)
    plt.title('Comparison dataset 1 with calculated y(t) 1', fontsize=14)
    plt.grid(True)

    # Path
    plt.figure("Comparison paths", figsize=(10, 4))
    axcom2 = plt.gca()
    plt.xlabel("x [m]", fontsize=14)
    plt.ylabel("y [m]", fontsize=14)
    plt.title('Comparison dataset 1 with calculated path 1', fontsize=14)
    plt.grid(True)


    # Vx(t)/Vy(t)

    plt.figure("Comparison velocities", figsize=(10, 4))
    plt.subplot(1, 2, 1)
    axcom3a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal velocity [m/s]", fontsize=14)
    plt.grid(True)
    plt.title('Vx(t) comparison', fontsize=14)

    plt.subplot(1, 2, 2)
    axcom3b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical velocity [m/s]", fontsize=14)
    plt.title('Vy(t) comparison', fontsize=14)
    plt.grid(True)

    # Ax(t)/Ay(t)
    plt.figure("Comparison accelerations", figsize=(10, 4))
    plt.subplot(1, 2, 1)
    axcom4a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal acceleration [m/s^2]", fontsize=14)
    plt.grid(True)
    plt.title('Ax(t) comparison', fontsize=14)

    plt.subplot(1, 2, 2)
    axcom4b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical acceleration [m/s^2]", fontsize=14)
    plt.title('Ay(t) comparison', fontsize=14)
    plt.grid(True)

    # Jx(t)/Jy(t)

    plt.figure("Comparison jerks", figsize=(10, 4))
    plt.subplot(1, 2, 1)
    axcom5a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal jerk [m/s^3]", fontsize=14)
    plt.grid(True)
    plt.title('jx(t) comparison', fontsize=14)

    plt.subplot(1, 2, 2)
    axcom5b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical jerk [m/s^3]", fontsize=14)
    plt.title('jy(t) comparison', fontsize=14)
    plt.grid(True)

    # ux(t)/uy(t)

    plt.figure("Comparison jounce", figsize=(10, 4))
    plt.subplot(1, 2, 1)
    axcom6a = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Horizontal jounce [m/s^4]", fontsize=14)
    plt.grid(True)
    plt.title('ux(t) comparison', fontsize=14)

    plt.subplot(1, 2, 2)
    axcom6b = plt.gca()
    plt.xlabel("Time [s]", fontsize=14)
    plt.ylabel("Vertical jounce [m/s^4]", fontsize=14)
    plt.title('uy(t) comparison', fontsize=14)
    plt.grid(True)

    # Curvature
    plt.figure("Comparison curvature", figsize=(10, 4))
    axcom7 = plt.gca()
    plt.xlabel("t [s]", fontsize=14)
    plt.ylabel("Curvature [1/m]", fontsize=14)
    plt.title('Curvature comparison', fontsize=14)
    plt.grid(True)

    # Plotting data in figures
    axcom1a.plot(data_cl['time_cl'], data_cl['x_cl'], '-', label="X(t) 1 (data)", linewidth=3.0)
    axcom1b.plot(data_cl['time_cl'], data_cl['y_cl'], '-', label="Y(t) 1 (data)", linewidth=3.0)

    axcom2.plot(data_cl['x_cl'], data_cl['y_cl'], '-', label="path 1 (data)", linewidth=3.0)

    # The vx and vy velocities are as seen in the global axis (fixed).
    axcom3a.plot(data_cl['time_cl'], data_cl['vx_proj_cl'], '-', label="Vx(t) 1 (data)", linewidth=3.0)
    axcom3b.plot(data_cl['time_cl'], data_cl['vy_proj_cl'], '-', label="Vy(t) 1 (data)", linewidth=3.0)

    # The ax and ay accelerations are as seen in the global axis (fixed).
    axcom4a.plot(data_cl['time_cl'], data_cl['ax_proj_cl'], '-', label="Ax(t) 1 (data)", linewidth=3.0)
    axcom4b.plot(data_cl['time_cl'], data_cl['ay_proj_cl'], '-', label="Ay(t) 1 (data)", linewidth=3.0)

    # The jerk_x and jerk_y are as seen in the global axis (fixed).
    axcom5a.plot(data_cl['time_cl'], data_cl['jx_cl'], '-', label="Jx(t) 1 (data)", linewidth=3.0)
    axcom5b.plot(data_cl['time_cl'], data_cl['jy_cl'], '-', label="Jy(t) 1 (data)", linewidth=3.0)

    # The curvature
    axcom7.plot(data_cl['time_cl'], data_cl['curvature_cl'], '-', label="Curvature 1 (data)", linewidth=3.0)

    return axf,acw, axfn, axcom1a,axcom1b,axcom2,axcom3a,axcom3b,axcom4a,axcom4b,axcom5a,axcom5b,axcom6a,axcom6b,axcom7