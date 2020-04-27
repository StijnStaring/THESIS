def comparing_features(data_cl,file_name):
    import pylab as plt

    plt.rc('axes', linewidth=2)
    plt.rc('legend', fontsize=16)
    # plt.rcParams["legend.loc"] = 'best'
    fontsize = 14
    font = 16

    # Define plot to compare normalized features
    plt.figure("Comparing Normalized Features")
    axf = plt.gca()
    plt.xlabel("Feature number", fontsize=font,fontweight='bold')
    plt.ylabel("Normalized Feature value", fontsize=font,fontweight='bold')
    plt.title("Comparing Features", fontsize=font,fontweight='bold')
    plt.grid(True)

    for tick in axf.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axf.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    # Define plot to compare not normalized features
    plt.figure("Comparing Features")
    axfn = plt.gca()
    plt.xlabel("Feature number", fontsize=font,fontweight='bold')
    plt.ylabel("Feature value", fontsize=font,fontweight='bold')
    plt.title("Comparing Features", fontsize=font,fontweight='bold')
    plt.grid(True)

    for tick in axfn.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axfn.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    # comparison of used weights
    plt.figure("Comparison weights", figsize=(10, 4))
    acw = plt.gca()
    plt.xlabel("Weight number", fontsize=font,fontweight='bold')
    plt.ylabel("Weight value", fontsize=font,fontweight='bold')
    plt.title("Comparison of weights over iterations", fontsize=font,fontweight='bold')
    plt.grid(True)

    for tick in acw.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in acw.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    # comparison of data and calculations
    #####################################

    # X(t)/Y(t)
    plt.figure("Comparison X(t) and Y(t)", figsize=(10, 4))
    plt.subplot(1, 2, 1)
    axcom1a = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("Horizontal distance [m]", fontsize=font,fontweight='bold')
    plt.grid(True)
    # plt.title('Comparison dataset 1 with calculated x(t) 1', fontsize=font,fontweight='bold')

    for tick in axcom1a.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom1a.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(1, 2, 2)
    axcom1b = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("Vertical distance [m]", fontsize=font,fontweight='bold')
    # plt.title('Comparison observed and calculated y(t)', fontsize=font,fontweight='bold')
    plt.grid(True)

    for tick in axcom1b.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom1b.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    # Path
    plt.figure("Comparison paths", figsize=(10, 4))
    axcom2 = plt.gca()
    plt.xlabel("x [m]", fontsize=font,fontweight='bold')
    plt.ylabel("y [m]", fontsize=font,fontweight='bold')
    # plt.title('Comparison dataset 1 with calculated path 1', fontsize=font,fontweight='bold')
    plt.grid(True)

    for tick in axcom2.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom2.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')


    # Vx(t)/Vy(t)

    plt.figure("Comparison velocities", figsize=(10, 4))
    plt.subplot(1, 2, 1)
    axcom3a = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("Horizontal velocity [m/s]", fontsize=font,fontweight='bold')
    plt.grid(True)
    # plt.title('Vx(t) comparison', fontsize=font,fontweight='bold')

    for tick in axcom3a.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom3a.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(1, 2, 2)
    axcom3b = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("Vertical velocity [m/s]", fontsize=font,fontweight='bold')
    # plt.title('Vy(t) comparison', fontsize=font,fontweight='bold')
    plt.grid(True)

    for tick in axcom3b.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom3b.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    # Ax(t)/Ay(t)
    plt.figure("Comparison accelerations", figsize=(10, 4))
    plt.subplot(1, 2, 1)
    axcom4a = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("Horizontal acceleration [m/s^2]", fontsize=font,fontweight='bold')
    plt.grid(True)
    # plt.title('Ax(t) comparison (total)', fontsize=font,fontweight='bold')

    for tick in axcom4a.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom4a.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(1, 2, 2)
    axcom4b = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("Vertical acceleration [m/s^2]", fontsize=font,fontweight='bold')
    # plt.title('Ay(t) comparison (total)', fontsize=font,fontweight='bold')
    plt.grid(True)

    for tick in axcom4b.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom4b.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    # Jx(t)/Jy(t)

    plt.figure("Comparison jerks", figsize=(10, 4))
    plt.subplot(1, 2, 1)
    axcom5a = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("Horizontal jerk [m/s^3]", fontsize=font,fontweight='bold')
    plt.grid(True)
    # plt.title('jx(t) comparison (total)', fontsize=font,fontweight='bold')

    for tick in axcom5a.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom5a.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(1, 2, 2)
    axcom5b = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("Vertical jerk [m/s^3]", fontsize=font,fontweight='bold')
    # plt.title('jy(t) comparison (total)', fontsize=font,fontweight='bold')
    plt.grid(True)

    for tick in axcom5b.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom5b.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    # yaw(t)/yaw/s(t)

    plt.figure("Yaw angle", figsize=(10, 4))
    plt.subplot(1, 2, 1)
    axcom6a = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("Yaw angle [degrees]", fontsize=font,fontweight='bold')
    plt.grid(True)
    # plt.title('Yaw angle ', fontsize=font,fontweight='bold')

    for tick in axcom6a.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom6a.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(1, 2, 2)
    axcom6b = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("Yaw angle/s [degrees/s]", fontsize=font,fontweight='bold')
    # plt.title('Yaw angle/s ', fontsize=font,fontweight='bold')
    plt.grid(True)

    for tick in axcom6b.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom6b.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    # Inputs
    plt.figure("throttle", figsize=(10, 4))
    plt.subplot(1, 2, 1)
    axcom7a = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("throttle [-]", fontsize=font,fontweight='bold')
    plt.grid(True)
    # plt.title('throttle ', fontsize=font,fontweight='bold')

    for tick in axcom7a.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom7a.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(1, 2, 2)
    axcom7b = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("delta [degrees]", fontsize=font,fontweight='bold')
    # plt.title('delta local', fontsize=font,fontweight='bold')
    plt.grid(True)

    for tick in axcom7b.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom7b.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    # Different parts of the local lateral acceleration
    plt.figure("Ayt", figsize=(10, 4))
    plt.subplot(1, 2, 1)
    axcom8a = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("Ayt [m/s^2]", fontsize=font,fontweight='bold')
    plt.grid(True)
    # plt.title('Ayt ', fontsize=font,fontweight='bold')

    for tick in axcom8a.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom8a.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(1, 2, 2)
    axcom8b = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("Ayn [m/s^2]", fontsize=font,fontweight='bold')
    plt.grid(True)
    # plt.title('Ayn', fontsize=font,fontweight='bold')

    for tick in axcom8b.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom8b.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    # yaw acceleration

    plt.figure("Yaw acceleration", figsize=(10, 4))
    axcom9 = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("Yaw acceleration [degrees/s^2]", fontsize=font,fontweight='bold')
    plt.grid(True)
    # plt.title('Yaw acceleration ', fontsize=font,fontweight='bold')

    for tick in axcom9.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom9.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')



    # Plotting data in figures
    axcom1a.plot(data_cl['time_cl'], data_cl['x_cl'], '-', label="X(t) "+file_name[16:-4], linewidth=3.0)
    axcom1b.plot(data_cl['time_cl'], data_cl['y_cl'], '-', label="Y(t) "+file_name[16:-4], linewidth=3.0)

    axcom2.plot(data_cl['x_cl'], data_cl['y_cl'], '-', label="path "+file_name[16:-4], linewidth=3.0)

    # The vx and vy velocities are as seen in the global axis (fixed).
    axcom3a.plot(data_cl['time_cl'], data_cl['vx_cl'], '-', label="Vx(t) "+file_name[16:-4], linewidth=3.0)
    axcom3b.plot(data_cl['time_cl'], data_cl['vy_cl'], '-', label="Vy(t) "+file_name[16:-4], linewidth=3.0)

    # The ax and ay accelerations are as seen in the global axis (fixed).
    axcom4a.plot(data_cl['time_cl'], data_cl['ax_cl'], '-', label="Ax(t) "+file_name[16:-4], linewidth=3.0)
    axcom4b.plot(data_cl['time_cl'], data_cl['ay_cl'], '-', label="Ay(t) "+file_name[16:-4], linewidth=3.0)

    # The jerk_x and jerk_y are as seen in the global axis (fixed).
    axcom5a.plot(data_cl['time_cl'], data_cl['jx_cl'], '-', label="Jx(t) "+file_name[16:-4], linewidth=3.0)
    axcom5b.plot(data_cl['time_cl'], data_cl['jy_cl'], '-', label="Jy(t) "+file_name[16:-4], linewidth=3.0)

    # yaw and yaw rate
    axcom6a.plot(data_cl['time_cl'], data_cl['psi_cl']* 180 / plt.pi, '-', label="Psi(t) "+file_name[16:-4], linewidth=3.0)
    axcom6b.plot(data_cl['time_cl'], data_cl['psi_dot_cl']* 180 / plt.pi, '-', label="Psi_dot(t) "+file_name[16:-4], linewidth=3.0)

    # throttle and delta and yaw rate
    axcom7a.plot(data_cl['time_cl'], data_cl['throttle_cl'] , '-', label="Throttle(t) "+file_name[16:-4], linewidth=3.0)
    axcom7b.plot(data_cl['time_cl'], data_cl['delta_cl'] * 180 / plt.pi, '-', label="Delta(t) "+file_name[16:-4],linewidth=3.0)

    # throttle and delta and yaw rate
    axcom8a.plot(data_cl['time_cl'], data_cl['aty_cl'], '-', label="Aty(t) "+file_name[16:-4], linewidth=3.0)
    axcom8b.plot(data_cl['time_cl'], data_cl['any_cl'], '-', label="Any(t) "+file_name[16:-4], linewidth=3.0)

    # yaw rate aceleration
    axcom9.plot(data_cl['time_cl'], data_cl['psi_ddot_cl']*180/plt.pi, '-', label="Psi_ddot(t) "+file_name[16:-4], linewidth=3.0)

    axcom1a.legend()
    axcom1b.legend()
    axcom2.legend()
    axcom3a.legend()
    axcom3b.legend()
    axcom4a.legend()
    axcom4b.legend()
    axcom5a.legend()
    axcom5b.legend()
    axcom6a.legend()
    axcom6b.legend()
    axcom7a.legend()
    axcom7b.legend()
    axcom8a.legend()
    axcom8b.legend()
    axcom9.legend()


    return axf,acw, axfn, axcom1a,axcom1b,axcom2,axcom3a,axcom3b,axcom4a,axcom4b,axcom5a,axcom5b,axcom6a,axcom6b,axcom7a,axcom7b,axcom8a,axcom8b, axcom9