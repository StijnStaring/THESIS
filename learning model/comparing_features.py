def comparing_features(data_cl,file_name):
    import pylab as plt

    plt.rc('axes', linewidth=2)
    plt.rc('legend', fontsize=16)
    # plt.rcParams["legend.loc"] = 'best'
    fontsize = 14
    font = 16

    # Define plot to compare normalized features
    plt.figure("Comparing Normalized Features")
    plt.tight_layout()
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
    plt.tight_layout()
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
    plt.tight_layout()
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
    plt.tight_layout()
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
    plt.tight_layout()
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
    plt.tight_layout()
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
    plt.tight_layout()
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
    plt.tight_layout()
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
    plt.tight_layout()
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
    plt.tight_layout()
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

    # Different parts of the longitudinal lateral acceleration
    plt.figure("Axt", figsize=(10, 4))
    plt.tight_layout()
    plt.subplot(1, 2, 1)
    axcom8a = plt.gca()
    plt.xlabel("Time [s]", fontsize=font, fontweight='bold')
    plt.ylabel("Axt [m/s^2]", fontsize=font, fontweight='bold')
    plt.grid(True)
    # plt.title('Axt ', fontsize=font,fontweight='bold')

    for tick in axcom8a.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom8a.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(1, 2, 2)
    axcom8b = plt.gca()
    plt.xlabel("Time [s]", fontsize=font, fontweight='bold')
    plt.ylabel("Axn [m/s^2]", fontsize=font, fontweight='bold')
    plt.grid(True)
    # plt.title('Ayn', fontsize=font,fontweight='bold')

    for tick in axcom8b.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom8b.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    # Different parts of the local lateral acceleration
    plt.figure("Ayt", figsize=(10, 4))
    plt.tight_layout()
    plt.subplot(1, 2, 1)
    axcom9a = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("Axt [m/s^2]", fontsize=font,fontweight='bold')
    plt.grid(True)
    # plt.title('Ayt ', fontsize=font,fontweight='bold')

    for tick in axcom9a.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom9a.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(1, 2, 2)
    axcom9b = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("Ayn [m/s^2]", fontsize=font,fontweight='bold')
    plt.grid(True)
    # plt.title('Ayn', fontsize=font,fontweight='bold')

    for tick in axcom9b.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom9b.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    # yaw acceleration

    plt.figure("Yaw acceleration", figsize=(10, 4))
    plt.tight_layout()
    axcom10 = plt.gca()
    plt.xlabel("Time [s]", fontsize=font,fontweight='bold')
    plt.ylabel("Yaw acceleration [degrees/s^2]", fontsize=font,fontweight='bold')
    plt.grid(True)
    # plt.title('Yaw acceleration ', fontsize=font,fontweight='bold')

    for tick in axcom10.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom10.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    # Inputs_dot
    plt.figure("throttle_dot", figsize=(10, 4))
    plt.tight_layout()
    plt.subplot(1, 2, 1)
    axcom11a = plt.gca()
    plt.xlabel("Time [s]", fontsize=font, fontweight='bold')
    plt.ylabel("throttle_dot [1/s]", fontsize=font, fontweight='bold')
    plt.grid(True)
    # plt.title('throttle ', fontsize=font,fontweight='bold')

    for tick in axcom11a.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom11a.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')

    plt.subplot(1, 2, 2)
    axcom11b = plt.gca()
    plt.xlabel("Time [s]", fontsize=font, fontweight='bold')
    plt.ylabel("delta_dot [degrees/s]", fontsize=font, fontweight='bold')
    # plt.title('delta local', fontsize=font,fontweight='bold')
    plt.grid(True)

    for tick in axcom11b.xaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')
    for tick in axcom11b.yaxis.get_major_ticks():
        tick.label1.set_fontsize(fontsize)
        tick.label1.set_fontweight('bold')


    # Plotting data in figures
    N = len(plt.squeeze(data_cl['x_cl'])) - 1
    axcom1a.plot(plt.squeeze(data_cl['time_cl']), plt.squeeze(data_cl['x_cl']), '-', label="X(t) "+file_name[15:-4], linewidth=3.0)
    axcom1b.plot(plt.squeeze(data_cl['time_cl']), plt.squeeze(data_cl['y_cl']), '-', label="Y(t) "+file_name[15:-4], linewidth=3.0)

    axcom2.plot(plt.squeeze(data_cl['x_cl']), plt.squeeze(data_cl['y_cl']), '-', label="path "+file_name[15:-4], linewidth=3.0)

    # The vx and vy velocities are as seen in the global axis (fixed).
    axcom3a.plot(plt.squeeze(data_cl['time_cl']), plt.squeeze(data_cl['vx_cl']), '-', label="Vx(t) "+file_name[15:-4], linewidth=3.0)
    axcom3b.plot(plt.squeeze(data_cl['time_cl']), plt.squeeze(data_cl['vy_cl']), '-', label="Vy(t) "+file_name[15:-4], linewidth=3.0)

    # The ax and ay accelerations are as seen in the global axis (fixed).
    axcom4a.plot(plt.squeeze(data_cl['time_cl']), plt.squeeze(data_cl['ax_cl']), '-', label="Ax(t) "+file_name[15:-4], linewidth=3.0)
    axcom4b.plot(plt.squeeze(data_cl['time_cl']), plt.squeeze(data_cl['ay_cl']), '-', label="Ay(t) "+file_name[15:-4], linewidth=3.0)

    # The jerk_x and jerk_y are as seen in the global axis (fixed).
    axcom5a.plot(plt.squeeze(data_cl['time_cl']), plt.squeeze(data_cl['jx_cl']), '-', label="Jx(t) "+file_name[15:-4], linewidth=3.0)
    axcom5b.plot(plt.squeeze(data_cl['time_cl']), plt.squeeze(data_cl['jy_cl']), '-', label="Jy(t) "+file_name[15:-4], linewidth=3.0)

    # yaw and yaw rate
    axcom6a.plot(plt.squeeze(data_cl['time_cl']), plt.squeeze(data_cl['psi_cl'])* 180 / plt.pi, '-', label="Psi(t) "+file_name[15:-4], linewidth=3.0)
    axcom6b.plot(plt.squeeze(data_cl['time_cl']), plt.squeeze(data_cl['psi_dot_cl'])* 180 / plt.pi, '-', label="Psi_dot(t) "+file_name[15:-4], linewidth=3.0)

    # throttle and delta and yaw rate
    axcom7a.plot(plt.squeeze(data_cl['time_cl']), plt.squeeze(data_cl['throttle_cl']) , '-', label="Throttle(t) "+file_name[15:-4], linewidth=3.0)
    axcom7b.plot(plt.squeeze(data_cl['time_cl']), plt.squeeze(data_cl['delta_cl']) * 180 / plt.pi, '-', label="Delta(t) "+file_name[15:-4],linewidth=3.0)

    # axt and axn
    axcom8a.plot(plt.squeeze(data_cl['time_cl']), plt.squeeze(data_cl['atx_cl']), '-',label="Atx(t) " + file_name[15:-4], linewidth=3.0)
    axcom8b.plot(plt.squeeze(data_cl['time_cl']), plt.squeeze(data_cl['anx_cl']), '-',label="Anx(t) " + file_name[15:-4], linewidth=3.0)

    # ayt and ayn
    axcom9a.plot(plt.squeeze(data_cl['time_cl']), plt.squeeze(data_cl['aty_cl']), '-', label="Aty(t) "+file_name[15:-4], linewidth=3.0)
    axcom9b.plot(plt.squeeze(data_cl['time_cl']), plt.squeeze(data_cl['any_cl']), '-', label="Any(t) "+file_name[15:-4], linewidth=3.0)

    # yaw rate aceleration
    axcom10.plot(plt.squeeze(data_cl['time_cl']), plt.squeeze(data_cl['psi_ddot_cl'])*180/plt.pi, '-', label="Psi_ddot(t) "+file_name[15:-4], linewidth=3.0)

    # throttle_dot and delta_dot and yaw rate
    axcom11a.plot(plt.squeeze(data_cl['time_cl'])[0:N], plt.squeeze(data_cl['throttle_dot_cl']) , '-', label="Throttle_dot(t) "+file_name[15:-4], linewidth=3.0)
    axcom11b.plot(plt.squeeze(data_cl['time_cl'])[0:N], plt.squeeze(data_cl['delta_dot_cl']) * 180 / plt.pi, '-', label="Delta_dot(t) "+file_name[15:-4],linewidth=3.0)

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
    axcom9a.legend()
    axcom9b.legend()
    axcom10.legend()
    axcom11a.legend()
    axcom11b.legend()

    return axf,acw, axfn, axcom1a,axcom1b,axcom2,axcom3a,axcom3b,axcom4a,axcom4b,axcom5a,axcom5b,axcom6a,axcom6b,axcom7a,axcom7b,axcom8a,axcom8b, axcom9a, axcom9b, axcom10,axcom11a, axcom11b