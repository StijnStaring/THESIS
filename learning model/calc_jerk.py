def calc_jerk(data_cl,fig):
    """
    Calculation of the jerk with central difference scheme
    """
    import pylab as plt
    lengte = len(data_cl['time_cl'])
    dt = data_cl['time_cl'][1, 0] - data_cl['time_cl'][0, 0]

    ax_d = data_cl['ax_local_cl']
    ay_d = data_cl['ay_local_cl']

    # calculate lateral jerk (using the central scheme) // Kan beter voor punten op rand --> forward difference
    jy_list = []
    for i in plt.arange(0, lengte, 1):
        if i == 0:
            jy_list.append((ay_d[i + 1]-ay_d[i])/dt)
        elif i == len(ay_d)-1:
            jy_list.append((ay_d[i]-ay_d[i-1])/dt)
        else:
            jy_list.append((ay_d[i + 1] - ay_d[i - 1]) / (2 * dt))

    jerk_dy = plt.array(jy_list)

    # calculate longitudinal jerk
    jx_list = []
    for i in plt.arange(0, lengte, 1):
        if i == 0:
            jx_list.append((ax_d[i + 1] - ax_d[i]) / dt)
        elif i == len(ax_d) - 1:
            jx_list.append((ax_d[i] - ax_d[i - 1]) / dt)
        else:
            jx_list.append((ax_d[i + 1] - ax_d[i - 1]) / (2 * dt))

    jerk_dx = plt.array(jx_list)

    if fig:
        plt.figure()
        plt.plot(data_cl['time_cl'], jerk_dy, 'r-', linewidth=2.0, label='lateral jerk DATA')
        plt.xlabel('t [s]')
        plt.ylabel('Jerk [m/s^3]')
        plt.title('Lateral jerk during lane change')
        plt.legend()
        plt.grid(True)

        plt.figure()
        plt.plot(data_cl['time_cl'], jerk_dx, 'r-', linewidth=2.0, label='longitudinal jerk DATA')
        plt.xlabel('t [s]')
        plt.ylabel('Jerk [m/s^3]')
        plt.title('Longitudinal jerk during lane change')
        plt.legend()
        plt.grid(True)



    return jerk_dy, jerk_dx





