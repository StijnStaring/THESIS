def calc_jerk(data_cl,fig):
    """
    Calculation of the jerk with central difference scheme
    """
    import pylab as plt
    lengte = len(data_cl['time_cl'])
    dt = data_cl['time_cl'][1, 0] - data_cl['time_cl'][0, 0]
    # CHANGED TO LOCAL JERKS!!
    ax_d = data_cl['ax_cl']
    ay_d = data_cl['ay_cl']

    jerk_dy = plt.zeros((lengte, 1))
    jerk_dx = plt.zeros((lengte, 1))

    # calculate lateral jerk
    for i in plt.arange(1,lengte-1,1):
        jerk_dy[i] = (ay_d[i + 1] - ay_d[i - 1]) / (2 * dt)

    jerk_dy[0] = jerk_dy[1]
    jerk_dy[-1] = jerk_dy[-2]

    # calculate longitudinal jerk
    for i in plt.arange(1,lengte-1,1):
        jerk_dx[i] = (ax_d[i + 1] - ax_d[i - 1]) / (2 * dt)

    jerk_dx[0] = jerk_dx[1]
    jerk_dx[-1] = jerk_dx[-2]

    if fig:
        plt.figure()
        plt.plot(data_cl['time_cl'], jerk_dy, 'r-', linewidth=2.0, label='lateral jerk DATA')
        plt.xlabel('t [s]')
        plt.ylabel('Jerk [m/s^3]')
        plt.title('Lateral jerk during lane change (local)')
        plt.legend()
        plt.grid(True)

        plt.figure()
        plt.plot(data_cl['time_cl'], jerk_dx, 'r-', linewidth=2.0, label='longitudinal jerk DATA')
        plt.xlabel('t [s]')
        plt.ylabel('Jerk [m/s^3]')
        plt.title('Longitudinal jerk during lane change (local)')
        plt.legend()
        plt.grid(True)



    return jerk_dx, jerk_dy





