def jerk_vehicle_model(time,ax_d,ay_d,fig):
    """
    Calculation of the jerk with central difference scheme
    """
    import pylab as plt
    lengte = len(time)
    dt = time[1] - time[0]

    jerk_d = plt.zeros((lengte, 1))
    jerk_dx = plt.zeros((lengte, 1))

    # calculate lateral jerk (using the central scheme)
    for i in plt.arange(1,lengte-1,1):
        jerk_d[i] = (ay_d[i + 1] - ay_d[i - 1]) / (2 * dt)

    jerk_d[0] = jerk_d[1]
    jerk_d[-1] = jerk_d[-2]

    # calculate longitudinal jerk
    for i in plt.arange(1,lengte-1,1):
        jerk_dx[i] = (ax_d[i + 1] - ax_d[i - 1]) / (2 * dt)

    jerk_dx[0] = jerk_dx[1]
    jerk_dx[-1] = jerk_dx[-2]

    if fig:
        plt.figure()
        plt.plot(time, jerk_d, 'r-', linewidth=2.0, label='lateral jerk DATA')
        plt.xlabel('t [s]')
        plt.ylabel('Jerk [m/s^3]')
        plt.title('Lateral jerk during lane change')
        plt.legend()
        plt.grid(True)

        plt.figure()
        plt.plot(time, jerk_dx, 'r-', linewidth=2.0, label='longitudinal jerk DATA')
        plt.xlabel('t [s]')
        plt.ylabel('Jerk [m/s^3]')
        plt.title('Longitudinal jerk during lane change')
        plt.legend()
        plt.grid(True)


    jerk_dy = jerk_d
    return jerk_dy, jerk_dx





