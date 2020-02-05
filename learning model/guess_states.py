def guess_states(data_cl,v_desired,CP):
    import pylab as plt
    from calc_jerk import calc_jerk

    jerk_y, jerk_x = calc_jerk(data_cl,0)
    yaw = data_cl['yaw_cl']

    # projections to global axis
    # vx_proj = plt.cos(yaw)* data_cl['vx_cl'] - plt.sin(yaw)* data_cl['vy_cl']
    vy_proj = plt.sin(yaw)* data_cl['vx_cl'] + plt.cos(yaw)* data_cl['vy_cl']
    # ax_proj = plt.cos(yaw)* data_cl['ax_cl'] - plt.sin(yaw)* data_cl['ay_cl']
    ay_proj = plt.sin(yaw)* data_cl['ax_cl'] + plt.cos(yaw)* data_cl['ay_cl']

    # sampling of observations
    index = plt.arange(0,CP+1,1)
    x_guess = plt.squeeze(data_cl['x_cl'][index])
    y_guess = plt.squeeze(data_cl['y_cl'][index])
    vx_guess = plt.ones(len(index))*v_desired
    vy_guess = plt.squeeze(vy_proj[index])
    ax_guess = plt.zeros(len(index))
    jx_guess = plt.zeros(len(index))
    ay_guess = plt.squeeze(ay_proj[index])
    jy_guess = plt.squeeze(jerk_y[index])

    return x_guess, y_guess, vx_guess, vy_guess, ax_guess,ay_guess,jx_guess, jy_guess