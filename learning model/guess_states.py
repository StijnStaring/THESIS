def guess_states(data_cl,v_desired,CP):
    import pylab as plt

    # sampling of observations
    index = plt.squeeze(plt.around(plt.linspace(0, len(data_cl['time_cl'])-1, CP+1), 0)).astype(int)
    x_guess = plt.squeeze(data_cl['x_cl'][index])
    y_guess = plt.squeeze(data_cl['y_cl'][index])
    vx_guess = plt.ones(len(index)) * v_desired
    vy_guess = plt.squeeze(data_cl['vy_proj_cl'][index])
    ax_guess = plt.zeros(len(index))
    jx_guess = plt.zeros(len(index))
    ay_guess = plt.squeeze(data_cl['ay_proj_cl'][index])
    jy_guess = plt.squeeze(data_cl['jy_cl'][index])

    return x_guess, y_guess, vx_guess, vy_guess, ax_guess,ay_guess,jx_guess, jy_guess