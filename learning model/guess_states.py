def guess_states(data_cl,CP):
    import pylab as plt

    # sampling of observations
    index = plt.squeeze(plt.around(plt.linspace(0, len(data_cl['time_cl'])-1, CP+1), 0)).astype(int)
    x_guess = plt.squeeze(data_cl['x_cl'][index])
    y_guess = plt.squeeze(data_cl['y_cl'][index])
    vx_guess = plt.squeeze(data_cl['vx_cl'][index])
    vy_guess = plt.squeeze(data_cl['vy_cl'][index])
    psi_guess = plt.squeeze(data_cl['yaw_cl'][index])
    dpsi_guess = plt.squeeze(data_cl['r_cl'][index])
    throttle_guess = plt.squeeze(data_cl['throttle_cl'][index])
    # Fout!! deel door pi/180
    delta_guess = plt.squeeze(plt.pi/180*data_cl['steering_deg_cl'][index])

    return x_guess, y_guess, vx_guess, vy_guess, psi_guess, dpsi_guess ,throttle_guess, delta_guess