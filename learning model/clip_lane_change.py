def clip_lane_change(data):
    """
    % Finite difference scheme used: central difference scheme Differentiation
    Remarks:
    1. throw away first 0.15 s --> bad data
    """
    import pylab as plt
    from calc_jerk import calc_jerk

    # initializing
    time_d = plt.array([data.time]).T
    x = plt.array([data.X]).T
    y = plt.array([data.Y]).T
    yaw_d = plt.array([data.yaw]).T
    vx_d = plt.array([data.Vx]).T
    vy_d = plt.array([data.Vy]).T
    dt = time_d[1, 0]- time_d[0, 0]
    throw_away = 15
    vy_th = 0.0001 # sensitivity
    sleep = 10 # to notice a trend in data and avoid decision on single outlier
    index_start = 0
    index_end = 0

    # projection on global axis
    vy_proj = plt.sin(yaw_d)* vx_d + plt.cos(yaw_d) * vy_d

    # clipping data
    vy_diff = vy_proj[1:] - vy_proj[0:-1]

    for i in plt.arange(throw_away-1,len(time_d)-sleep,1):
        if all(abs(vy_diff[i:i+sleep])>=vy_th):
            index_start = i
            break

    for i in plt.arange(index_start-1,len(time_d)-sleep,1):
        if all(abs(vy_diff[i:i+sleep])<=vy_th):
            index_end = i
            break

    # End of the lane change is calculated based on the offset in the generation of the dataset.
    # Ts = 0.01
    # offset = round((float(file[15:-15]) - 140) / ((80 / 3.6) * Ts))
    # buffer = 20
    # index_end = int(140 / (80 / 3.6 * 0.01)) + int(offset) + buffer

    # Assign clipped data to a dictionary
    data_cl = dict()
    data_cl['time_cl'] = plt.array([data.time]).T[index_start:index_end+1] - plt.array([data.time]).T[index_start:index_end+1][0]
    data_cl['x_cl'] = plt.array([data.X]).T[index_start:index_end+1] - plt.array([data.X]).T[index_start:index_end+1][0]
    data_cl['y_cl'] = plt.array([data.Y]).T[index_start:index_end+1]
    data_cl['vx_cl'] = plt.array([data.Vx]).T[index_start:index_end+1] # small difference with projected speed
    data_cl['vy_cl'] = plt.array([data.Vy]).T[index_start:index_end+1] # big difference with projected speed
    data_cl['vy_proj_cl'] = vy_proj[index_start:index_end+1]
    data_cl['r_cl'] = plt.array([data.r]).T[index_start:index_end+1]
    data_cl['yaw_cl'] = plt.array([data.yaw]).T[index_start:index_end+1]
    data_cl['steering_deg_cl'] = plt.array([data.steering_deg]).T[index_start:index_end+1]
    data_cl['throttle_cl'] = plt.array([data.throttle]).T[index_start:index_end+1]
    data_cl['brake_cl'] = plt.array([data.brake]).T[index_start:index_end+1]
    data_cl['ax_cl'] = plt.array([data.ax]).T[index_start:index_end+1] # big difference with projected acceleration / total local acc
    data_cl['ay_cl'] = plt.array([data.ay]).T[index_start:index_end+1] # very small difference with projected acceleration / total local acc

    # output
    time_lane_change = (index_end - index_start) * dt
    start_lane_change = time_d[index_start]
    end_lane_change = time_d[index_end]
    delta_lane = data_cl['y_cl'][-1]

    dt_grid = dt

    # Amesim model produceert signals in de tangent vehicle axis (Dit is niet correct!!)
    # ax_loc_tot = data_cl['ax_cl'] - data_cl['r_cl']*data_cl['vy_cl']
    # ay_loc_tot = data_cl['ay_cl'] + data_cl['r_cl'] * data_cl['vx_cl']

    # The Amesim model is producing the total acceleration: tangent acceleration + normal acceleration!!
    ax_loc_tot = data_cl['ax_cl']
    ay_loc_tot = data_cl['ay_cl']

    # State of vehicle at the start of the lane change
    data_cl['vx_proj_cl'] = plt.cos(data_cl['yaw_cl'])* data_cl['vx_cl'] - plt.sin(data_cl['yaw_cl'])* data_cl['vy_cl']
    data_cl['ax_proj_cl'] = plt.cos(data_cl['yaw_cl'])* ax_loc_tot - plt.sin(data_cl['yaw_cl'])* ay_loc_tot
    data_cl['ay_proj_cl'] = plt.sin(data_cl['yaw_cl'])* ax_loc_tot  + plt.cos(data_cl['yaw_cl'])* ay_loc_tot

    # Calculating the real local accelerations
    data_cl['ax_local_cl'] = ax_loc_tot + data_cl['r_cl']*data_cl['vy_cl']
    data_cl['ay_local_cl'] = ay_loc_tot - data_cl['r_cl'] * data_cl['vx_cl']

    # calculating jerks
    jerk_y, jerk_x = calc_jerk(data_cl, 0)  # jerk is calculated in local axis.
    data_cl['jx_local_cl'] = jerk_x # This is the local jerk
    data_cl['jy_local_cl'] = jerk_y # This is the local jerk

    # setting desired speed and intitial values
    desired_speed = data_cl['vx_cl'][-1]


    return time_lane_change, start_lane_change, end_lane_change, index_start, index_end, delta_lane, desired_speed, dt_grid,data_cl
