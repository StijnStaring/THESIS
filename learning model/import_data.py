def import_data(plot):
    import glob
    from clip_lane_change import clip_lane_change
    from plotting_datasets import plotting_datasets
    import pylab as plt
    import pandas as pd

    files = glob.glob("used_data/*.csv")
    print("The amount of m fils are: ", len(files))
    length = len(files)

    # plotting of the datasets
    if plot == 1:
        [ax1a,ax1b,ax2,ax3a,ax3b,ax4a,ax4b,ax5a,ax5b,ax6,ax7a,ax7b,ax8a,ax8b] = plotting_datasets()

    # initialise the init state conditions:
    init_matrix = plt.zeros((length,8))
    des_matrix = plt.zeros((length,3))
    index = 0
    dict_list = []
    # intitialize the features
    f1 = 0
    f2 = 0
    f3 = 0
    f4 = 0
    f5 = 0
    f6 = 0
    f7 = 0

    for file in files:
        data = pd.read_csv(file)
        file = file[10:]
        print(file)

        # Clipping and plotting the observed lane change
        [time_lane_change, start_lane_change, end_lane_change, index_start, index_end, delta_lane, desired_speed, dt_grid, init, data_cl] =clip_lane_change(data)


        # Assign initial and desired conditions
        # init_matrix = [x,vx,y,vy,psi,dpsi,throttle,steerwheel_deg]
        init_matrix[index,:] = plt.squeeze(plt.array([init[0],init[1],init[2],init[3],init[4],init[5],init[6],init[7]]))
        des_matrix[index,:] = plt.squeeze(plt.array([delta_lane, desired_speed,time_lane_change]))

        # Calculate observed feature values --> Cranck-Nicolson integration

        # f1: total acceleration
        integrand = data_cl['ax_cl']**2+data_cl['ay_cl']**2
        for i in plt.arange(0,len(integrand)-1,1):
            f1 = f1 + 0.5*(integrand[i]+integrand[i+1])*dt_grid

        # f2: lateral acceleration
        integrand = data_cl['ay_cl']**2
        for i in plt.arange(0,len(integrand)-1,1):
            f2 = f2 + 0.5*(integrand[i]+integrand[i+1])*dt_grid

        # f3: yaw_rate
        integrand = data_cl['r_cl']**2
        for i in plt.arange(0,len(integrand)-1,1):
            f3 = f3 + 0.5*(integrand[i]+integrand[i+1])*dt_grid

        # f4: desired speed
        integrand = (desired_speed-data_cl['vx_cl'])**2
        for i in plt.arange(0, len(integrand) - 1, 1):
            f4 = f4 + 0.5 * (integrand[i] + integrand[i + 1]) * dt_grid

        # f5: desired lane change
        integrand = (delta_lane - data_cl['y_cl'])**2
        for i in plt.arange(0, len(integrand) - 1, 1):
            f5 = f5 + 0.5 * (integrand[i] + integrand[i + 1]) * dt_grid

        # Plotting in figures
        if plot == 1:

            ax1a.plot(data_cl['time_cl'], data_cl['x_cl'], '-',label = file,linewidth = 3.0)
            ax1b.plot(data_cl['time_cl'], data_cl['y_cl'], '-',label = file,linewidth = 3.0)

            ax2.plot(data_cl['x_cl'], data_cl['y_cl'], '-',label = file,linewidth = 3.0)

            # The vx and vy velocities are as seen in the local axis (on vehicle).
            ax3a.plot(data_cl['time_cl'], data_cl['vx_cl'], '-',label = file,linewidth = 3.0)
            ax3b.plot(data_cl['time_cl'], data_cl['vy_cl'], '-',label = file,linewidth = 3.0)

            # The ax and ay accelerations (a_t) are as seen in the local axis (on vehicle).
            ax4a.plot(data_cl['time_cl'], data_cl['ax_cl'], '-',label = file,linewidth = 3.0)
            ax4b.plot(data_cl['time_cl'], data_cl['ay_cl'], '-',label = file,linewidth = 3.0)

            # The jerk_x and jerk_y are as seen in the local axis (on vehicle).
            ax5a.plot(data_cl['time_cl'], data_cl['jx_cl'], '-',label = file,linewidth = 3.0)
            ax5b.plot(data_cl['time_cl'], data_cl['jy_cl'], '-',label = file,linewidth = 3.0)

            # The curvature of the path (global axis)
            ax6.plot(data_cl['time_cl'], data_cl['curvature_cl'], '-', label=file, linewidth=3.0)

            # The yaw and yaw_rate in degrees.
            ax7a.plot(data_cl['time_cl'], data_cl['yaw_cl']*180/plt.pi, '-', label=file, linewidth=3.0)
            ax7b.plot(data_cl['time_cl'], data_cl['r_cl']*180/plt.pi, '-', label=file, linewidth=3.0)

            # Inputs
            ax8a.plot(data_cl['time_cl'], data_cl['throttle_cl'], '-', label=file, linewidth=3.0)
            ax8b.plot(data_cl['time_cl'], data_cl['steering_deg_cl'], '-', label=file, linewidth=3.0)

            ax1a.legend()
            ax1b.legend()
            ax2.legend()
            ax3a.legend()
            ax3b.legend()
            ax4a.legend()
            ax4b.legend()
            ax5a.legend()
            ax5b.legend()
            ax6.legend()
            ax7a.legend()
            ax7b.legend()
            ax8a.legend()
            ax8b.legend()

        dict_list.append(data_cl)
        index = index + 1

    # Devide featuers by amount of datasets (averaging)
    f1 = f1/length
    f2 = f2/length
    f3 = f3/length
    f4 = f4/length
    f5 = f5/length


    return f1,f2,f3,f4,f5,init_matrix,des_matrix,dict_list,files
# init matrix: amount of datasets x 8
# des  matrix: amount of datasets x 3





