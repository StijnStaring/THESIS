import pylab as plt

def calc_features(his_x, his_vx, his_ax, his_jx, his_y, his_vy, his_ay, his_jy,his_time_cal_lc,des_matrix):
    length = his_x.shape[0]
    # intitialize the features
    f1 = 0
    f2 = 0
    f3 = 0
    f4 = 0
    f5 = 0
    f6 = 0
    f7 = 0

    for k in plt.arange(0,length,1):
        # x = his_x[k,:,plt.newaxis]
        vx = his_vx[k, :,plt.newaxis]
        ax = his_ax[k, :,plt.newaxis]
        jx = his_jx[k, :,plt.newaxis]

        y = his_y[k,:,plt.newaxis]
        vy = his_vy[k, :,plt.newaxis]
        ay = his_ay[k, :,plt.newaxis]
        jy = his_jy[k, :,plt.newaxis]

        dt_grid = his_time_cal_lc[k,1]
        delta_lane = des_matrix[k,0]
        desired_speed = des_matrix[k,1]


        # Calculate integration features --> Cranck-Nicolson integration

        # f1: total acceleration
        integrand = ax** 2 + ay** 2
        for i in plt.arange(0, len(integrand) - 1, 1):
            f1 = f1 + 0.5 * (integrand[i] + integrand[i + 1]) * dt_grid

        # f2: lateral acceleration
        integrand = ay** 2
        for i in plt.arange(0, len(integrand) - 1, 1):
            f2 = f2 + 0.5 * (integrand[i] + integrand[i + 1]) * dt_grid

        # f3: total jerk
        integrand = jx ** 2 + jy ** 2
        for i in plt.arange(0, len(integrand) - 1, 1):
            f3 = f3 + 0.5 * (integrand[i] + integrand[i + 1]) * dt_grid

        # f4: lateral jerk
        integrand = jy ** 2
        for i in plt.arange(0, len(integrand) - 1, 1):
            f4 = f4 + 0.5 * (integrand[i] + integrand[i + 1]) * dt_grid

        # f5: curvature
        integrand = (vx * ay - vy* ax) ** 2 / (vx ** 2 + vy ** 2) ** 3
        for i in plt.arange(0, len(integrand) - 1, 1):
            f5 = f5 + 0.5 * (integrand[i] + integrand[i + 1]) * dt_grid

        # f6: desired speed
        integrand = (desired_speed - vx) ** 2
        for i in plt.arange(0, len(integrand) - 1, 1):
            f6 = f6 + 0.5 * (integrand[i] + integrand[i + 1]) * dt_grid

        # f7: desired lane change
        integrand = (delta_lane - y) ** 2
        for i in plt.arange(0, len(integrand) - 1, 1):
            f7 = f7 + 0.5 * (integrand[i] + integrand[i + 1]) * dt_grid

    # Devide featuers by m
    f1 = f1/length
    f2 = f2/length
    f3 = f3/length
    f4 = f4/length
    f5 = f5/length
    f6 = f6/length
    f7 = f7/length


    return f1,f2,f3,f4,f5,f6,f7