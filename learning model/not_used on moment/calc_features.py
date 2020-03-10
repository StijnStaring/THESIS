import pylab as plt
import scipy
from scipy import integrate

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
        list = []
        for i in plt.arange(0,len(ax),1):
            list.append(i*dt_grid)
        time_vector = plt.array(list)
        # des_matrix = [delta_lane, desired_speed, time_lane_change]
        delta_lane = des_matrix[k,0]
        desired_speed = des_matrix[k,1]


        # Calculate integration features --> Simpson integration

        # f1: total acceleration
        integrand = plt.squeeze(ax** 2 + ay** 2)
        f1_cal = scipy.integrate.simps(integrand,time_vector)
        f1 = f1+ f1_cal
        # print('f1: ',f1)

        # f2: lateral acceleration
        integrand = plt.squeeze(ay** 2)
        f2_cal = scipy.integrate.simps(integrand, time_vector)
        f2 = f2 + f2_cal
        # print('f2: ', f2)

        # f3: total jerk
        integrand = plt.squeeze(jx ** 2 + jy ** 2)
        f3_cal = scipy.integrate.simps(integrand, time_vector)
        f3 = f3 + f3_cal
        # print('f3: ', f3)

        # f4: lateral jerk
        integrand = plt.squeeze(jy ** 2)
        f4_cal = scipy.integrate.simps(integrand, time_vector)
        f4 = f4 + f4_cal
        # print('f4: ', f4)

        # f5: curvature
        integrand = plt.squeeze((vx * ay - vy* ax) ** 2 / (vx ** 2 + vy ** 2) ** 3)
        f5_cal = scipy.integrate.simps(integrand, time_vector)
        f5 = f5 + f5_cal
        # print('f5: ', f5)

        # f6: desired speed
        integrand = plt.squeeze((desired_speed - vx) ** 2)
        f6_cal = scipy.integrate.simps(integrand, time_vector)
        f6 = f6 + f6_cal
        # print('f6: ', f6)

        # f7: desired lane change
        integrand = plt.squeeze((delta_lane - y) ** 2)
        f7_cal = scipy.integrate.simps(integrand, time_vector)
        f7 = f7 + f7_cal
        # print('f7: ', f7)

    # Devide featuers by m
    f1 = f1/length
    f2 = f2/length
    f3 = f3/length
    f4 = f4/length
    f5 = f5/length
    f6 = f6/length
    f7 = f7/length


    return f1,f2,f3,f4,f5,f6,f7