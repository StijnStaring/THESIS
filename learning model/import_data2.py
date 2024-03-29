def import_data2(file,ak):
    import pylab as plt
    import pandas as pd


    # data = pd.read_csv(file[0])
    data = pd.read_csv(file)
    # theta_rel used in data = plt.array([4,5,6,1,2] with norms:
    # norm0 = 0.007276047781441449
    # norm1 = 2.6381715506137424
    # norm2 = 11.283498669013454
    # norm3 = 0.046662223759442054
    # norm4 = 17.13698903738383

    data_cl = dict()

    # path = "rading_data\ diff_weights_guess"
    # file = open(path, newline='')
    # reader = csv.reader(file)
    # header = next(reader)
    # data = []

    # "time","x","y","vx","vy","ax","ay","jx","jy","psi","psi_dot","throttle","delta","aty","any"
    data_cl['time_cl'] = plt.array([data.time]).T
    data_cl['x_cl'] = plt.array([data.x]).T
    data_cl['y_cl'] = plt.array([data.y]).T
    data_cl['vx_cl'] = plt.array([data.vx]).T
    data_cl['vy_cl'] = plt.array([data.vy]).T
    data_cl['ax_cl'] = plt.array([data.ax]).T #total acc
    data_cl['ay_cl'] = plt.array([data.ay]).T #total acc
    data_cl['jx_cl'] = plt.array([data.jx]).T
    data_cl['jy_cl'] = plt.array([data.jy]).T
    data_cl['psi_cl'] = plt.array([data.psi]).T
    data_cl['psi_dot_cl'] = plt.array([data.psi_dot]).T
    data_cl['psi_ddot_cl'] = plt.array([data.psi_ddot]).T
    data_cl['throttle_cl'] = plt.array([data.throttle]).T
    data_cl['delta_cl'] = plt.array([data.delta]).T
    data_cl['aty_cl'] = plt.array([data.aty]).T
    data_cl['any_cl'] = data_cl['psi_dot_cl'] * data_cl['vx_cl']
    data_cl['dt_cl'] = data_cl['time_cl'][1, 0] - data_cl['time_cl'][0, 0]

    # Asssigning data generartion degree of freedom
    width_road = data_cl['y_cl'][-1]
    vx_start = data_cl['vx_cl'][0]

    # Calculation of features
    # f0: longitudinal acceleration
    integrand = data_cl['ax_cl'] ** 2
    f0_cal = 0
    for i in plt.arange(0, len(integrand) - 1, 1):
        f0_cal = f0_cal + 0.5 * (integrand[i] + integrand[i + 1]) * data_cl['dt_cl']

    # f1: lateral acceleration
    integrand = data_cl['ay_cl'] ** 2
    f1_cal = 0
    for i in plt.arange(0, len(integrand) - 1, 1):
        f1_cal = f1_cal + 0.5 * (integrand[i] + integrand[i + 1]) * data_cl['dt_cl']
    # f1_cal = scipy.integrate.simps(integrand,plt.array(time_list))
    # print('f1: ',f1_cal)

    # f2: lateral jerk
    integrand = plt.array(data_cl['jy_cl']) ** 2
    f2_cal = 0
    for i in plt.arange(0, len(integrand) - 1, 1):
        f2_cal = f2_cal + 0.5 * (integrand[i] + integrand[i + 1]) * data_cl['dt_cl']
    # f2_cal = scipy.integrate.simps(integrand,plt.array(time_list))

    # f3: desired velocity
    integrand = plt.array(vx_start - data_cl['vx_cl']) ** 2
    f3_cal = 0
    for i in plt.arange(0, len(integrand) - 1, 1):
        f3_cal = f3_cal + 0.5 * (integrand[i] + integrand[i + 1]) * data_cl['dt_cl']
    # f4_cal = scipy.integrate.simps(integrand,plt.array(time_list))

    # f4: desired lane change
    integrand = plt.array(width_road - data_cl['y_cl']) ** 2
    f4_cal = 0
    for i in plt.arange(0, len(integrand) - 1, 1):
        f4_cal = f4_cal + 0.5 * (integrand[i] + integrand[i + 1]) * data_cl['dt_cl']

    if ak == 1:
        print("\n")
        print('Integrated feature values of the DATA ')
        print('------------------------------')
        print('integrand = plt.squeeze(data_cl[ax_cl]**2)')
        print(f0_cal)
        print('integrand = plt.squeeze(data_cl[ay_cl] ** 2)')
        print(f1_cal)
        print('integrand = plt.squeeze(data_cl[jy_cl] ** 2)')
        print(f2_cal)
        print('integrand = plt.squeeze((desired_speed - data_cl[vx_cl]) ** 2)')
        print(f3_cal)
        print('integrand = plt.squeeze((delta_lane - data_cl[y_cl]) ** 2)')
        print(f4_cal)

    features = plt.array([f0_cal,f1_cal,f2_cal,f3_cal,f4_cal])

    return data_cl,features,width_road,vx_start