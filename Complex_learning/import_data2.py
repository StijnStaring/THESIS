def import_data2(file,ak):
    import pylab as plt
    import pandas as pd
    from derivative import derivative
    data = pd.read_csv(file)
    data_cl = dict()

    # "time","x","y","vx","vy","ax","ay","jx","jy","psi","psi_dot","throttle","delta","aty","any","atx","anx"
    data_cl['time_cl'] = plt.array([data.time])
    N = data_cl['time_cl'].shape[1] - 1
    data_cl['x_cl'] = plt.array([data.x])
    data_cl['y_cl'] = plt.array([data.y])
    data_cl['vx_cl'] = plt.array([data.vx])
    data_cl['vy_cl'] = plt.array([data.vy])
    data_cl['ax_cl'] = plt.array([data.ax])  # total acc
    data_cl['ay_cl'] = plt.array([data.ay])  # total acc
    data_cl['jx_cl'] = plt.array([data.jx])  # total jerk
    data_cl['jy_cl'] = plt.array([data.jy])  # total jerk
    data_cl['psi_cl'] = plt.array([data.psi])  # in rad
    data_cl['psi_dot_cl'] = plt.array([data.psi_dot])
    data_cl['psi_ddot_cl'] = plt.array([data.psi_ddot])
    data_cl['throttle_cl'] = plt.array([data.throttle])
    data_cl['delta_cl'] = plt.array([data.delta])
    data_cl['atx_cl'] = plt.array([data.atx])
    data_cl['anx_cl'] = plt.array([data.anx])
    data_cl['aty_cl'] = plt.array([data.aty])
    data_cl['any_cl'] = plt.array([data.a_ny])
    data_cl['dt_cl'] = data_cl['time_cl'][0, 1] - data_cl['time_cl'][0, 0]
    data_cl['width'] = data_cl['y_cl'][0, -1]
    data_cl['vx_start'] = data_cl['vx_cl'][0, 0]
    data_cl['throttle_dot_cl'] = plt.array([data.throttle_dot])[None, 0, 0: N]
    data_cl['delta_dot_cl'] = plt.array([data.delta_dot])[None, 0, 0: N]

    # Asssigning data generartion degree of freedom

    # Calculation of features
    # f0: longitudinal acceleration
    integrand = data_cl['ax_cl'] ** 2
    f0_cal = 0
    for i in plt.arange(0, integrand.shape[1] - 1, 1):
        f0_cal = f0_cal + 0.5 * (integrand[0,i] + integrand[0,i + 1]) * data_cl['dt_cl']

    # f1: lateral acceleration
    integrand = data_cl['ay_cl'] ** 2
    f1_cal = 0
    for i in plt.arange(0, integrand.shape[1] - 1, 1):
        f1_cal = f1_cal + 0.5 * (integrand[0,i] + integrand[0,i + 1]) * data_cl['dt_cl']

    # f2: longitudinal jerk
    integrand = data_cl['jx_cl'] ** 2
    f2_cal = 0
    for i in plt.arange(0, integrand.shape[1] - 1, 1):
        f2_cal = f2_cal + 0.5 * (integrand[0,i] + integrand[0,i + 1]) * data_cl['dt_cl']

    # f3: lateral jerk
    integrand = data_cl['jy_cl'] ** 2
    f3_cal = 0
    for i in plt.arange(0, integrand.shape[1] - 1, 1):
        f3_cal = f3_cal + 0.5 * (integrand[0,i] + integrand[0,i + 1]) * data_cl['dt_cl']

    # f4: desired velocity
    integrand = (data_cl['vx_start'] - data_cl['vx_cl']) ** 2
    f4_cal = 0
    for i in plt.arange(0, integrand.shape[1] - 1, 1):
        f4_cal = f4_cal + 0.5 * (integrand[0,i] + integrand[0,i + 1]) * data_cl['dt_cl']

    # f5: desired lane change
    integrand = (data_cl['width'] - data_cl['y_cl']) ** 2
    f5_cal = 0
    for i in plt.arange(0, integrand.shape[1] - 1, 1):
        f5_cal = f5_cal + 0.5 * (integrand[0,i] + integrand[0,i + 1]) * data_cl['dt_cl']

    if ak == 1:
        print("\n")
        print('Integrated feature values of the DATA ')
        print('------------------------------')
        print('integrand = plt.squeeze(data_cl[ax_cl]**2)')
        print(f0_cal)
        print('integrand = plt.squeeze(data_cl[ay_cl] ** 2)')
        print(f1_cal)
        print('integrand = plt.squeeze(data_cl[jx_cl] ** 2)')
        print(f2_cal)
        print('integrand = plt.squeeze(data_cl[jy_cl] ** 2)')
        print(f3_cal)
        print('integrand = plt.squeeze((desired_speed - data_cl[vx_cl]) ** 2)')
        print(f4_cal)
        print('integrand = plt.squeeze((delta_lane - data_cl[y_cl]) ** 2)')
        print(f5_cal)

    features = plt.squeeze(plt.array([f0_cal,f1_cal,f2_cal,f3_cal,f4_cal,f5_cal]))
    data_cl['features'] = features

    return data_cl