def import_ideal_data_PG():
    import glob
    import pylab as plt
    import pandas as pd

    files = glob.glob("used_guesses/*.csv")
    print("The name of the file: ", files)
    data_cl_list = []

    for i in range(len(files)):

        data = pd.read_csv(files[i])

        data_cl = dict()

        if files[i] == 'used_guesses\\3.csv':

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
            data_cl['throttle_cl'] = plt.array([data.throttle]).T
            data_cl['delta_cl'] = plt.array([data.delta]).T
            data_cl['aty_cl'] = plt.array([data.aty]).T
            data_cl['any_cl'] = plt.array([data.any]).T
            data_cl['dt_cl'] = data_cl['time_cl'][1, 0] - data_cl['time_cl'][0, 0]

            data_cl_list.append(data_cl)

        else:


    return data_cl_list