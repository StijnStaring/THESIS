def import_ideal_data():
    import glob
    import pylab as plt
    import pandas as pd

    file = glob.glob("written_data/*.csv")
    print("The name of the file: ", file)

    data = pd.read_csv(file)
    data_cl = dict()

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

    return data_cl