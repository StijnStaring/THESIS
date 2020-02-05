import pandas as pd
import numpy as np

data=pd.read_csv('Dataset_Xchange130.3244_V0=95.4982.csv')

print((np.array([data.time]).shape))

print(data)





# # This function is for defining the data parameters in a structure 'data'.
# % % Read
# data
# read = csvread('Dataset_Xchange130.3244_V0=95.4982.csv', 1);
# % % Struct
# data
# data = struct;
# data.time_d = read(:, 1);
# data.x_d = read(:, 2);
# data.y_d = read(:, 3);
# data.vx_d = read(:, 4);
# data.vy_d = read(:, 5);
# data.r_d = read(:, 6);
# data.yaw_d = read(:, 7);
# data.steering_deg_d = read(:, 8);
# data.throttle_d = read(:, 9);
# data.brake_d = read(:, 10);
# data.ax_d = read(:, 11);
# data.ay_d = read(:, 12);
#
# end