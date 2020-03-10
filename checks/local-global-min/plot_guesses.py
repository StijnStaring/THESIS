import pylab as plt
from define_plots_PG import define_plots_PG
from import_data_PG import import_data_PG
from import_ideal_data_PG import import_ideal_data_PG

[data_cl_list,files] = import_ideal_data_PG()
[ax1a,ax1b,ax2,ax3a,ax3b,ax4a,ax4b,ax5a,ax5b,ax6a,ax6b,ax7a,ax7b,ax8a,ax8b] = define_plots_PG()

for i in range(3):
    if files[i] == 'used_guesses\\3.csv':
        data_cl = data_cl_list[i]
        T_sol = data_cl['time_cl'][-1]
        x_sol = data_cl['x_cl']
        vx_sol = data_cl['vx_cl']
        ax_sol = data_cl['ax_cl']
        jx_sol = data_cl['jx_cl']
        y_sol = data_cl['y_cl']
        vy_sol = data_cl['vy_cl']
        ay_sol = data_cl['ay_cl']
        jy_sol = data_cl['jy_cl']
        psi_sol = data_cl['psi_cl']
        psi_dot_sol = data_cl['psi_dot_cl']
        throttle_sol = data_cl['throttle_cl']
        delta_sol = data_cl['delta_cl']
        aty_sol = data_cl['aty_cl']
        any_sol = data_cl['any_cl']

        time_vector = plt.linspace(0,T_sol,len(x_sol))

        # states
        ax1a.plot(time_vector, x_sol, '.-', linewidth=3.0,label= files[i])
        ax1b.plot(time_vector, y_sol, '.-', linewidth=3.0,label= files[i])
        ax2.plot(x_sol, y_sol, '.-', linewidth=3.0,label= files[i])
        ax3a.plot(time_vector, vx_sol, '.-', linewidth=3.0,label= files[i])
        ax3b.plot(time_vector, vy_sol, '.-', linewidth=3.0,label= files[i])
        ax4a.plot(time_vector, ax_sol, '.-', linewidth=3.0,label= files[i])
        ax4b.plot(time_vector, ay_sol, '.-', linewidth=3.0,label= files[i])
        ax5a.plot(time_vector, jx_sol, '.-', linewidth=3.0,label= files[i])
        ax5b.plot(time_vector, jy_sol, '.-', linewidth=3.0,label= files[i])
        ax6a.plot(time_vector, psi_sol * 180 / plt.pi, '.-', linewidth=3.0,label= files[i])
        ax6b.plot(time_vector, psi_dot_sol * 180 / plt.pi, '.-', linewidth=3.0,label= files[i])
        ax7a.plot(time_vector[0:-1], throttle_sol[0:-1], '.-', linewidth=3.0,label= files[i])
        ax7b.plot(time_vector[0:-1], delta_sol[0:-1]* 180 / plt.pi, '.-', linewidth=3.0,label= files[i])
        ax8a.plot(time_vector, aty_sol, '.-', linewidth=3.0,label= files[i])
        ax8b.plot(time_vector, any_sol, '.-', linewidth=3.0,label= files[i])

        print("\n")
        print("Lane change duration of: ",T_sol," [s]")

        ax1a.legend()
        ax1b.legend()
        ax2.legend()
        ax3a.legend()
        ax3b.legend()
        ax4a.legend()
        ax4b.legend()
        ax5a.legend()
        ax5b.legend()
        ax6a.legend()
        ax6b.legend()
        ax7a.legend()
        ax7b.legend()
        ax8a.legend()
        ax8b.legend()

    else:
        data_cl = data_cl_list[i]
        T_sol = data_cl['time_cl'][-1]
        x_sol = data_cl['x_cl']
        vx_sol = data_cl['vx_cl']
        ax_sol = data_cl['ax_cl']
        jx_sol = data_cl['jx_cl']
        y_sol = data_cl['y_cl']
        vy_sol = data_cl['vy_cl']
        ay_sol = data_cl['ay_cl']
        jy_sol = data_cl['jy_cl']
        psi_sol = data_cl['yaw_cl']
        psi_dot_sol = data_cl['r_cl']
        throttle_sol = data_cl['throttle_cl']
        delta_sol = data_cl['steering_rad_cl']
        aty_sol = data_cl['ay_local_cl']
        any_sol = data_cl['any_cl']

        time_vector = plt.linspace(0, T_sol, len(x_sol))

        # states
        ax1a.plot(time_vector, x_sol, '.-', linewidth=3.0,label= files[i])
        ax1b.plot(time_vector, y_sol, '.-', linewidth=3.0,label= files[i])
        ax2.plot(x_sol, y_sol, '.-', linewidth=3.0,label= files[i])
        ax3a.plot(time_vector, vx_sol, '.-', linewidth=3.0,label= files[i])
        ax3b.plot(time_vector, vy_sol, '.-', linewidth=3.0,label= files[i])
        ax4a.plot(time_vector, ax_sol, '.-', linewidth=3.0,label= files[i])
        ax4b.plot(time_vector, ay_sol, '.-', linewidth=3.0,label= files[i])
        ax5a.plot(time_vector, jx_sol, '.-', linewidth=3.0,label= files[i])
        ax5b.plot(time_vector, jy_sol, '.-', linewidth=3.0,label= files[i])
        ax6a.plot(time_vector, psi_sol * 180 / plt.pi, '.-', linewidth=3.0,label= files[i])
        ax6b.plot(time_vector, psi_dot_sol * 180 / plt.pi, '.-', linewidth=3.0,label= files[i])
        ax7a.plot(time_vector[0:-1], throttle_sol[0:-1], '.-', linewidth=3.0,label= files[i])
        ax7b.plot(time_vector[0:-1], delta_sol[0:-1]* 180 / plt.pi, '.-', linewidth=3.0,label= files[i])
        ax8a.plot(time_vector, aty_sol, '.-', linewidth=3.0,label= files[i])
        ax8b.plot(time_vector, any_sol, '.-', linewidth=3.0,label= files[i])

        ax1a.legend()
        ax1b.legend()
        ax2.legend()
        ax3a.legend()
        ax3b.legend()
        ax4a.legend()
        ax4b.legend()
        ax5a.legend()
        ax5b.legend()
        ax6a.legend()
        ax6b.legend()
        ax7a.legend()
        ax7b.legend()
        ax8a.legend()
        ax8b.legend()

        print("\n")
        print("Lane change duration of: ", T_sol, " [s]")

plt.show()
