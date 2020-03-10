import pylab as plt
from define_plots_PR import define_plots_PR
from import_ideal_data_PR import import_ideal_data_PR

[data_cl_list,files] = import_ideal_data_PR()
[ax1a,ax1b,ax2,ax3a,ax3b,ax4a,ax4b,ax5a,ax5b,ax6a,ax6b,ax7a,ax7b,ax8a,ax8b, ex1a,ex1b,ex3a,ex3b,ex6a,ex6b] = define_plots_PR()

error_x = 0
error_y = 0
error_vx = 0
error_vy = 0
error_psi = 0
error_psi_dot = 0
amount = len(files) - 1

T_so = 0
x_so = 0
vx_so = 0
ax_so = 0
jx_so = 0
y_so = 0
vy_so = 0
ay_so = 0
jy_so = 0
psi_so = 0
psi_dot_so = 0
throttle_so = 0
delta_so = 0
aty_so = 0
any_so = 0

for i in range(3):
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

    if i == 0:
        T_so = data_cl['time_cl'][-1]
        x_so = data_cl['x_cl']
        vx_so = data_cl['vx_cl']
        ax_so = data_cl['ax_cl']
        jx_so = data_cl['jx_cl']
        y_so = data_cl['y_cl']
        vy_so = data_cl['vy_cl']
        ay_so = data_cl['ay_cl']
        jy_so = data_cl['jy_cl']
        psi_so = data_cl['psi_cl']
        psi_dot_so = data_cl['psi_dot_cl']
        throttle_so = data_cl['throttle_cl']
        delta_so = data_cl['delta_cl']
        aty_so = data_cl['aty_cl']
        any_so = data_cl['any_cl']

    else:
        error_x = error_x + (x_so -x_sol)
        error_y = error_y +(y_so - y_sol)
        error_vx = error_vx + (vx_so - vx_sol)
        error_vy = error_vy + (vy_so - vy_sol)
        error_psi = error_psi + (psi_so - psi_sol)
        error_psi_dot = error_psi_dot + (psi_dot_so -psi_dot_sol)

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

    if i> 0:
        ex1a.plot(time_vector, error_x, '.-', linewidth=3.0, label=files[i])
        ex1b.plot(time_vector, error_y, '.-', linewidth=3.0, label=files[i])
        ex3a.plot(time_vector, error_vx, '.-', linewidth=3.0, label=files[i])
        ex3b.plot(time_vector, error_vy, '.-', linewidth=3.0, label=files[i])
        ex6a.plot(time_vector, error_psi * 180 / plt.pi, '.-', linewidth=3.0, label=files[i])
        ex6b.plot(time_vector, error_psi_dot * 180 / plt.pi, '.-', linewidth=3.0, label=files[i])

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

    ex1a.legend()
    ex1b.legend()
    ex3a.legend()
    ex3b.legend()
    ex6a.legend()
    ex6b.legend()

plt.show()
