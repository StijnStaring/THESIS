function data_mpc = track_reference(data_planned)
% remember to take different delta_angle into account.
disp('Tracking mpc started!')
import casadi.*
global x_sol_prev lam_prev tracking_lane_change iteration T_pl N data T_MPC iter_expected plot_MPC Ts Tf 
plot_MPC = 0;
N = 50; % Control horizon of one optimization of the MPC.

data_temp = data_planned;
data_temp.delta_cl = data_planned.delta_cl*16.96; % SWA
data_temp.delta_dot_cl = data_planned.delta_dot_cl*16.96; % SWA
data = get_data(data_temp);
update_casadi_function = 1; % in order to save time when developing
[x_sol_prev,lam_prev] = Function_generation(data,N,update_casadi_function);
iteration = 1;
throttle_start = 0.02295;

% Simulation sampling time and duration
% Ts = 0.01; % sampling rate Amesim - taken standard value
T_MPC = 0.1; % sampling rate of tracking algorithm
T_pl = data.time(2,1)-data.time(1,1);
N_ref = int64(Tf/T_pl+1);
N_ame = int64(Tf/Ts+1);
iter_expected = Tf/T_MPC + 1;
pi = 3.14159265359;
% Set the initial speed in the Amesim model
addpath(fullfile(getenv('AME'),'scripting','matlab','amesim'));
! AMECirChecker -g -q --nobackup --nologfile Dynamics.ame
! AMELoad Dynamics.ame
% Adjusted this state manually
ameputgpar('Dynamics', 'V0', 80/3.6) % this command only is not working --> have to set manual in amesim block in simulink
sim_opt = amegetsimopt('Dynamics');
amerunsingle('Dynamics', sim_opt);

%% Load the CasADi function for the MPC planner

tracking_lane_change = Function.load('tracking_lane_change.casadi');
DM.set_precision(15);

%% run simulation simulink
open('generating_data_lane_change');
sim_out = sim('generating_data_lane_change','SrcWorkspace','current'); % This is running the simulink file with function workspace

%% Figures and save data
% time_control = control_signals.time;
t_mpc = sim_out.Results_States.time';
t_max = 23;
x_max = 550;
x_mpc  = sim_out.Results_States.signals(1).values';  % Vector of 2551 points
y_mpc   = sim_out.Results_States.signals(2).values'; 
vx_mpc  = sim_out.Results_States.signals(3).values';
vy_mpc  = sim_out.Results_States.signals(4).values';
psi_mpc = sim_out.Results_States.signals(5).values'; 
psi_dot_mpc   = sim_out.Results_States.signals(6).values';  
throttle_mpc = squeeze(sim_out.Results_States.signals(7).values)';
delta_mpc = squeeze(sim_out.Results_States.signals(8).values)';


% time_acc = Accelerations_output.time;
ax_mpc       = sim_out.Accelerations_output.signals.values(:,1)';  % in the beginning --> undesired deaccelleration due to delay in controls.
ay_mpc       = sim_out.Accelerations_output.signals.values(:,2)';  

t_ref = data.time';
x_ref = data.x';
y_ref = data.y';
vx_ref = data.vx';
vy_ref = data.vy';
psi_ref = data.psi';
psi_dot_ref = data.psi_dot';

atx_ref = data.atx';
anx_ref = data.anx';
ax_ref = data.ax';

aty_ref = data.aty';
any_ref = data.any';
ay_ref = data.ay';

jx_ref = data.jx';
jy_ref = data.jy';

delta_ref = data.delta';
throttle_ref = data.throttle';

delta_dot_ref = data.delta_dot';
throttle_dot_ref = data.throttle_dot';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Path (x,y)
figure('name', 'Path')
plot(x_ref,y_ref,'r.','LineWidth',1.0)
hold on
plot(x_mpc,y_mpc,'b.','LineWidth',1.0)
title('Vehicle path','fontsize',12,'fontweight','bold')
xlabel('X [m]','fontsize',12)
xlim([0, x_max])
ylabel('Y [m]','fontsize',12)
% legend('R\_path','C\_path','Location','southeast')
% saveas(gcf,".\written_data\1path_N"+string(N)+"_TMPC C"+string(T_MPC)+"_Tf"+string(Tf)+".png")
% saveas(gcf,".\written_data\1path_N"+string(N)+"_TMPC C"+string(T_MPC)+"_Tf"+string(Tf)+".fig")
% Postion vs time
figure('name', 'Pos')
subplot(2,2,1)
plot(t_ref,x_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,x_mpc,'b.','LineWidth',1.0)
title('X [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position X [m]','fontsize',12)
% legend('R\_x path','C\_x path','Location','southeast')
subplot(2,2,3)
plot(t_mpc(1:5:N_ame),abs(x_ref(1:2:N_ref)-x_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position error X [m]','fontsize',12)

subplot(2,2,2)
plot(t_ref,y_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,y_mpc,'b.','LineWidth',1.0)
title('Y [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position Y [m]','fontsize',12)
% legend('R\_y path','C\_y path','Location','southeast')
subplot(2,2,4)
plot(t_ref(1:2:N_ref),abs(y_ref(1:2:N_ref)-y_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position error Y [m]','fontsize',12)
% saveas(gcf,".\written_data\2xy_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
% saveas(gcf,".\written_data\2xy_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")

% Velocity vs time
figure('name', 'Vel')
subplot(2,2,1)
plot(t_ref,vx_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,vx_mpc,'b.','LineWidth',1.0)
title('v_X [m/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('velocity X [m/s]','fontsize',12)
% legend('R\_X vel','C\_X vel','Location','southeast')
subplot(2,2,3)
plot(t_mpc(1:5:N_ame),abs(vx_ref(1:2:N_ref)-vx_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('velocity error X [m/s]','fontsize',12)
subplot(2,2,2)
plot(t_ref,vy_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,vy_mpc,'b.','LineWidth',1.0)
title('v_Y [m/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('velocity Y [m/s]','fontsize',12)
% legend('R\_Y vel','C\_Y vel','Location','southeast')
subplot(2,2,4)
plot(t_mpc(1:5:N_ame),abs(vy_ref(1:2:N_ref)-vy_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('velocity error Y [m/s]','fontsize',12)
% saveas(gcf,".\written_data\3vxy_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
% saveas(gcf,".\written_data\3vxy_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Need plots of ay_tot, aty, any, ax_tot, atx, any
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
atx_mpc = derivative(vx_mpc,Ts);
anx_mpc = -vy_mpc.*psi_dot_mpc;
aty_mpc = derivative(vy_mpc,Ts);
any_mpc = vx_mpc.*psi_dot_mpc;

figure('name', 'Acc\_y')
subplot(2,2,1)
plot(t_ref,aty_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,aty_mpc,'b.','LineWidth',1.0)
title('atY [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration atY [m/s²]','fontsize',12)
% legend('R\_aty acc','C\_aty acc','Location','southeast')
subplot(2,2,3)
plot(t_mpc(1:5:N_ame),abs(aty_ref(1:2:N_ref)-aty_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration error aty [m/s²]','fontsize',12)
subplot(2,2,2)
plot(t_ref,any_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,any_mpc,'b.','LineWidth',1.0)
title('anY [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration any [m/s²]','fontsize',12)
% legend('R\_any acc','C\_any acc','Location','southeast')
subplot(2,2,4)
plot(t_mpc(1:5:N_ame),abs(any_ref(1:2:N_ref)-any_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration error anY [m/s²]','fontsize',12)
% saveas(gcf,".\written_data\4ay_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
% saveas(gcf,".\written_data\4ay_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")
figure('name', 'Acc\_x')
subplot(2,2,1)
plot(t_ref,atx_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,atx_mpc,'b.','LineWidth',1.0)
title('atx [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration atx [m/s²]','fontsize',12)
% legend('R\_atx acc','C\_atx acc','Location','southeast')
subplot(2,2,3)
plot(t_mpc(1:5:N_ame),abs(atx_ref(1:2:N_ref)-atx_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration error atx [m/s²]','fontsize',12)
subplot(2,2,2)
plot(t_ref,anx_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,anx_mpc,'b.','LineWidth',1.0)
title('anx [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration anx [m/s²]','fontsize',12)
% legend('R\_anx acc','C\_anx acc','Location','southeast')
subplot(2,2,4)
plot(t_mpc(1:5:N_ame),abs(anx_ref(1:2:N_ref)-anx_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration error anx [m/s²]','fontsize',12)
% saveas(gcf,".\written_data\5ax_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
% saveas(gcf,".\written_data\5ax_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")
%%%%%%%%%%%%%%%%%%%%%%%%%
figure('name', 'Acc\_tot')
subplot(2,2,1)
plot(t_ref,ay_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,ay_mpc,'b.','LineWidth',1.0)
title('aytot [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration aytot [m/s²]','fontsize',12)
% legend('R\_aytot acc','C\_aytot acc','Location','southeast')
subplot(2,2,3)
plot(t_mpc(1:5:N_ame),abs(ay_ref(1:2:N_ref)-ay_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration error aytot [m/s²]','fontsize',12)
subplot(2,2,2)
plot(t_ref,ax_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,ax_mpc,'b.','LineWidth',1.0)
title('axtot [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration axtot [m/s²]','fontsize',12)
% legend('R\_axtot acc','C\_axtot acc','Location','southeast')
subplot(2,2,4)
plot(t_mpc(1:5:N_ame),abs(ax_mpc(1:5:N_ame)-ax_ref(1:2:N_ref)),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration error axtot [m/s²]','fontsize',12)
% saveas(gcf,".\written_data\6atot_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
% saveas(gcf,".\written_data\6atot_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")
%%%%%%%%%%%%%%%%%%%%%
% Yaw and Yaw rate 
figure('name', 'Yaw')
subplot(2,2,1)
plot(t_ref,psi_ref*180/pi,'r.','LineWidth',1.0)
hold on
plot(t_mpc,psi_mpc*180/pi,'b.','LineWidth',1.0)
title('\psi [deg]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('yaw angle \psi [deg]','fontsize',12)
% legend('R\_yaw','C\_yaw','Location','northeast')
subplot(2,2,3)
plot(t_mpc(1:5:N_ame),abs(psi_ref(1:2:N_ref)-psi_mpc(1:5:N_ame))*180/pi,'b','LineWidth',1.0)
title('Error [deg]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('yaw angle error \psi [deg]','fontsize',12)
subplot(2,2,2)
plot(t_ref,psi_dot_ref*180/pi,'r.','LineWidth',1.0)
hold on
plot(t_mpc,psi_dot_mpc*180/pi,'b.','LineWidth',1.0)
title('d\psi [deg/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('yaw velocity d\psi [deg/s]','fontsize',12)
% legend('R\_yaw vel','C\_yaw vel','Location','northeast')
subplot(2,2,4)
plot(t_mpc(1:5:N_ame),abs(psi_dot_ref(1:2:N_ref)-psi_dot_mpc(1:5:N_ame))*180/pi,'b','LineWidth',1.0)
title('Error [deg/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('yaw velocity error d\psi [deg/s]','fontsize',12)
% saveas(gcf,".\written_data\7psi_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
% saveas(gcf,".\written_data\7psi_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")

%  delta & throttle
figure('name', 'tr&delta')
subplot(2,2,1)
plot(t_ref,delta_ref*180/pi,'r.','LineWidth',1.0)
hold on
plot(t_mpc,delta_mpc*180/pi,'b.','LineWidth',1.0)
title('delta [deg]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('Steerwheelangle \delta [deg]','fontsize',12)
% legend('R\_aytot acc','C\_aytot acc','Location','southeast')
subplot(2,2,3)
plot(t_mpc(1:5:N_ame),abs(delta_ref(1:2:N_ref)-delta_mpc(1:5:N_ame))*180/pi,'b','LineWidth',1.0)
title('Error [deg]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('Steerwheelangle error \delta [deg]','fontsize',12)
subplot(2,2,2)
plot(t_ref,throttle_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,throttle_mpc,'b.','LineWidth',1.0)
title('throttle [-]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('throttle tr [-]','fontsize',12)
% legend('R\_axtot acc','C\_axtot acc','Location','southeast')
subplot(2,2,4)
plot(t_mpc(1:5:N_ame),abs(throttle_mpc(1:5:N_ame)-throttle_ref(1:2:N_ref)),'b','LineWidth',1.0)
title('Error [-]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('throttle error tr [-]','fontsize',12)
% saveas(gcf,".\written_data\8tr&delta_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
% saveas(gcf,".\written_data\8tr&delta_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")

% Calculate the jerks
psi_ddot_mpc = derivative(psi_dot_mpc,Ts);
jy_t = deriv2(vy_mpc,Ts);
jy_n =  zeros(1,length(t_mpc));
for i =1:1:length(t_mpc)
    jy_n(i) = psi_dot_mpc(i)*atx_mpc(i) + vx_mpc(i)*psi_ddot_mpc(i);
end
jy_mpc = jy_t+jy_n;

jx_t = deriv2(vx_mpc,Ts);
jx_n =  zeros(1,length(t_mpc));
for i =1:1:length(t_mpc)
    jx_n(i) = -psi_dot_mpc(i) * aty_mpc(i) - vy_mpc(i) * psi_ddot_mpc(i);
end
jx_mpc = jx_t+jx_n;


% Jerks
figure('name', 'jerks')
subplot(2,2,1)
plot(t_ref,jx_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,jx_mpc,'b.','LineWidth',1.0)
title('jerk x [m/s^3]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('jerk\_x [m/s^3]','fontsize',12)

subplot(2,2,3)
plot(t_mpc(1:5:N_ame),abs(jx_ref(1:2:N_ref)-jx_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s³]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('Error jx [m/s³]','fontsize',12)

subplot(2,2,2)
plot(t_ref,jy_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,jy_mpc,'b.','LineWidth',1.0)

title('jerk y [m/s³]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('jerk\_y [m/s³]','fontsize',12)

subplot(2,2,4)
plot(t_mpc(1:5:N_ame),abs(jy_ref(1:2:N_ref)-jy_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s³]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('Error jy [m/s³]','fontsize',12)

% saveas(gcf,".\written_data\9jerks_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
% saveas(gcf,".\written_data\9jerks_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")

if plot_MPC == 1
    figure(1);
    saveas(gcf,".\written_data\10states_loop_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
    saveas(gcf,".\written_data\10states_loop_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")
    figure(2)
    saveas(gcf,".\written_data\11path_loop_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
    saveas(gcf,".\written_data\11path_loop_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")
    figure(3)
    saveas(gcf,".\written_data\12tr&delta_loop_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
    saveas(gcf,".\written_data\12tr&delta_loop_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")   
    figure(5)
    saveas(gcf,".\written_data\13inputs_loop_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
    saveas(gcf,".\written_data\13inputs_loop_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output tracking
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% time_t = outputs_tracking.time';
delta_dot_mpc = sim_out.outputs_tracking.signals.values(:,1)';
throttle_dot_mpc = sim_out.outputs_tracking.signals.values(:,2)';
% time_t = output_motion_planning.time';
% delta_dot_mpc = output_motion_planning.signals.values(:,1)';
% throttle_dot_mpc = output_motion_planning.signals.values(:,2)';

% Inputs: delta_dot & throttle_dot
figure('name', 'tr_dot&delta_dot')
subplot(2,2,1)
plot(t_ref,delta_dot_ref*180/pi,'r.','LineWidth',1.0)
hold on
plot(t_mpc(1:5*2:N_ame),delta_dot_mpc*180/pi,'b.','LineWidth',1.0)
title('delta\_dot [deg/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('Steerwheelanglerate \delta\_dot [deg/s]','fontsize',12)
% legend('R\_aytot acc','C\_aytot acc','Location','southeast')
subplot(2,2,3)
plot(t_mpc(1:5*2:N_ame),abs(delta_dot_ref(1:2*2:N_ref)-delta_dot_mpc)*180/pi,'b','LineWidth',1.0)
title('Error [deg/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('Steerwheelanglerate error \delta [deg/s]','fontsize',12)
subplot(2,2,2)
plot(t_ref,throttle_dot_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc(1:5*2:N_ame),throttle_dot_mpc,'b.','LineWidth',1.0)
title('throttle\_dot [1/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('throttle tr [1/s]','fontsize',12)
% legend('R\_axtot acc','C\_axtot acc','Location','southeast')
subplot(2,2,4)
plot(t_mpc(1:5*2:N_ame),abs(throttle_dot_mpc-throttle_dot_ref(1:2*2:N_ref)),'b','LineWidth',1.0)
title('Error [-]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('throttle\_dot error  [1/s]','fontsize',12)
% saveas(gcf,".\written_data\14tr_dot&delta_dot_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
% saveas(gcf,".\written_data\14tr_dot&delta_dot_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")

fprintf('\n')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% saving data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% "time","x","y","vx","vy","ax","ay","jx","jy","psi","psi_dot","psi_ddot","throttle","delta","throttle_dot","delta_dot","aty","any","atx","anx"
data_mpc = struct();
% Struct data: 1xN

time_mpc_s = t_mpc(1,1001:end) - t_mpc(1,1001);
x_mpc_s = x_mpc(1,1001:end) - x_mpc(1,1001);
y_mpc_s = y_mpc(1,1001:end);
vx_mpc_s = vx_mpc(1,1001:end);
vy_mpc_s = vy_mpc(1,1001:end);
ax_mpc_s = ax_mpc(1,1001:end);
ay_mpc_s = ay_mpc(1,1001:end);
jx_mpc_s = jx_mpc(1,1001:end);
jy_mpc_s = jy_mpc(1,1001:end);
psi_mpc_s = psi_mpc(1,1001:end);
psi_dot_mpc_s = psi_dot_mpc(1,1001:end);
psi_ddot_mpc_s = psi_ddot_mpc(1,1001:end);
throttle_mpc_s = throttle_mpc(1,1001:end);
delta_mpc_s = delta_mpc(1,1001:end);
delta_mpc_s = 1/16.96*delta_mpc_s; % transforming back to front wheel angle

throttle_dot_mpc_s = zeros(1,size(time_mpc_s,2)-1);
throttle_dot_iter = throttle_dot_mpc(1,101:end);
iter = 1;
for curr = throttle_dot_iter(1,1:end-1)
    throttle_dot_mpc_s((iter-1)*10+1:(iter-1)*10+10) = curr*ones(1,int64(T_MPC/Ts));
    iter = iter +1;
end

delta_dot_mpc_s = zeros(1,size(time_mpc_s,2)-1);
delta_dot_iter = delta_dot_mpc(1,101:end);
iter = 1;
for curr = delta_dot_iter(1,1:end-1)
    delta_dot_mpc_s((iter-1)*10+1:(iter-1)*10+10) = curr*ones(1,int64(T_MPC/Ts));
    iter = iter +1;
end

delta_dot_mpc_s = 1/16.96*delta_dot_mpc_s; % transforming back to front wheel angle

aty_mpc_s = aty_mpc(1,1001:end);
any_mpc_s = any_mpc(1,1001:end);
atx_mpc_s = atx_mpc(1,1001:end);
anx_mpc_s = anx_mpc(1,1001:end);


data_mpc.time_cl = time_mpc_s;
N = size(data_mpc.time_cl,2) -1;
data_mpc.dt_cl = data_mpc.time_cl(1,2) - data_mpc.time_cl(1,1);
data_mpc.x_cl = x_mpc_s;
data_mpc.y_cl = y_mpc_s;
data_mpc.vx_cl =vx_mpc_s;
data_mpc.vy_cl = vy_mpc_s;
data_mpc.ax_cl = ax_mpc_s; % total acc
data_mpc.ay_cl = ay_mpc_s; % total acc
data_mpc.jx_cl = jx_mpc_s;
data_mpc.jy_cl = jy_mpc_s;
data_mpc.psi_cl = psi_mpc_s;
data_mpc.psi_dot_cl = psi_dot_mpc_s;
data_mpc.psi_ddot_cl = psi_ddot_mpc_s;
data_mpc.throttle_cl = throttle_mpc_s;
data_mpc.delta_cl = delta_mpc_s; % delta is the angle of the front wheel
data_mpc.throttle_dot_cl = throttle_dot_mpc_s;
data_mpc.delta_dot_cl = delta_dot_mpc_s; % delta is the angle of the front wheel
data_mpc.aty_cl = aty_mpc_s;
data_mpc.any_cl= any_mpc_s;
data_mpc.atx_cl = atx_mpc_s;
data_mpc.anx_cl = anx_mpc_s;
data_mpc.width = data_mpc.y_cl(1,end);
data_mpc.vx_start = data_mpc.vx_cl(1,1);


% Calculation of features
% f0: longitudinal acceleration
integrand = data_mpc.ax_cl .^  2;
f0_cal = 0;
for i = 1:1:N
    f0_cal = f0_cal + 0.5 * (integrand(i) + integrand(i+1) )* data_mpc.dt_cl;
end

% f1: lateral acceleration
integrand = data_mpc.ay_cl .^  2;
f1_cal = 0;
for i = 1:1:N
    f1_cal = f1_cal + 0.5 * (integrand(i) + integrand(i+1) )* data_mpc.dt_cl;
end
% f2: longitudinal jerk
integrand = data_mpc.jx_cl .^  2;
f2_cal = 0;
for i = 1:1:N
    f2_cal = f2_cal + 0.5 * (integrand(i) + integrand(i+1) )* data_mpc.dt_cl;
end
% f3: lateral jerk
integrand = data_mpc.jy_cl .^  2;
f3_cal = 0;
for i = 1:1:N
    f3_cal = f3_cal + 0.5 * (integrand(i) + integrand(i+1) )* data_mpc.dt_cl;
end
% f4: desired velocity
integrand = (data_mpc.vx_start - data_mpc.vx_cl) .^  2;
f4_cal = 0;
for i = 1:1:N
    f4_cal = f4_cal + 0.5 * (integrand(i) + integrand(i+1) )* data_mpc.dt_cl;
end
% f5: desired lane change
integrand = (data_mpc.width - data_mpc.y_cl) .^  2;
f5_cal = 0;
for i = 1:1:N
    f5_cal = f5_cal + 0.5 * (integrand(i) + integrand(i+1))* data_mpc.dt_cl;
end

fprintf("\n")
fprintf('Integrated feature values of the DATA ')
fprintf("\n")
fprintf('------------------------------')
fprintf("\n")
fprintf('integrand = plt.squeeze(data_mpc[ax_cl].^ 2)')
fprintf("\n")
fprintf('%i ',f0_cal)
fprintf("\n")
fprintf('integrand = plt.squeeze(data_mpc[ay_cl] .^  2)')
fprintf("\n")
fprintf('%i ',f1_cal)
fprintf("\n")
fprintf('integrand = plt.squeeze(data_mpc[jx_cl] .^  2)')
fprintf("\n")
fprintf('%i ',f2_cal)
fprintf("\n")
fprintf('integrand = plt.squeeze(data_mpc[jy_cl] .^  2)')
fprintf("\n")
fprintf('%i ',f3_cal)
fprintf("\n")
fprintf('integrand = plt.squeeze((desired_speed - data_mpc[vx_cl]) .^  2)')
fprintf("\n")
fprintf('%i ',f4_cal)
fprintf("\n")
fprintf('integrand = plt.squeeze((delta_lane - data_mpc[y_cl]) .^  2)')
fprintf("\n")
fprintf('%i ',f5_cal)
fprintf("\n")


data_mpc.features = [f0_cal,f1_cal,f2_cal,f3_cal,f4_cal,f5_cal];
close all

end

