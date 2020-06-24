%% Initialization
% Change throttle initial
clearvars
close all 
clc
import casadi.*
global x_sol_prev lam_prev tracking_lane_change iteration T_pl N data T_MPC iter_expected iteration_throttle plot_MPC V0
iteration_throttle = 1;
files = {'DCA2_V22.22_L3.47.csv'};
plot_MPC = 0;
N = 50; % Control horizon of one optimization of the MPC.
Tf = 40; % if want same length as reference lane change set Tf = 0

data = get_data(char(files(:,1)));
V0 = data.vx(1,1);
update_casadi_function = 1; % in order to save time when developing
[x_sol_prev,lam_prev] = Function_generation(data,N,update_casadi_function);
iteration = 1;

throttle_start = data.throttle(1,1);

% Simulation sampling time and duration
Ts = 0.01; % sampling rate Amesim - taken standard value
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
ameputgpar('Dynamics', 'V0', V0) % this command only is not working --> have to set manual in amesim block in simulink
sim_opt = amegetsimopt('Dynamics');
amerunsingle('Dynamics', sim_opt);

%% Load the CasADi function for the MPC planner

tracking_lane_change = Function.load('tracking_lane_change.casadi');
DM.set_precision(15);


%% run simulation simulink
open('generating_data_lane_change');
sim('generating_data_lane_change'); % This is running the simulink file

%% Figures and save data
% Remember that added a delay --> at time = zero --> controls are zero

% time_control = control_signals.time;
t_mpc = Results_States.time';
t_max = 23;
x_max = 550;
x_mpc  = Results_States.signals(1).values';  % Vector of 2551 points
y_mpc   = Results_States.signals(2).values'; 
vx_mpc  = Results_States.signals(3).values';
vy_mpc  = Results_States.signals(4).values';
psi_mpc = Results_States.signals(5).values'; 
psi_dot_mpc   = Results_States.signals(6).values';  
throttle_mpc = squeeze(Results_States.signals(7).values)';
delta_mpc = squeeze(Results_States.signals(8).values)';


% time_acc = Accelerations_output.time;
ax_mpc       = Accelerations_output.signals.values(:,1)';  % in the beginning --> undesired deaccelleration due to delay in controls.
ay_mpc       = Accelerations_output.signals.values(:,2)';  

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
font_ax = 14;
font = 16;
figure('name', 'Path')
plot(x_ref,y_ref,'r.','LineWidth',1.0)
hold on
plot(x_mpc,y_mpc,'b.','LineWidth',1.0)
title('Vehicle path','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('X [m]','fontsize',font,'fontweight','bold')
xlim([0, x_max])
ylabel('Y [m]','fontsize',font,'fontweight','bold')
% legend('R\_path','C\_path','Location','southeast')
saveas(gcf,".\written_data\1path_N"+string(N)+"_TMPC C"+string(T_MPC)+"_Tf"+string(Tf)+".png")
saveas(gcf,".\written_data\1path_N"+string(N)+"_TMPC C"+string(T_MPC)+"_Tf"+string(Tf)+".fig")
% Postion vs time
figure('name', 'Pos')
subplot(2,2,1)
plot(t_ref,x_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,x_mpc,'b.','LineWidth',1.0)
title('X [m]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('position X [m]','fontsize',font,'fontweight','bold')
% legend('R\_x path','C\_x path','Location','southeast')
subplot(2,2,3)
plot(t_mpc(1:5:N_ame),abs(x_ref(1:2:N_ref)-x_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('position error X [m]','fontsize',font,'fontweight','bold')

subplot(2,2,2)
plot(t_ref,y_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,y_mpc,'b.','LineWidth',1.0)
title('Y [m]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('position Y [m]','fontsize',font,'fontweight','bold')
% legend('R\_y path','C\_y path','Location','southeast')
subplot(2,2,4)
plot(t_ref(1:2:N_ref),abs(y_ref(1:2:N_ref)-y_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('position error Y [m]','fontsize',font,'fontweight','bold')
saveas(gcf,".\written_data\2xy_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
saveas(gcf,".\written_data\2xy_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")

% Velocity vs time
figure('name', 'Vel')
subplot(2,2,1)
plot(t_ref,vx_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,vx_mpc,'b.','LineWidth',1.0)
title('v_X [m/s]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('velocity X [m/s]','fontsize',font,'fontweight','bold')
% legend('R\_X vel','C\_X vel','Location','southeast')
subplot(2,2,3)
plot(t_mpc(1:5:N_ame),abs(vx_ref(1:2:N_ref)-vx_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('velocity error X [m/s]','fontsize',font,'fontweight','bold')
subplot(2,2,2)
plot(t_ref,vy_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,vy_mpc,'b.','LineWidth',1.0)
title('v_Y [m/s]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('velocity Y [m/s]','fontsize',font,'fontweight','bold')
% legend('R\_Y vel','C\_Y vel','Location','southeast')
subplot(2,2,4)
plot(t_mpc(1:5:N_ame),abs(vy_ref(1:2:N_ref)-vy_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('velocity error Y [m/s]','fontsize',font,'fontweight','bold')
saveas(gcf,".\written_data\3vxy_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
saveas(gcf,".\written_data\3vxy_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")

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
title('atY [m/s²]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('acceleration atY [m/s²]','fontsize',font,'fontweight','bold')
% legend('R\_aty acc','C\_aty acc','Location','southeast')
subplot(2,2,3)
plot(t_mpc(1:5:N_ame),abs(aty_ref(1:2:N_ref)-aty_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('acceleration error aty [m/s²]','fontsize',font,'fontweight','bold')
subplot(2,2,2)
plot(t_ref,any_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,any_mpc,'b.','LineWidth',1.0)
title('anY [m/s²]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('acceleration any [m/s²]','fontsize',font,'fontweight','bold')
% legend('R\_any acc','C\_any acc','Location','southeast')
subplot(2,2,4)
plot(t_mpc(1:5:N_ame),abs(any_ref(1:2:N_ref)-any_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('acceleration error anY [m/s²]','fontsize',font,'fontweight','bold')
saveas(gcf,".\written_data\4ay_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
saveas(gcf,".\written_data\4ay_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")
figure('name', 'Acc\_x')
subplot(2,2,1)
plot(t_ref,atx_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,atx_mpc,'b.','LineWidth',1.0)
title('atx [m/s²]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('acceleration atx [m/s²]','fontsize',font,'fontweight','bold')
% legend('R\_atx acc','C\_atx acc','Location','southeast')
subplot(2,2,3)
plot(t_mpc(1:5:N_ame),abs(atx_ref(1:2:N_ref)-atx_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('acceleration error atx [m/s²]','fontsize',font,'fontweight','bold')
subplot(2,2,2)
plot(t_ref,anx_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,anx_mpc,'b.','LineWidth',1.0)
title('anx [m/s²]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('acceleration anx [m/s²]','fontsize',font,'fontweight','bold')
% legend('R\_anx acc','C\_anx acc','Location','southeast')
subplot(2,2,4)
plot(t_mpc(1:5:N_ame),abs(anx_ref(1:2:N_ref)-anx_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('acceleration error anx [m/s²]','fontsize',font,'fontweight','bold')
saveas(gcf,".\written_data\5ax_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
saveas(gcf,".\written_data\5ax_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")
%%%%%%%%%%%%%%%%%%%%%%%%%
figure('name', 'Acc\_tot')
subplot(2,2,1)
plot(t_ref,ay_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,ay_mpc,'b.','LineWidth',1.0)
title('aytot [m/s²]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('acceleration aytot [m/s²]','fontsize',font,'fontweight','bold')
% legend('R\_aytot acc','C\_aytot acc','Location','southeast')
subplot(2,2,3)
plot(t_mpc(1:5:N_ame),abs(ay_ref(1:2:N_ref)-ay_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('acceleration error aytot [m/s²]','fontsize',font,'fontweight','bold')
subplot(2,2,2)
plot(t_ref,ax_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,ax_mpc,'b.','LineWidth',1.0)
title('axtot [m/s²]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('acceleration axtot [m/s²]','fontsize',font,'fontweight','bold')
% legend('R\_axtot acc','C\_axtot acc','Location','southeast')
subplot(2,2,4)
plot(t_mpc(1:5:N_ame),abs(ax_mpc(1:5:N_ame)-ax_ref(1:2:N_ref)),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('acceleration error axtot [m/s²]','fontsize',font,'fontweight','bold')
saveas(gcf,".\written_data\6atot_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
saveas(gcf,".\written_data\6atot_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")
%%%%%%%%%%%%%%%%%%%%%
% Yaw and Yaw rate 
figure('name', 'Yaw')
subplot(2,2,1)
plot(t_ref,psi_ref*180/pi,'r.','LineWidth',1.0)
hold on
plot(t_mpc,psi_mpc*180/pi,'b.','LineWidth',1.0)
title('\psi [deg]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('yaw angle \psi [deg]','fontsize',font,'fontweight','bold')
% legend('R\_yaw','C\_yaw','Location','northeast')
subplot(2,2,3)
plot(t_mpc(1:5:N_ame),abs(psi_ref(1:2:N_ref)-psi_mpc(1:5:N_ame))*180/pi,'b','LineWidth',1.0)
title('Error [deg]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('yaw angle error \psi [deg]','fontsize',font,'fontweight','bold')
subplot(2,2,2)
plot(t_ref,psi_dot_ref*180/pi,'r.','LineWidth',1.0)
hold on
plot(t_mpc,psi_dot_mpc*180/pi,'b.','LineWidth',1.0)
title('d\psi [deg/s]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('yaw velocity d\psi [deg/s]','fontsize',font,'fontweight','bold')
% legend('R\_yaw vel','C\_yaw vel','Location','northeast')
subplot(2,2,4)
plot(t_mpc(1:5:N_ame),abs(psi_dot_ref(1:2:N_ref)-psi_dot_mpc(1:5:N_ame))*180/pi,'b','LineWidth',1.0)
title('Error [deg/s]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('yaw velocity error d\psi [deg/s]','fontsize',font,'fontweight','bold')
saveas(gcf,".\written_data\7psi_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
saveas(gcf,".\written_data\7psi_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")

%  delta & throttle
figure('name', 'tr&delta')
subplot(2,2,1)
plot(t_ref,delta_ref*180/pi,'r.','LineWidth',1.0)
hold on
plot(t_mpc,delta_mpc*180/pi,'b.','LineWidth',1.0)
title('delta [deg]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('Steerwheelangle \delta [deg]','fontsize',font,'fontweight','bold')
% legend('R\_aytot acc','C\_aytot acc','Location','southeast')
subplot(2,2,3)
plot(t_mpc(1:5:N_ame),abs(delta_ref(1:2:N_ref)-delta_mpc(1:5:N_ame))*180/pi,'b','LineWidth',1.0)
title('Error [deg]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('Steerwheelangle error \delta [deg]','fontsize',font,'fontweight','bold')
subplot(2,2,2)
plot(t_ref,throttle_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,throttle_mpc,'b.','LineWidth',1.0)
title('throttle [-]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('throttle tr [-]','fontsize',font,'fontweight','bold')
% legend('R\_axtot acc','C\_axtot acc','Location','southeast')
subplot(2,2,4)
plot(t_mpc(1:5:N_ame),abs(throttle_mpc(1:5:N_ame)-throttle_ref(1:2:N_ref)),'b','LineWidth',1.0)
title('Error [-]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('throttle error tr [-]','fontsize',font,'fontweight','bold')
saveas(gcf,".\written_data\8tr&delta_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
saveas(gcf,".\written_data\8tr&delta_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")

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
title('jerk x [m/s^3]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('jerk\_x [m/s^3]','fontsize',font,'fontweight','bold')

subplot(2,2,3)
plot(t_mpc(1:5:N_ame),abs(jx_ref(1:2:N_ref)-jx_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s³]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('Error jx [m/s³]','fontsize',font,'fontweight','bold')

subplot(2,2,2)
plot(t_ref,jy_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc,jy_mpc,'b.','LineWidth',1.0)

title('jerk y [m/s³]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('jerk\_y [m/s³]','fontsize',font,'fontweight','bold')

subplot(2,2,4)
plot(t_mpc(1:5:N_ame),abs(jy_ref(1:2:N_ref)-jy_mpc(1:5:N_ame)),'b','LineWidth',1.0)
title('Error [m/s³]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('Error jy [m/s³]','fontsize',font,'fontweight','bold')

saveas(gcf,".\written_data\9jerks_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
saveas(gcf,".\written_data\9jerks_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")

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
delta_dot_mpc = outputs_tracking.signals.values(:,1)';
throttle_dot_mpc = outputs_tracking.signals.values(:,2)';
% time_t = output_motion_planning.time';
% delta_dot_mpc = output_motion_planning.signals.values(:,1)';
% throttle_dot_mpc = output_motion_planning.signals.values(:,2)';

% Inputs: delta_dot & throttle_dot
figure('name', 'tr_dot&delta_dot')
subplot(2,2,1)
plot(t_ref,delta_dot_ref*180/pi,'r.','LineWidth',1.0)
hold on
plot(t_mpc(1:5*2:N_ame),delta_dot_mpc*180/pi,'b.','LineWidth',1.0)
title('delta\_dot [deg/s]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('Steerwheelanglerate \delta\_dot [deg/s]','fontsize',font,'fontweight','bold')
% legend('R\_aytot acc','C\_aytot acc','Location','southeast')
subplot(2,2,3)
plot(t_mpc(1:5*2:N_ame),abs(delta_dot_ref(1:2*2:N_ref)-delta_dot_mpc)*180/pi,'b','LineWidth',1.0)
title('Error [deg/s]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('Steerwheelanglerate error \delta [deg/s]','fontsize',font,'fontweight','bold')
subplot(2,2,2)
plot(t_ref,throttle_dot_ref,'r.','LineWidth',1.0)
hold on
plot(t_mpc(1:5*2:N_ame),throttle_dot_mpc,'b.','LineWidth',1.0)
title('throttle\_dot [1/s]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('throttle tr [1/s]','fontsize',font,'fontweight','bold')
% legend('R\_axtot acc','C\_axtot acc','Location','southeast')
subplot(2,2,4)
plot(t_mpc(1:5*2:N_ame),abs(throttle_dot_mpc-throttle_dot_ref(1:2*2:N_ref)),'b','LineWidth',1.0)
title('Error [-]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('t [s]','fontsize',font,'fontweight','bold')
xlim([0, t_max])
ylabel('throttle\_dot error  [1/s]','fontsize',font,'fontweight','bold')
saveas(gcf,".\written_data\14tr_dot&delta_dot_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".png")
saveas(gcf,".\written_data\14tr_dot&delta_dot_N"+string(N)+"_TMPC "+string(T_MPC)+"_Tf"+string(Tf)+".fig")

fprintf('\n')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Write data to a csv file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
name = char(files(:,1));
t_mpc_s = t_mpc(1,1001:end) - t_mpc(1,1001);
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

throttle_dot_mpc_s = zeros(1,size(t_mpc_s,2));
throttle_dot_iter = throttle_dot_mpc(1,101:end);
iter = 1;
for curr = throttle_dot_iter(1,1:end-1)
    throttle_dot_mpc_s((iter-1)*10+1:(iter-1)*10+10) = curr*ones(1,int64(T_MPC/Ts));
    iter = iter +1;
end
throttle_dot_mpc_s(end) = throttle_dot_iter(end);

delta_dot_mpc_s = zeros(1,size(t_mpc_s,2));
delta_dot_iter = delta_dot_mpc(1,101:end);
iter = 1;
for curr = delta_dot_iter(1,1:end-1)
    delta_dot_mpc_s((iter-1)*10+1:(iter-1)*10+10) = curr*ones(1,int64(T_MPC/Ts));
    iter = iter +1;
end
delta_dot_mpc_s(end) = delta_dot_iter(end);
delta_dot_mpc_s = 1/16.96*delta_dot_mpc_s; % transforming back to front wheel angle

aty_mpc_s = aty_mpc(1,1001:end);
any_mpc_s = any_mpc(1,1001:end);
atx_mpc_s = atx_mpc(1,1001:end);
anx_mpc_s = anx_mpc(1,1001:end);


% Save values
M = [t_mpc_s', x_mpc_s',y_mpc_s',vx_mpc_s',vy_mpc_s',ax_mpc_s',ay_mpc_s',jx_mpc_s',jy_mpc_s',psi_mpc_s',psi_dot_mpc_s',psi_ddot_mpc_s',throttle_mpc_s',delta_mpc_s',throttle_dot_mpc_s',delta_dot_mpc_s',aty_mpc_s',any_mpc_s',atx_mpc_s',anx_mpc_s'];
% Convert cell to a table and use first row as variable names
T = array2table(M,'VariableNames',{'time','x','y','vx','vy','ax','ay','jx','jy','psi','psi_dot','psi_ddot','throttle','delta','throttle_dot','delta_dot','aty','a_ny','atx','anx'});
% Write the table to a CSV file
writetable(T,convertStringsToChars(".\written_data\TRData_V"+convertCharsToStrings(name(7:11))+"_L"+ convertCharsToStrings(name(14:17))+".csv"))
disp('CSV-file written')

