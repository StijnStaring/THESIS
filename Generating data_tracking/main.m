%% Initialization

clear vars
close all 
clc
import casadi.*
global x_sol_prev lam_prev tracking_lane_change iteration dt N data N_sim

files = {'DATA2_V22.22_L3.47.csv'};
N = 50; % Control horizon of one optimization of the MPC.
sampling_rate = 100;
dt = 1/sampling_rate;
data = get_data(char(files(:,1)),sampling_rate,N);
N_sim = length(data.time) - N;
update_casadi_function = 1; % in order to save time when developing
[x_sol_prev,lam_prev] = Function_generation(data,N,update_casadi_function);
iteration = 1;
lane_change = 1;


% Simulation sampling time and duration
Ts = dt;
Ts_MP = dt;
Tf = dt*N_sim;

% Set the initial speed in the Amesim model
addpath(fullfile(getenv('AME'),'scripting','matlab','amesim'));
! AMECirChecker -g -q --nobackup --nologfile Dynamics.ame
! AMELoad Dynamics.ame
% Adjusted this state manually
ameputgpar('Dynamics', 'V0', 80/3.6)
sim_opt = amegetsimopt('Dynamics');
amerunsingle('Dynamics', sim_opt);

%% Load the CasADi function for the MPC planner
if lane_change == 1
    tracking_lane_change = Function.load('tracking_lane_change.casadi');
    DM.set_precision(15);
end

%% run simulation simulink
open('generating_data_lane_change');
sim('generating_data_lane_change'); % This is running the simulink file

%% Figures and save data
% Remember that added a delay --> at time = zero --> controls are zero
Control_signals = control_signals.signals.values;
% time_control = control_signals.time;
t_mpc = Results_States.time';
t_ref_mpc = 0:dt:dt*N_sim;
t_max = 8.5;
x_max = 160;
x_mpc  = Results_States.signals(1).values';  
y_mpc   = Results_States.signals(2).values'; 
vx_mpc  = Results_States.signals(3).values';
vy_mpc  = Results_States.signals(4).values';
psi_mpc = Results_States.signals(5).values'; 
psi_dot_mpc   = Results_States.signals(6).values';  
delta_mpc = control_signals.signals.values(:,1);
delta_mpc = delta_mpc(1:end-1)';
throttle_mpc = control_signals.signals.values(:,2);
throttle_mpc = throttle_mpc(1:end-1)';
brake_mpc = control_signals.signals.values(:,3);
brake_mpc = brake_mpc(1:end-1)'; % In simulink --> can brake harder than accelerate.

time_acc = Accelerations.time;
ax_tot_mpc       = Accelerations.signals(1).values';  % in the beginning --> undesired deaccelleration due to delay in controls.
ay_tot_mpc       = Accelerations.signals(2).values';  

x_ref = data.x';
y_ref = data.y';
vx_ref = data.vx';
vy_ref = data.vy';
psi_ref = data.psi';
psi_dot_ref = data.psi_dot';

x_ref_mpc = x_ref(1:N_sim+1);
y_ref_mpc = y_ref(1:N_sim+1);
vx_ref_mpc = vx_ref(1:N_sim+1);
vy_ref_mpc = vy_ref(1:N_sim+1);
psi_ref_mpc = psi_ref(1:N_sim+1);
psi_dot_ref_mpc = psi_dot_ref(1:N_sim+1);

% % Save values
% [X, Y, Vx, Vy, r, yaw, steering_degree, throttle, brake, ax, ay]
% M = [time, X(:,1), Y(:,1), vx(:,1), vy(:,1), R(:,1), Phi(:,1), Control_signals(:,1).*(180/pi), Control_signals(:,2), Control_signals(:,3), ax, ay];
% % Convert cell to a table and use first row as variable names
% T = array2table(M,'VariableNames',{'time', 'X', 'Y', 'Vx', 'Vy', 'r', 'yaw', 'steering_deg', 'throttle', ...
% 'brake', 'ax', 'ay'});
% % Write the table to a CSV file
% writetable(T,['.\Datasets\Dataset_Xchange',num2str(X_change),'_V0=',num2str(v0*3.6),'.csv'])


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Path (x,y)
figure('name', 'Path')
plot(x_ref_mpc,y_ref_mpc,'r.','LineWidth',1.0)
hold on
plot(x_mpc,y_mpc,'b.','LineWidth',1.0)
title('Vehicle path','fontsize',12,'fontweight','bold')
xlabel('X [m]','fontsize',12)
xlim([0, x_max])
ylabel('Y [m]','fontsize',12)
legend('Reference path','Calculated path','Location','southeast')


% Postion vs time
figure('name', 'Pos')
subplot(2,2,1)
plot(t_ref_mpc,x_ref_mpc,'r.','LineWidth',1.0)
hold on
plot(t_mpc,x_mpc,'b.','LineWidth',1.0)
title('X [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position X [m]','fontsize',12)
legend('Reference x path','Calculated x path','Location','southeast')
subplot(2,2,3)
plot(t_mpc,abs(x_ref_mpc-x_mpc),'b.','LineWidth',1.0)
title('Error [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position error X [m]','fontsize',12)
subplot(2,2,2)
plot(t_ref_mpc,y_ref_mpc,'r.','LineWidth',1.0)
hold on
plot(t_mpc,y_mpc,'b.','LineWidth',1.0)
title('Y [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position Y [m]','fontsize',12)
legend('Reference y path','Calculated y path','Location','southeast')
subplot(2,2,4)
plot(t_ref_mpc,abs(y_ref_mpc-y_mpc),'b','LineWidth',1.0)
title('Error [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position error Y [m]','fontsize',12)

% Velocity vs time
figure('name', 'Vel')
subplot(2,2,1)
plot(t_ref_mpc,vx_ref_mpc,'r.','LineWidth',1.0)
hold on
plot(t_mpc,vx_mpc,'b.','LineWidth',1.0)
title('v_X [m/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('velocity X [m/s]','fontsize',12)
legend('Reference X velocity','Calculated X velocity','Location','southeast')
subplot(2,2,3)
plot(t_mpc,abs(vx_ref_mpc-vx_mpc),'b','LineWidth',1.0)
title('Error [m/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('velocity error X [m/s]','fontsize',12)
subplot(2,2,2)
plot(t_ref_mpc,vy_ref_mpc,'r.','LineWidth',1.0)
hold on
plot(t_mpc,vy_mpc,'b.','LineWidth',1.0)
title('v_Y [m/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('velocity Y [m/s]','fontsize',12)
legend('Reference Y velocity','Calculated Y velocity','Location','southeast')
subplot(2,2,4)
plot(t_mpc,abs(vy_ref_mpc-vy_mpc),'b','LineWidth',1.0)
title('Error [m/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('velocity error Y [m/s]','fontsize',12)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Need plots of ay_tot, aty, any, ax_tot, atx, any
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aty_mpc = derivative(vy_mpc,dt);
any_mpc = vx_mpc.*psi_dot_mpc;
% ay_tot_mpc = aty_mpc + any_mpc;
atx_mpc = derivative(vx_mpc,dt);
anx_mpc = -vy_mpc.*psi_dot_mpc;
% ax_tot_mpc = atx_mpc + anx_mpc;

aty_ref_mpc = derivative(vy_ref_mpc,dt);
any_ref_mpc = vx_ref_mpc.*psi_dot_ref_mpc;
ay_tot_ref_mpc = aty_ref_mpc + any_ref_mpc;
atx_ref_mpc = derivative(vx_ref_mpc,dt);
anx_ref_mpc = -vy_ref_mpc.*psi_dot_ref_mpc;
ax_tot_ref_mpc = atx_ref_mpc + anx_ref_mpc;

figure('name', 'Acc\_y')
subplot(2,2,1)
plot(t_ref_mpc,aty_ref_mpc,'r.','LineWidth',1.0)
hold on
plot(t_mpc,aty_mpc,'b.','LineWidth',1.0)
title('atY [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration atY [m/s²]','fontsize',12)
legend('Reference aty acceleration','Calculated aty acceleration','Location','southeast')
subplot(2,2,3)
plot(t_mpc,abs(aty_ref_mpc-aty_mpc),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration error aty [m/s²]','fontsize',12)
subplot(2,2,2)
plot(t_ref_mpc,any_ref_mpc,'r.','LineWidth',1.0)
hold on
plot(t_mpc,any_mpc,'b.','LineWidth',1.0)
title('anY [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration any [m/s²]','fontsize',12)
legend('Reference any acceleration','Calculated any acceleration','Location','southeast')
subplot(2,2,4)
plot(t_mpc,abs(any_ref_mpc-any_mpc),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration error anY [m/s²]','fontsize',12)

figure('name', 'Acc\_x')
subplot(2,2,1)
plot(t_ref_mpc,atx_ref_mpc,'r.','LineWidth',1.0)
hold on
plot(t_mpc,atx_mpc,'b.','LineWidth',1.0)
title('atx [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration atx [m/s²]','fontsize',12)
legend('Reference atx acceleration','Calculated atx acceleration','Location','southeast')
subplot(2,2,3)
plot(t_mpc,abs(atx_ref_mpc-atx_mpc),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration error atx [m/s²]','fontsize',12)
subplot(2,2,2)
plot(t_ref_mpc,anx_ref_mpc,'r.','LineWidth',1.0)
hold on
plot(t_mpc,anx_mpc,'b.','LineWidth',1.0)
title('anx [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration anx [m/s²]','fontsize',12)
legend('Reference anx acceleration','Calculated anx acceleration','Location','southeast')
subplot(2,2,4)
plot(t_mpc,abs(anx_ref_mpc-anx_mpc),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration error anx [m/s²]','fontsize',12)

figure('name', 'Acc\_tot')
subplot(2,2,1)
plot(t_ref_mpc,ay_tot_ref_mpc,'r.','LineWidth',1.0)
hold on
plot(t_mpc,ay_tot_mpc,'b.','LineWidth',1.0)
title('aytot [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration aytot [m/s²]','fontsize',12)
legend('Reference aytot acceleration','Calculated aytot acceleration','Location','southeast')
subplot(2,2,3)
plot(t_mpc,abs(ay_tot_ref_mpc-ay_tot_mpc),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration error aytot [m/s²]','fontsize',12)
subplot(2,2,2)
plot(t_ref_mpc,ax_tot_ref_mpc,'r.','LineWidth',1.0)
hold on
plot(t_mpc,ax_tot_mpc,'b.','LineWidth',1.0)
title('axtot [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration axtot [m/s²]','fontsize',12)
legend('Reference axtot acceleration','Calculated axtot acceleration','Location','southeast')
subplot(2,2,4)
plot(t_mpc,abs(ax_tot_mpc-ax_tot_ref_mpc),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration error axtot [m/s²]','fontsize',12)

% Yaw and Yaw rate 
figure('name', 'Yaw')
subplot(2,2,1)
plot(t_ref_mpc,psi_ref_mpc*180/pi,'r.','LineWidth',1.0)
hold on
plot(t_mpc,psi_mpc*180/pi,'b.','LineWidth',1.0)
title('\psi [deg]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('yaw angle \psi [deg]','fontsize',12)
legend('Reference yaw angle','Calculated yaw angle','Location','northeast')
subplot(2,2,3)
plot(t_mpc,abs(psi_ref_mpc-psi_mpc)*180/pi,'b','LineWidth',1.0)
title('Error [deg]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('yaw angle error \psi [deg]','fontsize',12)
subplot(2,2,2)
plot(t_ref_mpc,psi_dot_ref_mpc*180/pi,'r.','LineWidth',1.0)
hold on
plot(t_mpc,psi_dot_mpc*180/pi,'b.','LineWidth',1.0)
title('d\psi [deg/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('yaw velocity d\psi [deg/s]','fontsize',12)
legend('Reference yaw velocity','Calculated yaw velocity','Location','northeast')
subplot(2,2,4)
plot(t_mpc,abs(psi_dot_ref_mpc-psi_dot_mpc)*180/pi,'b','LineWidth',1.0)
title('Error [deg/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('yaw velocity error d\psi [deg/s]','fontsize',12)

% Inputs
% delta & throttle
figure('name', 'inputs')
subplot(2,1,1)
plot(t_mpc(1:end-1),delta_mpc*180/pi,'b','LineWidth',1.0)
title('\delta [deg]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('\delta [deg]','fontsize',12)
subplot(2,1,2)
plot(t_mpc(1:end-1),throttle_mpc,'b','LineWidth',1.0)
title('Throttle coefficient t_r [-]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('throttle coefficient t_r [-]','fontsize',12)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output of motion planning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time_MP = output_motion_planning.time;
delta_MP = output_motion_planning.signals.values(:,1);
throttle_MP = output_motion_planning.signals.values(:,2);

fprintf('\n')
disp('-------------------------------------------')
disp('OUTPUT OF MOTION PLANNER')
disp('-------------------------------------------')
disp('Output of time given by the motion planner: ')
% disp(time_MP')
fprintf('\n')
disp('Output of steerwheelangle [°] given by the motion planner: ')
% disp((delta_MP.*180/pi)')
fprintf('\n')
disp('Output of throttle [-] given by the motion planner: ')
% disp(throttle_MP')
fprintf('\n')
disp('Simulation terminated!')

