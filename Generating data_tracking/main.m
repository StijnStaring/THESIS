%% Initialization

clear all
close all
clc
import casadi.*
load('previous_solution');
load('reference')
?
global width_road x_sol_prev lam_prev theta planner_lane_change iteration vx_desired


% N = 100 --> used in python code
% Initial conditions 
% current_states = [0,0,80/3.6,0,0,0];

theta = [4,5,6,1,2];
width_road = 3.46990715;
lane_change = 1;
x_sol_prev = previous_solution{1, 1}.x_sol_prev;
lam_prev = previous_solution{1, 1}.lam_prev;
iteration = 1;
vx_desired = 80/3.6;


% Simulation sampling time and duration
% Ts = 0.01;
% Ts = 0.01;
% Ts = 1;
% Tf = 10;
% Tf = 0.1;
% Ts = 0.1;
Ts = 0.01;
Ts_MP = 0.1;
Tf = 7;
% Tf = 0.03;

% Set the initial speed in the Amesim model
addpath(fullfile(getenv('AME'),'scripting','matlab','amesim'));
! AMECirChecker -g -q --nobackup --nologfile Dynamics.ame
! AMELoad Dynamics.ame
ameputgpar('Dynamics', 'V0', 80/3.6)
sim_opt = amegetsimopt('Dynamics');
amerunsingle('Dynamics', sim_opt);

%% Load the CasADi function for the MPC planner
if lane_change == 1
    planner_lane_change = Function.load('planner_lane_change.casadi');
    DM.set_precision(15);
end

%% run simulation simulink
open('generating_data_lane_change');
sim('generating_data_lane_change'); % This is running the simulink file

%% Figures and save data
time = Results_States.time;
x  = Results_States.signals(1).values;  
y   = Results_States.signals(2).values; 
Vx  = Results_States.signals(3).values;
Vy  = Results_States.signals(4).values;
yaw = Results_States.signals(5).values.*(180/pi); 
r   = Results_States.signals(6).values.*(180/pi);  
 
Control_signals = control_signals.signals.values;
time_control = control_signals.time;
delta = control_signals.signals.values(:,1).*(180/pi);
throttle = control_signals.signals.values(:,2);
brake = control_signals.signals.values(:,3);

time_acc = Accelerations.time;
ax       = Accelerations.signals(1).values;  
ay       = Accelerations.signals(2).values;  

% % Save values
% [X, Y, Vx, Vy, r, yaw, steering_degree, throttle, brake, ax, ay]
% M = [time, X(:,1), Y(:,1), vx(:,1), vy(:,1), R(:,1), Phi(:,1), Control_signals(:,1).*(180/pi), Control_signals(:,2), Control_signals(:,3), ax, ay];
% % Convert cell to a table and use first row as variable names
% T = array2table(M,'VariableNames',{'time', 'X', 'Y', 'Vx', 'Vy', 'r', 'yaw', 'steering_deg', 'throttle', ...
% 'brake', 'ax', 'ay'});
% % Write the table to a CSV file
% writetable(T,['.\Datasets\Dataset_Xchange',num2str(X_change),'_V0=',num2str(v0*3.6),'.csv'])


% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Path
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('name','Path driven')
plot(x,y,'b','LineWidth',1)
xlabel('x [m]','fontsize',12)
ylabel('y [m]','fontsize',12)
grid on


% States
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('name','States')
subplot(2,3,1)
plot(time,x,'r','LineWidth',1)
xlabel('t [s]','fontsize',12)
ylabel('x [m]','fontsize',12)
grid on

subplot(2,3,2)
plot(time,y,'r','LineWidth',1)
xlabel('t [s]','fontsize',12)
ylabel('y [m]','fontsize',12)
grid on

subplot(2,3,3)
plot(time,Vx,'r','LineWidth',1)
xlabel('t [s]','fontsize',12)
ylabel('vx [m/s]','fontsize',12)
grid on

subplot(2,3,4)
plot(time,Vy,'r','LineWidth',1)
xlabel('t [s]','fontsize',12)
ylabel('vy [m/s]','fontsize',12)
grid on

subplot(2,3,5)
plot(time,yaw,'r','LineWidth',1)
xlabel('t [s]','fontsize',12)
ylabel('yaw [°]','fontsize',12)
grid on

subplot(2,3,6)
plot(time,r,'r','LineWidth',1)
xlabel('t [s]','fontsize',12)
ylabel('r [°/s]','fontsize',12)
grid on
%Controls
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('name','Controls')
subplot(1,3,1)
plot(time_control,delta,'b','LineWidth',1)
xlabel('t [s]','fontsize',12)
ylabel('delta [°]','fontsize',12)
grid on

subplot(1,3,2)
plot(time_control,throttle,'b','LineWidth',1)
xlabel('t [s]','fontsize',12)
ylabel('throttle [-]','fontsize',12)
grid on

subplot(1,3,3)
plot(time_control,brake,'b','LineWidth',1)
xlabel('t [s]','fontsize',12)
ylabel('brake [-]','fontsize',12)
grid on
%Acc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('name','Acceleration')
subplot(1,2,1)
plot(time_acc,ax,'g','LineWidth',1)
xlabel('t [s]','fontsize',12)
ylabel('ax [m/s^2]','fontsize',12)
grid on

subplot(1,2,2)
plot(time_acc,ay,'g','LineWidth',1)
xlabel('t [s]','fontsize',12)
ylabel('ay [m/s^2]','fontsize',12)
grid on
%Plotting the global velocity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vg = zeros(2,max(size(Vx)));
for i = 1:1:max(size(Vx))
    RotBA = [cos(yaw(i)),-sin(yaw(i));
            sin(yaw(i)), cos(yaw(i))];
    Vr = [Vx(i);
          Vy(i)];
    Vg(:,i) = RotBA*Vr;
end
Vx_g = Vg(1,:);
Vy_g = Vg(2,:);
figure('name','Global velocity')
subplot(1,2,1)
plot(time,Vx_g,'k','LineWidth',1)
xlabel('t [s]','fontsize',12)
ylabel('Vx_g [m/s]','fontsize',12)
grid on

subplot(1,2,2)
plot(time,Vy_g,'k','LineWidth',1)
xlabel('t [s]','fontsize',12)
ylabel('Vy_g [m/s]','fontsize',12)
grid on



%Output of motion planning
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

