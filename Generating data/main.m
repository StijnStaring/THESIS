%% Initialization

clear all
close all
clc

global centerline1 centerline2 left_line right_line middle_line
% Load the map, scanned offline in Prescan at 5m/s with frequency of 100Hz
load('line_data1.mat')
centerline1 = line_data.centerline_right;
centerline2 = line_data.centerline_left;
left_line = line_data.left_line;
right_line = line_data.right_line;
middle_line = line_data.middle_line;

% Initial conditions 
% Do NOT change:
x0 = 0;
y0 = 0;
phi0 = 0;
% You can change:
v0 = 80/3.6; % [m/s]

% Simulation sampling time
Ts = 0.01;
% Simulation duration
Tf = 300;

% Set the initial speed in the Amesim model
addpath(fullfile(getenv('AME'),'scripting','matlab','amesim'));
! AMECirChecker -g -q --nobackup --nologfile Dynamics.ame
! AMELoad Dynamics.ame
ameputgpar('Dynamics', 'V0', v0)
sim_opt = amegetsimopt('Dynamics');
amerunsingle('Dynamics', sim_opt);

%% Load the CasADi function for the MPC planner
global Planner
Plannerstr = load('Planner.mat');
Planner = Plannerstr.Planner;

%% run simulation
open('amesimcar_pathplannerv2');
sim('amesimcar_pathplannerv2');

%% Figures and save data

time = Results_States.time;
x   = Results_States.signals(1).values;  
y   = Results_States.signals(2).values;  
Vx  = Results_States.signals(3).values;  
Vy  = Results_States.signals(4).values;  
phi = wrapToPi(Results_States.signals(5).values);  
r   = Results_States.signals(6).values; 

Control_signals = control_signals.signals.values;

time_acc = Accelerations.time;
ax       = Accelerations.signals(1).values;  
ay       = Accelerations.signals(2).values;  


% Save Values 
values = zeros(length(1:length(time)-50),10);
for i = 1:length(time)-50
    
    X = x(i:i+49);
    Y = y(i:i+49);
    VX = Vx(i:i+49);
    VY = Vy(i:i+49);
    PHI = phi(i:i+49);
    R = r(i:i+49);
    
    ROT = [cos(PHI(1)) sin(PHI(1));
           -sin(PHI(1)) cos(PHI(1))];
    
    TransformedCoordinates = zeros(2,length(X));
    for j = 1:length(X)
        TransformedCoordinates(:,j) = ROT*[X(j);Y(j)] - ROT*[X(1); Y(1)];
    end
            
    Xt = TransformedCoordinates(1,:);
    Yt = TransformedCoordinates(2,:);
    
    t = 0:0.01:0.49;
    ty = linspace(0,0.49,3);
    
    coeff_x = polyfit(t,Xt,2);
    
    pp = spline(ty,Yt([1 25 50]));
    Bs = fn2fm(pp, 'B-');
    coeff_y = Bs.coefs;
    
    values(i,:) = [X(1) Y(1) VX(1) VY(1) PHI(1) R(1) coeff_x(1:end-1) coeff_y(2:end)];
       
end

T = array2table(values,'VariableNames',{'X', 'Y', 'Vx', 'Vy', 'Phi', 'r', ...
   'px2', 'px1', 'py1', 'py2'});
% writetable(T,'Datasets\Dataset_LK2.csv')

M = [centerline1(1,:)', centerline1(2,:)', centerline1(3,:)', left_line(1,:)', left_line(2,:)', right_line(1,:)', right_line(2,:)'];
T = array2table(M,'VariableNames',{'X_cen', 'Y_cen', 'PHI_cen', 'X_left', 'Y_left', 'X_right', 'Y_right'});
% writetable(T,'Datasets\Dataset_LK2_lines.csv')

state = figure(4);
subplot(2,3,1)
plot(time, Vx, 'b')
xlabel('time (s)')
ylabel('Vx (m/s)')
xlim([0 time(end)])

subplot(2,3,2)
plot(time, Vy)
xlabel('time (s)')
ylabel('Vy (m/s)')
xlim([0 time(end)])

subplot(2,3,3)
plot(time, r)
xlabel('time (s)')
ylabel('r (rad/s)')
xlim([0 time(end)])

subplot(2,3,4)
plot(time, phi*180/pi)
xlabel('time (s)')
ylabel('\phi ')
xlim([0 time(end)])

subplot(2,3,5)
plot(time, x)
xlabel('time (s)')
ylabel('X(m)')
xlim([0 time(end)])

subplot(2,3,6)
plot(time, y)
xlabel('time (s)')
ylabel('Y(m)')
xlim([0 time(end)])


input = figure(5);

subplot(2,2,1)
plot(time(1:length(Control_signals(:,1))), Control_signals(:,1)'.*(180/pi), 'k')
xlabel('time (s)')
ylabel('\delta_{steer} (deg)')
xlim([0 time(end)])

subplot(2,2,2)
plot(time(1:length(Control_signals(:,1))), Control_signals(:,2)', 'k')
xlabel('time (s)')
ylabel('throttle')
xlim([0 time(end)])

subplot(2,2,3)
stairs(time(1:length(Control_signals(:,1))), Control_signals(:,3)', 'k')
xlabel('time (s)')
ylabel('braking')
xlim([0 time(end)])

subplot(2,2,4)
plot(time_acc, ax/9.81, 'k'); hold on
plot(time_acc, ay/9.81, 'r');
xlabel('time (s)')
ylabel('acceleration (g)')
xlim([0 time_acc(end)])
ylim([-1 1])
legend('ax', 'ay')

output = figure(6);
plot(x, y, 'k'), hold on
plot(right_line(1,:), right_line(2,:), 'r')
plot(left_line(1,:), left_line(2,:), 'r')
plot(middle_line(1,:), middle_line(2,:), 'b--')
plot(centerline1(1,:), centerline1(2,:), 'm--')
xlabel('X')
ylabel('Y')
