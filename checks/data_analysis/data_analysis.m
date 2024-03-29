% stijn.staring@siemens.com

clc
clear all
close all

%% Files
end_value = 1001;
% files = {'Dataset_Xchange130.3244_V0=95.4982.csv',;
%          'Dataset_Xchange133.1118_V0=95.0987.csv',;
%          'Dataset_Xchange136.7518_V0=82.6395.csv',;
%          'Dataset_Xchange141.0329_V0=99.4119.csv',;
%          'Dataset_Xchange144.6254_V0=94.1856.csv',;
%          'Dataset_Xchange147.8353_V0=84.4808.csv',;
%          'Dataset_Xchange153.1601_V0=88.4862.csv',;
%          'Dataset_Xchange159.801_V0=92.8889.csv',;
%          'Dataset_Xchange162.2641_V0=95.4032.csv',;
%          'Dataset_Xchange166.5747_V0=99.8741.csv',;
%          'Dataset_Xchange171.6427_V0=85.2442.csv',;
%          'Dataset_Xchange175.9018_V0=80.7142.csv',;
%          'Dataset_Xchange178.9122_V0=83.9562.csv',;
%          'Dataset_Xchange185.2974_V0=86.3705.csv',;
%          'Dataset_Xchange189.4391_V0=98.6799.csv',;
%          'Dataset_Xchange195.0198_V0=94.6066.csv',;
%          'Dataset_Xchange199.9356_V0=83.4224.csv'};
files = {'Dataset_Xchange130.8331_V0=86.7425.csv'};
     
%% Initialization
length = size(files,1);
amount = 1001; % Length of the dataset
time_d = zeros(amount,length);
x_d = zeros(amount,length);
y_d  = zeros(amount,length);
vx_d = zeros(amount,length);
vy_d = zeros(amount,length);
r_d = zeros(amount,length);
yaw_d = zeros(amount,length);
steering_deg_d = zeros(amount,length);
throttle_d = zeros(amount,length);
brake_d = zeros(amount,length);
ax_d = zeros(amount,length);
ay_d = zeros(amount,length);
jerk_d = zeros(amount,length);

%% Load data

for i = 1:1:length     
    data = csvread(char(files(i,:)),1);
    %data = csvread(string(files(i)),1);
    time_d(:,i) = data(:,1);
    x_d(:,i) = data(:,2);
    y_d(:,i) = data(:,3);
    vx_d(:,i) = data(:,4);
    vy_d(:,i) = data(:,5);
    r_d(:,i) = data(:,6);
    yaw_d(:,i) = data(:,7);
    steering_deg_d(:,i) = data(:,8);
    throttle_d(:,i) = data(:,9);
    brake_d(:,i) = data(:,10);
    ax_d(:,i) = data(:,11);
    ay_d(:,i) = data(:,12);
end

ay_loc_tot = ay_d + r_d * vx_d;

data_cl['vx_proj_cl'] = plt.cos(data_cl['yaw_cl'])* data_cl['vx_cl'] - plt.sin(data_cl['yaw_cl'])* data_cl['vy_cl']
data_cl['ax_proj_cl'] = plt.cos(data_cl['yaw_cl'])* ax_loc_tot - plt.sin(data_cl['yaw_cl'])* ax_loc_tot
data_cl['ay_proj_cl'] = plt.sin(data_cl['yaw_cl'])* ay_loc_tot  + plt.cos(data_cl['yaw_cl'])* ay_loc_tot


%% Calculations
% Lateral Jerk
% Finite difference scheme used: central difference scheme
dt = time_d(2,1) - time_d(1,1);
for k = 1:1:length
    for i = 2:1:size(ay_d,1)-1
        jerk_d(i,k) = (ay_d(i+1,k)- ay_d(i-1,k))/(2*dt);
    end
end
for i = 1:1:length
    jerk_d(1,i) = jerk_d(2,i);
    jerk_d(end,i) = jerk_d(end-1,i);
end


%% Plots
% plotting the results

% Path

figure('name', 'Demonstrated paths')
title('Demonstrated path','fontsize',12,'fontweight','bold')
xlabel('x [m]','fontsize',12)
ylabel('y [m]','fontsize',12)
grid on;
Legend=cell(1,length);
for i = 1:1:length
    hold on
    plot(x_d(1:end_value,i),y_d(1:end_value,i),'LineWidth',2.0)
    hold off
    Legend{1,i} = char(files(i,:));
end
 legend(Legend)
 
% Lateral acceleration - this is the local acceleration.
figure('name', 'Lateral acceleration')
title('Lateral acceleration','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
ylabel('a_y','fontsize',12)
grid on;
Legend=cell(1,length);
for i = 1:1:length
    hold on
    plot(time_d(1:end_value,i),ay_d(1:end_value,i),'LineWidth',2.0)
    hold off
    Legend{1,i} = char(files(i,:));
end
 legend(Legend)

% Steerwheel angle
figure('name', 'Steerwheel angle')
title('Steerwheel angle','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
ylabel('Steerwheel angle [degrees]','fontsize',12)
grid on;
Legend=cell(1,length);
for i = 1:1:length
    hold on
    plot(time_d(1:end_value,i),steering_deg_d(1:end_value,i),'LineWidth',2.0)
    hold off
    Legend{1,i} = char(files(i,:));
end
 legend(Legend)

% Yaw angle
figure('name', 'Yaw angle')
title('Yaw angle','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
ylabel('Yaw angle [degrees]','fontsize',12)
grid on;
Legend=cell(1,length);
for i = 1:1:length
    hold on
    plot(time_d(1:end_value,i),180/pi.*yaw_d(1:end_value,i),'LineWidth',2.0)
    hold off
    Legend{1,i} = char(files(i,:));
end
 legend(Legend)
 
 % Lateral Jerk
figure('name', 'Lateral Jerk')
title('Lateral Jerk','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
ylabel('Lateral Jerk [m/s^3]','fontsize',12)
grid on;
Legend=cell(1,length);
for i = 1:1:length
    hold on
    plot(time_d(1:end_value,i),jerk_d(1:end_value,i),'LineWidth',2.0)
    hold off
    Legend{1,i} = char(files(i,:));
end
 legend(Legend)


