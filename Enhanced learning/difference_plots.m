clc
clearvars
close all

load('workspace.mat');
font_ax = 14;
font = 16;
t_max = 750.0;
x_ref = data_list{1, 2}.x_cl;
y_ref = data_list{1, 2}.y_cl;
y_mpc = solutions{1, 28}{1, 2}.y_cl;


figure(1)
plot(x_ref,abs(y_ref-y_mpc),'g','LineWidth',1.0)
hold on

title('Error [m]','fontsize',font,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
set(gca,'fontsize',font_ax,'fontweight','bold')
xlabel('x [m]','fontsize',font,'fontweight','bold')
ylabel('position error Y [m]','fontsize',font,'fontweight','bold')
legend('V22.22-L3.47','V25.00-L6.94')