close all
clc
clearvars
import casadi.*
%% Assumptions
% Instantanous torque and steerwheelangle change --> okay (small values)
% Front and rear wheels are powered
% Linear tyre model -> valid if slipangle < 5 degrees and lat_acc < 4 m/s^2
% Parameters of bicycle model based on thesis Kylian
% Suspension is neglected and assumed symmetrical behavior. (bicyle model)

%% Get data
N = 50; % Control horizon of one optimization of the MPC.
sampling_rate = 100;
files = {'DATA2_V22.22_L3.47.csv'};
amount_files = max(size(files));
total_data = cell(1,amount_files);
data = get_data(char(files(:,1)),sampling_rate,N);
dt = 1/sampling_rate;
amount_simulations = length(data.time) - 1;
vx_start = data.vx(1,1);
x_ref = data.x(1:N+1)';
y_ref = data.y(1:N+1)';
vx_ref = data.vx(1:N+1)';
vy_ref = data.vy(1:N+1)';
psi_ref = data.psi(1:N+1)';
psi_dot_ref = data.psi_dot(1:N+1)';
t_ref = data.time(1:N+1)';

%% Parameters of the optimization
disp('Start of Simulation')
disp('-----------------------')
fprintf('The control horizon in points: %i \n',N)
fprintf('The control horizon in seconds: %i \n',N*dt)
% System is composed of 6 states and 2 controls
nx = 6;
nc = 2;

%%
% ----------------------------------
%    continuous system dot(x)=f(x,u)
% ----------------------------------
% Construct a CasADi function for the ODE right-hand side
% Remark !x and y are coordinates in local axis!
M  = 1430; Izz = 1300;
a = 1.056; 
b = 1.344; 
Kyf= 41850.8527587; Kyr = 51175.775017;
Cr0  = 0.6; Cr2 = 0.1; rw = 0.292;
Tmax = 584; pi = 3.14159265359;

%States
x = MX.sym('x'); % in global axis
y = MX.sym('y'); % in global axis
vx = MX.sym('vx'); % in local axis
vy = MX.sym('vy'); % in local axis
psi = MX.sym('psi'); %yaw angle
psi_dot = MX.sym('psi_dot'); %rate of yaw angle

%Controls
throttle = MX.sym('throttle');
delta = MX.sym('delta');

% Equations from Bicycle model
x_glob = vx*cos(psi)-vy*sin(psi);
y_glob = vx*sin(psi)+vy*cos(psi);
slipangle_f = atan2(vy+psi_dot*a,vx) - delta;
slipangle_r = atan2(vy-psi_dot*b,vx);
Fxf = throttle*Tmax/(2*rw);
Fxr = Fxf;
Fyf = -2*Kyf* slipangle_f;
Fyr = -2*Kyr* slipangle_r;
F_d = Cr0 + Cr2*vx*vx;
ddx = (cos(delta)*Fxf-sin(delta)*Fyf+Fxr-F_d)/M +vy*psi_dot; 
ddy = (sin(delta)*Fxf+cos(delta)*Fyf+Fyr)/M -vx*psi_dot;
ddpsi = (sin(delta)*Fxf*a+cos(delta)*Fyf*a-b*Fyr)/Izz;

% states: x, y, vx, vy, psi, psi_dot
% controls: torque, delta
rhs = [x_glob; y_glob; ddx; ddy; psi_dot; ddpsi];

% Continuous system dynamics as a CasADi Function
states = [x; y; vx; vy; psi; psi_dot];
controls = [throttle; delta];
f = Function('f', {states,controls}, {rhs},{'states','controls'},{'rhs'});

%%
% -----------------------------------
%    Discrete system x_next = F(x,u)
% -----------------------------------
% Integration is needed to know the next state of the vehicle.

% Integrator options
intg_options = struct;
intg_options.simplify = true; % Enhances integration
intg_options.tf = dt; % Integration step

% Runge-Kutta implementation
intg = integrator('intg','rk',struct('x',states,'p',controls,'ode',f(states,controls)),intg_options);
res = intg('x0',states,'p',controls);
states_next = res.xf; % States after integration

% Discretized (sampling time dt) system dynamics as a CasADi Function
F = Function('F', {states, controls}, {states_next},{'states','controls'},{'states_next'});

%%
% -----------------------------------------------
%    Optimal control problem, multiple shooting
% -----------------------------------------------

opti = casadi.Opti();
% Decision variables for states
X = opti.variable(nx,N+1);

% Aliases for states
x    = X(1,:);
y  = X(2,:);
vx = X(3,:);
vy = X(4,:);
psi = X(5,:);
psi_dot = X(6,:);

% Decision variables for control vector
U =  opti.variable(nc,N);
throttle = U(1,:);
delta = U(2,:);

% Parameters
x0 = opti.parameter(nx);
ref = opti.parameter(nx,N+1);
opti.set_value(x0,[0;0;vx_start;0;0;0]);
opti.set_value(ref,[x_ref(1:N+1);y_ref(1:N+1);vx_ref(1:N+1);vy_ref(1:N+1);psi_ref(1:N+1);psi_dot_ref(1:N+1)]);

% Gap-closing shooting constraints
for k=1:N
   opti.subject_to(X(:,k+1) == F(X(:,k),U(:,k)));
end

% Path constraints
% opti.subject_to(1  <= vx <= 33); %local axis [m/s]
opti.subject_to(-1  <= throttle <= 1); %local axis [m/s^2]
% opti.subject_to(-0.011  <= delta <= 0.011); % Limit on steeringwheelangle (150°)
% opti.subject_to(-0.070 <= psi   <= 0.070);% Limit yaw for doing lane change
% opti.subject_to(-0.061 <= psi_dot   <= 0.061);% Limit on change of yaw (30°/s)

% Initial and terminal constraints
opti.subject_to(X(:,1)==x0);
% opti.subject_to((X(1,N+1) - ref(1,N+1))^2<=0.1);
% opti.subject_to((X(2,N+1) - ref(2,N+1))^2<= 0.1);
% opti.subject_to((X(3,N+1) - ref(3,N+1))^2<=1);
% opti.subject_to((X(4,N+1) - ref(4,N+1))^2<=1);
% opti.subject_to((X(5,N+1) - ref(5,N+1))^2<=1);
% opti.subject_to((X(6,N+1) - ref(6,N+1))^2<= 1);

% DIT MOET NOG AANPASSEN ZIE OPMERKINGEN NAETS!

% Objective: vehicle tries to follow a moving point model: x(t) and y(t)
% opti.minimize((x-ref(1,1:N+1))*transpose((x-ref(1,1:N+1)))+(y-ref(2,1:N+1))*transpose((y-ref(2,1:N+1)))+(psi-ref(5,1:N+1))*transpose((psi-ref(5,1:N+1))));
opti.minimize((x-ref(1,1:N+1))*transpose((x-ref(1,1:N+1)))+(y-ref(2,1:N+1))*transpose((y-ref(2,1:N+1))));
disp('opti.minimize((x-ref(1,1:N+1))*transpose((x-ref(1,1:N+1)))+(y-ref(2,1:N+1))*transpose((y-ref(2,1:N+1))));')

% opti.minimize(sumsqr((x-ref(1,1:N+1)))+sumsqr((y-ref(2,1:N+1)))+ sumsqr(U(1,:))+sumsqr(U(2,:)));
% opti.minimize((x-x_ref_obj)*transpose((x-x_ref_obj))+(y-y_ref_obj)*transpose((y-y_ref_obj)));
% opti.minimize(sumsqr((x-ref(1,1:N+1)))+sumsqr((y-ref(2,1:N+1)))+ sumsqr((vy-ref(4,1:N+1))) + sumsqr(U(1,:))+sumsqr(U(2,:)));



% opti.minimize(sumsqr((x-ref(1,1:N+1))))
% opti.minimize(sumsqr((y-ref(2,1:N+1))))
% % % opti.minimize(sumsqr((psi-ref(5,1:N+1))))
% % % opti.minimize(sumsqr((psi_dot-ref(6,1:N+1))))
% opti.minimize(sumsqr(U(1,:))+sumsqr(U(2,:)))


% solve optimization problem
options = struct;
options.print_time = false;
options.expand = true; % expand makes function evaluations faster but requires more memory: MX --> SX

% Different solver
options.qpsol = 'qrqp';
options.qpsol_options.print_iter = false;
options.qpsol_options.print_header = false;
options.print_iteration = false;
options.print_header = false;
options.print_status = false;
opti.solver('sqpmethod',options)
%sqp --> this approximates the lagrangian of the problam by a quadratic problem with
%the same KKT conditions. :) 
% sqp is faster --> required for MPC.

% options.ipopt.print_level = 0;
% opti.solver('ipopt',options)
% ipopt --> this is a constraint relaxation method

%% Initial guesses
% states: x, y, vx, vy, psi, psi_dot, theta_r, theta_r_dot, theta_f,theta_f_dot,delta
opti.set_initial(x, x_ref(1:N+1));
opti.set_initial(y, y_ref(1:N+1));
opti.set_initial(vx, vx_ref(1:N+1));
opti.set_initial(vy, vy_ref(1:N+1));
opti.set_initial(psi,psi_ref(1:N+1));
opti.set_initial(psi_dot,psi_dot_ref(1:N+1));
tic
sol = opti.solve();
toc
%% plotting
% These are graphs when the optimization is solved for the first time.

% Retrieving solution
x_sol = sol.value(X(1,:));
y_sol = sol.value(X(2,:));
vx_sol = sol.value(X(3,:));
vy_sol = sol.value(X(4,:));
psi_sol = sol.value(X(5,:));
psi_dot_sol = sol.value(X(6,:));
throttle_sol = sol.value(U(1,:));
delta_sol = sol.value(U(2,:));
t_sol = 0:dt:N*dt;

% Axis transformation
vx_global = vx_sol.*cos(psi_sol)-vy_sol.*sin(psi_sol);
vy_global= vx_sol.*sin(psi_sol)+vy_sol.*cos(psi_sol);

% Path (x,y)
figure('name', 'Path')
plot(x_sol,y_sol,'LineWidth',2.0)
hold on
plot(x_ref,y_ref,'LineWidth',1.0)
title('Path','fontsize',12,'fontweight','bold')
xlabel('X [m]','fontsize',12)
ylabel('Y [m]','fontsize',12)
legend('calc\_path','ref\_path')



 % Postion vs time
figure('name', 'pos')
subplot(1,2,1)
plot(t_sol,x_sol,'LineWidth',2.0)
hold on
plot(t_ref,x_ref,'LineWidth',1.0)
title('X [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
ylabel('Position X [m]','fontsize',12)
legend('calc\_pathx','ref\_pathx')


subplot(1,2,2)
plot(t_sol,y_sol,'LineWidth',2.0)
hold on
plot(t_ref,y_ref,'LineWidth',1.0)
title('Y [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
ylabel('Position Y [m]','fontsize',12)
legend('calc\_pathy','ref\_pathy')


 % Velocity vs time
figure('name', 'vel')
subplot(1,2,1)
plot(t_ref,vx_ref,'LineWidth',1.0)
hold on
plot(t_sol,vx_sol,'LineWidth',1.0)
title('vx [m/s] local axis','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
ylabel('Velocity x [m/s]','fontsize',12)
legend('calc\_vx','ref\_vx')
axis([0 4 20 30])

subplot(1,2,2)
plot(t_ref,vy_ref,'LineWidth',1.0)
hold on
plot(t_sol,vy_sol,'LineWidth',1.0)
title('vy [m/s] local axis','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
ylabel('Velocity y [m/s]','fontsize',12)
legend('calc\_vy','ref\_vy')


 % Yaw and Yaw rate 
figure('name', 'yaws')
subplot(1,2,1)
plot(t_sol,psi_sol*180/pi,'LineWidth',2.0)
hold on
plot(t_ref,psi_ref*180/pi,'LineWidth',1.0)
title('yaw [°]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
ylabel('yaw [°]','fontsize',12)
legend('calc\_psi','ref\_psi')


subplot(1,2,2)
plot(t_sol,psi_dot_sol*180/pi,'LineWidth',2.0)
hold on
plot(t_ref,psi_dot_ref*180/pi,'LineWidth',1.0)
title('yaw\_rate [°/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
ylabel('yaw/s [°/s]','fontsize',12)
legend('calc\_psi_dot','ref\_psi_dot')

 % deltas
figure('name', 'delta')
% subplot(1,2,1)
plot(t_sol(1:end-1),delta_sol*180/pi,'LineWidth',2.0)
title('Delta [°]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
ylabel('delta [°]','fontsize',12)

 % throttle
figure('name', 'throttle')
plot(t_sol(1:end-1),throttle_sol,'LineWidth',2.0)
title('throttle [-]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
ylabel('throttle [-]','fontsize',12)

% Calculate the total error made 
x_error = sum(abs(x_sol-x_ref));
y_error = sum(abs(y_sol-y_ref));
vx_error = sum(abs(vx_sol-vx_ref));
vy_error = sum(abs(vy_sol-vy_ref));
psi_error = sum(abs(psi_sol-psi_ref));
psi_dot_error = sum(abs(psi_dot_sol-psi_dot_ref));
disp('**************************************************')
disp('This are the errors: ')
disp('------------------------')
fprintf('This is the total x error: %i \n',x_error)
fprintf('This is the total y error: %i \n',y_error)
fprintf('This is the total vx error: %i \n',vx_error)
fprintf('This is the total vy error: %i \n',vy_error)
fprintf('This is the total psi error: %i \n',psi_error)
fprintf('This is the total psi dot error: %i \n',psi_dot_error)



%%
% -----------------------------------------------
%    MPC loop
% -----------------------------------------------

current_x = [0;0;25.91;0;0;0];
current_ref = zeros(nx,N+1);

x_history = zeros(nx,N_sim+1);
u_history = zeros(nc,N_sim);
x_history(:,1) = current_x;

rand ("state", 0) % state is the seed of the random number.
disp('MPC running')
% hot start --> start optimization with solution of lagrange multipliers and optimal states of previous optimization.
inputs = {x0,ref,opti.x,opti.lam_g}; 
outputs = {U(:,1),opti.x,opti.lam_g};
mpc_step = opti.to_function('mpc_step',inputs,outputs);

u = sol.value(U(:,1));
X_sol = sol.value(opti.x);
lam = sol.value(opti.lam_g);

for i=1:N_sim
    tic
    u_history(:,i) = full(u);

  % Simulate the system over dt
    current_x = full(F(current_x,u));
    current_ref(1,1:N+1) = x_ref(i+1:(N+1)+i);
    current_ref(2,1:N+1) = y_ref(i+1:(N+1)+i);
    current_ref(3,1:N+1) = vx_ref(i+1:(N+1)+i);
    current_ref(4,1:N+1) = vy_ref(i+1:(N+1)+i);
    current_ref(5,1:N+1) = psi_ref(i+1:(N+1)+i);
    current_ref(6,1:N+1) = psi_dot_ref(i+1:(N+1)+i);
    
%     % Adding model mismatch
%     SNR = 40; % Signal to noise ratio
%     current_x = 0*current_x + 1*[awgn(current_x(1,1), SNR);awgn(current_x(2,1), SNR);awgn(current_x(3,1), SNR);awgn(current_x(4,1), SNR);awgn(current_x(5,1), SNR);awgn(current_x(6,1), SNR);];
%   
  [u,X_sol,lam] = mpc_step(current_x,current_ref,X_sol,lam);

  x_history(:,i+1) = current_x;
  toc
  
end
  
%% Plotting (report)

t_max = 4;
x_max = 103;
N_diff = 50;

% Path tracking
% Path (x,y)
figure('name', 'Path')
plot(x_ref,y_ref,'r.','LineWidth',1.0)
hold on
plot(x_sol,y_sol,'b--','LineWidth',1.0)
title('Vehicle path','fontsize',12,'fontweight','bold')
xlabel('X [m]','fontsize',12)
xlim([0, x_max])
ylabel('Y [m]','fontsize',12)
legend('Reference path','Calculated path','Location','southeast')


% Postion vs time
figure('name', 'Pos')
subplot(2,2,1)
plot(t_ref,x_ref,'r.','LineWidth',1.0)
hold on
plot(t_sol,x_sol,'b--','LineWidth',1.0)
title('X [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position X [m]','fontsize',12)
legend('Reference x path','Calculated x path','Location','southeast')
subplot(2,2,3)
plot(t_sol,abs(x_ref(1:end-N_diff)-x_sol),'b','LineWidth',1.0)
title('Error [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position error X [m]','fontsize',12)
subplot(2,2,2)
plot(t_ref,y_ref,'r.','LineWidth',1.0)
hold on
plot(t_sol,y_sol,'b--','LineWidth',1.0)
title('Y [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position Y [m]','fontsize',12)
legend('Reference y path','Calculated y path','Location','southeast')
subplot(2,2,4)
plot(t_sol,abs(y_ref(1:end-N_diff)-y_sol),'b','LineWidth',1.0)
title('Error [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position error Y [m]','fontsize',12)

% Velocity vs time
figure('name', 'Vel')
subplot(2,2,1)
plot(t_ref,vx_ref,'r.','LineWidth',1.0)
hold on
plot(t_sol,vx_global,'b--','LineWidth',1.0)
title('v_X [m/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('velocity X [m/s]','fontsize',12)
legend('Reference X velocity','Calculated X velocity','Location','southeast')
subplot(2,2,3)
plot(t_sol,abs(vx_ref(1:end-N_diff)-vx_global),'b','LineWidth',1.0)
title('Error [m/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('velocity error X [m/s]','fontsize',12)
subplot(2,2,2)
plot(t_ref,vy_ref,'r.','LineWidth',1.0)
hold on
plot(t_sol,vy_global,'b--','LineWidth',1.0)
title('v_Y [m/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('velocity Y [m/s]','fontsize',12)
legend('Reference Y velocity','Calculated Y velocity','Location','southeast')
subplot(2,2,4)
plot(t_sol,abs(vy_ref(1:end-N_diff)-vy_global),'b','LineWidth',1.0)
title('Error [m/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('velocity error Y [m/s]','fontsize',12)

% Acceleration vs time
vx_ref_local = vx_ref.*cos(psi_ref)+vy_ref.*sin(psi_ref);
vy_ref_local= -vx_ref.*sin(psi_ref)+vy_ref.*cos(psi_ref);

figure('name', 'Acc')
subplot(2,2,1)
plot(t_ref(1:end-1),diff(vx_ref)./diff(t_ref),'r.','LineWidth',1.0)
hold on
plot(t_sol(1:end-1),diff(vx_global)./diff(t_sol),'b--','LineWidth',1.0)
title('a_X [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration X [m/s²]','fontsize',12)
legend('Reference X acceleration','Calculated X acceleration','Location','southeast')
subplot(2,2,3)
plot(t_sol(1:end-1),abs(diff(vx_ref_local(1:end-N_diff))./diff(t_ref(1:end-N_diff))-diff(vx_global)./diff(t_sol)),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration error X [m/s²]','fontsize',12)
subplot(2,2,2)
plot(t_ref(1:end-1),diff(vy_ref)./diff(t_ref),'r.','LineWidth',1.0)
hold on
plot(t_sol(1:end-1),diff(vy_global)./diff(t_sol),'b--','LineWidth',1.0)
title('a_Y [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration Y [m/s²]','fontsize',12)
legend('Reference Y acceleration','Calculated Y acceleration','Location','southeast')
subplot(2,2,4)
plot(t_sol(1:end-1),abs(diff(vy_ref(1:end-N_diff))./diff(t_ref(1:end-N_diff))-diff(vy_global)./diff(t_sol)),'b','LineWidth',1.0)
title('Error [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration error Y [m/s²]','fontsize',12)


% Yaw and Yaw rate 
figure('name', 'Yaw')
subplot(2,2,1)
plot(t_ref,psi_ref*180/pi,'r.','LineWidth',1.0)
hold on
plot(t_sol,psi_sol*180/pi,'b--','LineWidth',1.0)
title('\psi [deg]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('yaw angle \psi [deg]','fontsize',12)
legend('Reference yaw angle','Calculated yaw angle','Location','northeast')
subplot(2,2,3)
plot(t_sol,abs(psi_ref(1:end-N_diff)-psi_sol)*180/pi,'b','LineWidth',1.0)
title('Error [deg]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('yaw angle error \psi [deg]','fontsize',12)
subplot(2,2,2)
plot(t_ref,psi_dot_ref*180/pi,'r.','LineWidth',1.0)
hold on
plot(t_sol,psi_dot_sol*180/pi,'b--','LineWidth',1.0)
title('d\psi [deg/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('yaw velocity d\psi [deg/s]','fontsize',12)
legend('Reference yaw velocity','Calculated yaw velocity','Location','northeast')
subplot(2,2,4)
plot(t_sol,abs(psi_dot_ref(1:end-N_diff)-psi_dot_sol)*180/pi,'b','LineWidth',1.0)
title('Error [deg/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('yaw velocity error d\psi [deg/s]','fontsize',12)

% Inputs
% delta & throttle
figure('name', 'inputs')
subplot(2,1,1)
plot(t_sol(1:end-1),delta_sol*180/pi,'b','LineWidth',1.0)
title('\delta [deg]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('\delta [deg]','fontsize',12)
subplot(2,1,2)
plot(t_sol(1:end-1),throttle_sol,'b','LineWidth',1.0)
title('Throttle coefficient t_r [-]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('throttle coefficient t_r [-]','fontsize',12)

% Slipangles
slipangle_f = atan2(vy_sol(1:end-1)+psi_dot_sol(1:end-1)*a,vx_sol(1:end-1)) - delta_sol;
slipangle_r = atan2(vy_sol(1:end-1)-psi_dot_sol(1:end-1)*b,vx_sol(1:end-1));
figure('name', 'slipangles')
plot(t_sol(1:end-1),slipangle_f*180/pi,'b.','LineWidth',1.0)
hold on
plot(t_sol(1:end-1),slipangle_r*180/pi,'b--','LineWidth',1.0)
title('Slipangle [deg]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('\alpha [deg]','fontsize',12)
legend('\alpha_f','\alpha_r')

% Robustness check
res_noerror = load('MPC_results_noerror.mat');
res_error = load('MPC_results_error.mat');

% Path tracking
% Path (x,y)
figure('name', 'Path - Robustness check')
plot(x_ref,y_ref,'r.','LineWidth',1.0)
hold on
plot(res_noerror.x_sol,res_noerror.y_sol,'b--','LineWidth',1.0)
plot(res_error.x_sol,res_error.y_sol,'g--','LineWidth',1.0)
title('Vehicle path','fontsize',12,'fontweight','bold')
xlabel('X [m]','fontsize',12)
xlim([0, x_max])
ylabel('Y [m]','fontsize',12)
legend('Reference path','MPC (ideal model)','MPC (model mismatch)','Location','southeast')

% Postion vs time
figure('name', 'Pos - Robustness check')
subplot(2,2,1)
plot(t_ref,x_ref,'r.','LineWidth',1.0)
hold on
plot(res_noerror.t_sol,res_noerror.x_sol,'b--','LineWidth',1.0)
plot(res_error.t_sol,res_error.x_sol,'g--','LineWidth',1.0)
title('X [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position X [m]','fontsize',12)
legend('Reference x path','MPC (ideal model)','MPC (model mismatch)','Location','southeast')
subplot(2,2,3)
plot(res_noerror.t_sol,abs(x_ref(1:end-N_diff)-res_noerror.x_sol),'b','LineWidth',1.0)
hold on
plot(res_error.t_sol,abs(x_ref(1:end-N_diff)-res_error.x_sol),'g','LineWidth',1.0)
title('Error [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position error X [m]','fontsize',12)
legend('MPC (ideal model)','MPC (model mismatch)','Location','southeast')
subplot(2,2,2)
plot(t_ref,y_ref,'r.','LineWidth',1.0)
hold on
plot(res_noerror.t_sol,res_noerror.y_sol,'b--','LineWidth',1.0)
plot(res_error.t_sol,res_error.y_sol,'g--','LineWidth',1.0)
title('Y [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position Y [m]','fontsize',12)
legend('Reference y path','MPC (ideal model)','MPC (model mismatch)','Location','southeast')
subplot(2,2,4)
plot(res_noerror.t_sol,abs(y_ref(1:end-N_diff)-res_noerror.y_sol),'b','LineWidth',1.0)
hold on
plot(res_error.t_sol,abs(y_ref(1:end-N_diff)-res_error.y_sol),'g','LineWidth',1.0)
title('Error [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position error Y [m]','fontsize',12)
legend('MPC (ideal model)','MPC (model mismatch)','Location','southeast')

% Velocity vs time
figure('name', 'Vel - Robustness check')
subplot(2,2,1)
plot(t_ref,vx_ref,'r.','LineWidth',1.0)
hold on
plot(res_noerror.t_sol,res_noerror.vx_global,'b--','LineWidth',1.0)
plot(res_error.t_sol,res_error.vx_global,'g--','LineWidth',1.0)
title('v_X [m/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('velocity X [m/s]','fontsize',12)
legend('Reference X velocity','MPC (ideal model)','MPC (model mismatch)','Location','southeast')
subplot(2,2,3)
plot(res_noerror.t_sol,abs(vx_ref(1:end-N_diff)-res_noerror.vx_global),'b','LineWidth',1.0)
hold on
plot(res_error.t_sol,abs(vx_ref(1:end-N_diff)-res_error.vx_global),'g','LineWidth',1.0)
title('Error [m/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('velocity error X [m/s]','fontsize',12)
legend('MPC (ideal model)','MPC (model mismatch)','Location','southeast')
subplot(2,2,2)
plot(t_ref,vy_ref,'r.','LineWidth',1.0)
hold on
plot(res_noerror.t_sol,res_noerror.vy_global,'b--','LineWidth',1.0)
plot(res_error.t_sol,res_error.vy_global,'g--','LineWidth',1.0)
title('v_Y [m/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('velocity Y [m/s]','fontsize',12)
legend('Reference Y velocity','MPC (ideal model)','MPC (model mismatch)','Location','southeast')
subplot(2,2,4)
plot(res_noerror.t_sol,abs(vy_ref(1:end-N_diff)-res_noerror.vy_global),'b','LineWidth',1.0)
hold on
plot(res_error.t_sol,abs(vy_ref(1:end-N_diff)-res_error.vy_global),'g','LineWidth',1.0)
title('Error [m/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('velocity error Y [m/s]','fontsize',12)
legend('MPC (ideal model)','MPC (model mismatch)','Location','southeast')

accx_ref = diff(vx_ref)./diff(t_ref);
accx_sol_noerror = diff(res_noerror.vx_global)./diff(res_noerror.t_sol);
accx_sol_error = diff(res_error.vx_global)./diff(res_error.t_sol);
accy_ref = diff(vy_ref)./diff(t_ref);
accy_sol_noerror = diff(res_noerror.vy_global)./diff(res_noerror.t_sol);
accy_sol_error = diff(res_error.vy_global)./diff(res_error.t_sol);

figure('name', 'Acc - Robustness check')
subplot(2,2,1)
plot(t_ref(1:end-1),accx_ref,'r.','LineWidth',1.0)
hold on
plot(res_noerror.t_sol(1:end-1),diff(res_noerror.vx_global)./diff(res_noerror.t_sol),'b--','LineWidth',1.0)
plot(res_error.t_sol(1:end-1),diff(res_error.vx_global)./diff(res_error.t_sol),'g--','LineWidth',1.0)
title('a_X [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration X [m/s²]','fontsize',12)
legend('Reference X acceleration','MPC (ideal model)','MPC (model mismatch)','Location','southeast')
subplot(2,2,3)
plot(res_noerror.t_sol(1:end-1),abs(diff(vx_ref_local(1:end-N_diff))./diff(t_ref(1:end-N_diff))-diff(res_noerror.vx_global)./diff(res_noerror.t_sol)),'b','LineWidth',1.0)
hold on
plot(res_error.t_sol(1:end-1),abs(diff(vx_ref_local(1:end-N_diff))./diff(t_ref(1:end-N_diff))-diff(res_error.vx_global)./diff(res_error.t_sol)),'g','LineWidth',1.0)
title('Error [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration error X [m/s²]','fontsize',12)
legend('MPC (ideal model)','MPC (model mismatch)','Location','southeast')
subplot(2,2,2)
plot(t_ref(1:end-1),diff(vy_ref)./diff(t_ref),'r.','LineWidth',1.0)
hold on
plot(res_noerror.t_sol(1:end-1),diff(res_noerror.vy_global)./diff(res_noerror.t_sol),'b--','LineWidth',1.0)
plot(res_error.t_sol(1:end-1),diff(res_error.vy_global)./diff(res_error.t_sol),'g--','LineWidth',1.0)
title('a_Y [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration Y [m/s²]','fontsize',12)
legend('Reference Y acceleration','MPC (ideal model)','MPC (model mismatch)','Location','southeast')
subplot(2,2,4)
plot(res_noerror.t_sol(1:end-1),abs(diff(vy_ref(1:end-N_diff))./diff(t_ref(1:end-N_diff))-diff(res_noerror.vy_global)./diff(res_noerror.t_sol)),'b','LineWidth',1.0)
hold on
plot(res_error.t_sol(1:end-1),abs(diff(vy_ref(1:end-N_diff))./diff(t_ref(1:end-N_diff))-diff(res_error.vy_global)./diff(res_error.t_sol)),'g','LineWidth',1.0)
title('Error [m/s²]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('acceleration error Y [m/s²]','fontsize',12)
legend('MPC (ideal model)','MPC (model mismatch)','Location','southeast')

% Yaw and Yaw rate 
figure('name', 'Yaw - Robustness check')
subplot(2,2,1)
plot(t_ref,psi_ref*180/pi,'r.','LineWidth',1.0)
hold on
plot(res_noerror.t_sol,res_noerror.psi_sol*180/pi,'b--','LineWidth',1.0)
plot(res_error.t_sol,res_error.psi_sol*180/pi,'g--','LineWidth',1.0)
title('\psi [deg]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('yaw angle \psi [deg]','fontsize',12)
legend('Reference yaw angle','MPC (ideal model)','MPC (model mismatch)','Location','northeast')
subplot(2,2,3)
plot(res_noerror.t_sol,abs(psi_ref(1:end-N_diff)-res_noerror.psi_sol)*180/pi,'b','LineWidth',1.0)
hold on
plot(res_error.t_sol,abs(psi_ref(1:end-N_diff)-res_error.psi_sol)*180/pi,'g','LineWidth',1.0)
title('Error [deg]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('yaw angle error \psi [deg]','fontsize',12)
legend('MPC (ideal model)','MPC (model mismatch)','Location','southeast')
subplot(2,2,2)
plot(t_ref,psi_dot_ref*180/pi,'r.','LineWidth',1.0)
hold on
plot(res_noerror.t_sol,res_noerror.psi_dot_sol*180/pi,'b--','LineWidth',1.0)
plot(res_error.t_sol,res_error.psi_dot_sol*180/pi,'g--','LineWidth',1.0)
title('d\psi [deg/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('yaw velocity d\psi [deg/s]','fontsize',12)
legend('Reference yaw velocity','MPC (ideal model)','MPC (model mismatch)','Location','northeast')
subplot(2,2,4)
plot(res_noerror.t_sol,abs(psi_dot_ref(1:end-N_diff)-res_noerror.psi_dot_sol)*180/pi,'b','LineWidth',1.0)
hold on
plot(res_error.t_sol,abs(psi_dot_ref(1:end-N_diff)-res_error.psi_dot_sol)*180/pi,'g','LineWidth',1.0)
title('Error [deg/s]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('yaw velocity error d\psi [deg/s]','fontsize',12)
legend('MPC (ideal model)','MPC (model mismatch)','Location','southeast')

% Inputs
% delta & throttle
figure('name', 'inputs - Robustness check')
subplot(2,1,1)
plot(res_noerror.t_sol(1:end-1),res_noerror.delta_sol*180/pi,'r.','LineWidth',1.0)
hold on
plot(res_error.t_sol(1:end-1),res_error.delta_sol*180/pi,'b--','LineWidth',1.0)
title('\delta [deg]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('\delta [deg]','fontsize',12)
legend('MPC (ideal model)','MPC (model mismatch)','Location','northeast')
subplot(2,1,2)
plot(res_noerror.t_sol(1:end-1),res_noerror.throttle_sol,'r.','LineWidth',1.0)
hold on
plot(res_error.t_sol(1:end-1),res_error.throttle_sol,'b--','LineWidth',1.0)
title('Throttle coefficient t_r [-]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('throttle coefficient t_r [-]','fontsize',12)
legend('MPC (ideal model)','MPC (model mismatch)','Location','northeast')

% Slipangles
res_noerror.slipangle_f = atan2(res_noerror.vy_sol(1:end-1)+res_noerror.psi_dot_sol(1:end-1)*a,res_noerror.vx_sol(1:end-1)) - res_noerror.delta_sol;
res_noerror.slipangle_r = atan2(res_noerror.vy_sol(1:end-1)-res_noerror.psi_dot_sol(1:end-1)*b,res_noerror.vx_sol(1:end-1));

res_error.slipangle_f = atan2(res_error.vy_sol(1:end-1)+res_error.psi_dot_sol(1:end-1)*a,res_error.vx_sol(1:end-1)) - res_error.delta_sol;
res_error.slipangle_r = atan2(res_error.vy_sol(1:end-1)-res_error.psi_dot_sol(1:end-1)*b,res_error.vx_sol(1:end-1));

figure('name', 'slipangles - Robustness check')
plot(res_noerror.t_sol(1:end-1),res_noerror.slipangle_f*180/pi,'b.','LineWidth',1.0)
hold on
plot(res_noerror.t_sol(1:end-1),res_noerror.slipangle_r*180/pi,'b--','LineWidth',1.0)
plot(res_error.t_sol(1:end-1),res_error.slipangle_f*180/pi,'g.','LineWidth',1.0)
plot(res_error.t_sol(1:end-1),res_error.slipangle_r*180/pi,'g--','LineWidth',1.0)
title('Slipangle [deg]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('\alpha [deg]','fontsize',12)
legend('\alpha_f (ideal model)','\alpha_r (ideal model)','\alpha_f (model mismatch)','\alpha_r (ideal model mismatch)')

% Relative errors
signalx = abs((x_ref(1:end-N_diff)-res_noerror.x_sol)./x_ref(1:end-N_diff));
signalx(isnan(signalx)) = 0;
signalx(isinf(signalx)) = 0;
signaly = abs((y_ref(1:end-N_diff)-res_noerror.y_sol)./y_ref(1:end-N_diff));
signaly(isnan(signaly)) = 0;
signaly(isinf(signaly)) = 0;
error_pos_noerror = mean(sqrt(signalx.^2 + signaly.^2));

signalx = abs((x_ref(1:end-N_diff)-res_error.x_sol)./x_ref(1:end-N_diff));
signalx(isnan(signalx)) = 0;
signalx(isinf(signalx)) = 0;
signaly = abs((y_ref(1:end-N_diff)-res_error.y_sol)./y_ref(1:end-N_diff));
signaly(isnan(signaly)) = 0;
signaly(isinf(signaly)) = 0;
error_pos_error = mean(sqrt(signalx.^2 + signaly.^2));

signalx = abs((vx_ref(1:end-N_diff)-res_noerror.vx_global)./vx_ref(1:end-N_diff));
signalx(isnan(signalx)) = 0;
signalx(isinf(signalx)) = 0;
signaly = abs((vy_ref(1:end-N_diff)-res_noerror.vy_global)./vy_ref(1:end-N_diff));
signaly(isnan(signaly)) = 0;
signaly(isinf(signaly)) = 0;
error_vel_noerror = mean(sqrt(signalx.^2 + signaly.^2));

signalx = abs((vx_ref(1:end-N_diff)-res_error.vx_global)./vx_ref(1:end-N_diff));
signalx(isnan(signalx)) = 0;
signalx(isinf(signalx)) = 0;
signaly = abs((vy_ref(1:end-N_diff)-res_error.vy_global)./vy_ref(1:end-N_diff));
signaly(isnan(signaly)) = 0;
signaly(isinf(signaly)) = 0;
error_vel_error = mean(sqrt(signalx.^2 + signaly.^2));

signalx = abs((accx_ref(1:end-N_diff)-accx_sol_noerror)./accx_ref(1:end-N_diff));
signalx(isnan(signalx)) = 0;
signalx(isinf(signalx)) = 0;
signaly = abs((accy_ref(1:end-N_diff)-accy_sol_noerror)./accy_ref(1:end-N_diff));
signaly(isnan(signaly)) = 0;
signaly(isinf(signaly)) = 0;
error_acc_noerror = mean(sqrt(signalx.^2 + signaly.^2));

signalx = abs((accx_ref(1:end-N_diff)-accx_sol_error)./accx_ref(1:end-N_diff));
signalx(isnan(signalx)) = 0;
signalx(isinf(signalx)) = 0;
signaly = abs((accy_ref(1:end-N_diff)-accy_sol_error)./accy_ref(1:end-N_diff));
signaly(isnan(signaly)) = 0;
signaly(isinf(signaly)) = 0;
error_acc_error = mean(sqrt(signalx.^2 + signaly.^2));

signalx = abs((psi_ref(1:end-N_diff)-res_noerror.psi_sol)./x_ref(1:end-N_diff));
signalx(isnan(signalx)) = 0;
signalx(isinf(signalx)) = 0;
signaly = abs((psi_ref(1:end-N_diff)-res_noerror.psi_sol)./psi_ref(1:end-N_diff));
signaly(isnan(signaly)) = 0;
signaly(isinf(signaly)) = 0;
error_yaw_noerror = mean(sqrt(signalx.^2 + signaly.^2));

signalx = abs((psi_ref(1:end-N_diff)-res_error.psi_sol)./psi_ref(1:end-N_diff));
signalx(isnan(signalx)) = 0;
signalx(isinf(signalx)) = 0;
signaly = abs((psi_ref(1:end-N_diff)-res_error.psi_sol)./psi_ref(1:end-N_diff));
signaly(isnan(signaly)) = 0;
signaly(isinf(signaly)) = 0;
error_yaw_error = mean(sqrt(signalx.^2 + signaly.^2));

disp('Relative errors')
disp('---------------')
disp('   Position:')
disp(['     No error: ', string(error_pos_noerror)])
disp(['     Error: ', string(error_pos_error)])
disp('   Velocity:')
disp(['     No error: ', string(error_vel_noerror)])
disp(['     Error: ', string(error_vel_error)])
disp('   Acceleration:')
disp(['     No error: ', string(error_acc_noerror)])
disp(['     Error: ', string(error_acc_error)])
disp('   Yaw angle:')
disp(['     No error: ', string(error_yaw_noerror)])
disp(['     Error: ', string(error_yaw_error)])