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
data = get_data(char(files(:,1)),sampling_rate,N);
dt = 1/sampling_rate;
N_sim = length(data.time) - N;
vx_start = data.vx(1,1);
x_ref = data.x';
y_ref = data.y';
vx_ref = data.vx';
vy_ref = data.vy';
psi_ref = data.psi';
psi_dot_ref = data.psi_dot';
t_ref = data.time';

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
ref = opti.parameter(nx,N);
opti.set_value(x0,[0;0;vx_start;0;0;0]);
opti.set_value(ref,[x_ref(2:N+1);y_ref(2:N+1);vx_ref(2:N+1);vy_ref(2:N+1);psi_ref(2:N+1);psi_dot_ref(2:N+1)]);

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
opti.minimize((x(2:N+1)-ref(1,1:N))*transpose((x(2:N+1)-ref(1,1:N)))+(y(2:N+1)-ref(2,1:N))*transpose((y(2:N+1)-ref(2,1:N)))+(vx(2:N+1)-ref(3,1:N))*transpose((vx(2:N+1)-ref(3,1:N)))+(vy(2:N+1)-ref(4,1:N))*transpose((vy(2:N+1)-ref(4,1:N)))+(psi(2:N+1)-ref(5,1:N))*transpose((psi(2:N+1)-ref(5,1:N)))+(psi_dot(2:N+1)-ref(6,1:N))*transpose((psi_dot(2:N+1)-ref(6,1:N))));
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
% options.ipopt.print_level = 0;
% opti.solver('ipopt',options)

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
%% plotting one time solving
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
disp('Error calculation - first MPC iteration')
disp('--------------------------------------------')
x_error = sum(abs(x_sol-x_ref(1:N+1)));
y_error = sum(abs(y_sol-y_ref(1:N+1)));
vx_error = sum(abs(vx_sol-vx_ref(1:N+1)));
vy_error = sum(abs(vy_sol-vy_ref(1:N+1)));
psi_error = sum(abs(psi_sol-psi_ref(1:N+1)));
psi_dot_error = sum(abs(psi_dot_sol-psi_dot_ref(1:N+1)));
mx_error = mean(abs(x_sol-x_ref(1:N+1)));
my_error = mean(abs(y_sol-y_ref(1:N+1)));
mvx_error = mean(abs(vx_sol-vx_ref(1:N+1)));
mvy_error = mean(abs(vy_sol-vy_ref(1:N+1)));
mpsi_error = mean(abs(psi_sol-psi_ref(1:N+1)));
mpsi_dot_error = mean(abs(psi_dot_sol-psi_dot_ref(1:N+1)));
disp('**************************************************')
disp('This are the errors: ')
disp('------------------------')
disp('total error')
fprintf('This is the total x error: %i \n',x_error)
fprintf('This is the total y error: %i \n',y_error)
fprintf('This is the total vx error: %i \n',vx_error)
fprintf('This is the total vy error: %i \n',vy_error)
fprintf('This is the total psi error: %i \n',psi_error)
fprintf('This is the total psi dot error: %i \n',psi_dot_error)
disp('mean error')
fprintf('This is the total x error: %i \n',mx_error)
fprintf('This is the total y error: %i \n',my_error)
fprintf('This is the total vx error: %i \n',mvx_error)
fprintf('This is the total vy error: %i \n',mvy_error)
fprintf('This is the total psi error: %i \n',mpsi_error)
fprintf('This is the total psi dot error: %i \n',mpsi_dot_error)


%%
% -----------------------------------------------
%    MPC loop
% -----------------------------------------------

current_x = [0;0;vx_start;0;0;0];
current_ref = zeros(nx,N);

x_history = zeros(nx,N_sim+1);
u_history = zeros(nc,N_sim);
x_history(:,1) = current_x;

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
  % Simulate the system over dt
    current_ref(1,1:N) = x_ref(i+1:N+i);
    current_ref(2,1:N) = y_ref(i+1:N+i);
    current_ref(3,1:N) = vx_ref(i+1:N+i);
    current_ref(4,1:N) = vy_ref(i+1:N+i);
    current_ref(5,1:N) = psi_ref(i+1:N+i);
    current_ref(6,1:N) = psi_dot_ref(i+1:N+i);
    [u,X_sol,lam] = mpc_step(current_x,current_ref,X_sol,lam);
    u_history(:,i) = full(u);
    current_x = full(F(current_x,u));
    x_history(:,i+1) = current_x;
    toc
  
end
  
%% Plotting MPC
t_mpc = 0:dt:dt*N_sim;
t_ref_mpc = t_mpc;
t_max = 8.5;
x_max = 160;
x_mpc = x_history(1,:);
y_mpc = x_history(2,:);
vx_mpc = x_history(3,:);
vy_mpc = x_history(4,:);
psi_mpc = x_history(5,:);
psi_dot_mpc = x_history(6,:);
throttle_mpc = u_history(1,:);
delta_mpc = u_history(2,:);

x_ref_mpc = x_ref(1:N_sim+1);
y_ref_mpc = y_ref(1:N_sim+1);
vx_ref_mpc = vx_ref(1:N_sim+1);
vy_ref_mpc = vy_ref(1:N_sim+1);
psi_ref_mpc = psi_ref(1:N_sim+1);
psi_dot_ref_mpc = psi_dot_ref(1:N_sim+1);

close all
% Path tracking
% Path (x,y)
figure('name', 'Path')
plot(x_ref_mpc,y_ref_mpc,'r.','LineWidth',1.0)
hold on
plot(x_mpc,y_mpc,'b--','LineWidth',1.0)
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
plot(t_mpc,x_mpc,'b--','LineWidth',1.0)
title('X [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position X [m]','fontsize',12)
legend('Reference x path','Calculated x path','Location','southeast')
subplot(2,2,3)
plot(t_mpc,abs(x_ref_mpc-x_mpc),'b','LineWidth',1.0)
title('Error [m]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('position error X [m]','fontsize',12)
subplot(2,2,2)
plot(t_ref_mpc,y_ref_mpc,'r.','LineWidth',1.0)
hold on
plot(t_mpc,y_mpc,'b--','LineWidth',1.0)
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
plot(t_mpc,vx_mpc,'b--','LineWidth',1.0)
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
plot(t_mpc,vy_mpc,'b--','LineWidth',1.0)
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

% Need plots of ay_tot, aty, any, ax_tot, atx, any
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aty_mpc = derivative(vy_mpc,dt);
any_mpc = vx_mpc.*psi_mpc;
ay_tot_mpc = aty_mpc + any_mpc;
atx_mpc = derivative(vx_mpc,dt);
anx_mpc = -vy_mpc.*psi_mpc;
ax_tot_mpc = atx_mpc + anx_mpc;

aty_ref_mpc = derivative(vy_ref_mpc,dt);
any_ref_mpc = vx_ref_mpc.*psi_ref_mpc;
ay_tot_ref_mpc = aty_ref_mpc + any_ref_mpc;
atx_ref_mpc = derivative(vx_ref_mpc,dt);
anx_ref_mpc = -vy_ref_mpc.*psi_ref_mpc;
ax_tot_ref_mpc = atx_ref_mpc + anx_ref_mpc;

figure('name', 'Acc\_y')
subplot(2,2,1)
plot(t_ref_mpc,aty_ref_mpc,'r.','LineWidth',1.0)
hold on
plot(t_mpc,aty_mpc,'b--','LineWidth',1.0)
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
plot(t_mpc,any_mpc,'b--','LineWidth',1.0)
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
plot(t_mpc,atx_mpc,'b--','LineWidth',1.0)
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
plot(t_mpc,anx_mpc,'b--','LineWidth',1.0)
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
plot(t_mpc,ay_tot_mpc,'b--','LineWidth',1.0)
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
plot(t_mpc,ax_tot_mpc,'b--','LineWidth',1.0)
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
plot(t_mpc,psi_mpc*180/pi,'b--','LineWidth',1.0)
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
plot(t_mpc,psi_dot_mpc*180/pi,'b--','LineWidth',1.0)
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

% Slipangles
part1 = atan2(vy_mpc+psi_dot_mpc*a,vx_mpc); 
slipangle_f = part1(1:end-1) - delta_mpc;
slipangle_r = atan2(vy_mpc-psi_dot_mpc*b,vx_mpc);
figure('name', 'slipangles')
plot(t_mpc(1:end-1),slipangle_f*180/pi,'b.','LineWidth',1.0)
hold on
plot(t_mpc,slipangle_r*180/pi,'b--','LineWidth',1.0)
title('Slipangle [deg]','fontsize',12,'fontweight','bold')
xlabel('t [s]','fontsize',12)
xlim([0, t_max])
ylabel('\alpha [deg]','fontsize',12)
legend('\alpha_f','\alpha_r')

disp('End of tracking!')
disp('**********************************')