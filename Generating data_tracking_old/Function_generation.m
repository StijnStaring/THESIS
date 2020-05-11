function [x_sol_prev,lam_prev] = Function_generation(data,N,update_casadi_function)
import casadi.*
%% Assumptions
% Instantanous torque and steerwheelangle change --> okay (small values)
% Front and rear wheels are powered
% Linear tyre model -> valid if slipangle < 5 degrees and lat_acc < 4 m/s^2
% Parameters of bicycle model based on thesis Kylian
% Suspension is neglected and assumed symmetrical behavior. (bicyle model)

%% Get data
dt = data.time(2,1) - data.time(1,1);
% N_sim = length(data.time) - N;
vx_start = data.vx(1,1);
x_ref = data.x';
y_ref = data.y';
vx_ref = data.vx';
vy_ref = data.vy';
psi_ref = data.psi';
psi_dot_ref = data.psi_dot';
% t_ref = data.time';

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
Tmax = 584; 
% pi = 3.14159265359;

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
throttle_dot = diff(throttle)./dt;
delta_dot = diff(delta)./dt; % for smoothing the delta signal

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
%%
% -----------------------------------------------
%    Objective
% -----------------------------------------------
% Objective: vehicle tries to follow a moving point model: x(t) and y(t)
% opti.minimize((x-ref(1,1:N+1))*transpose((x-ref(1,1:N+1)))+(y-ref(2,1:N+1))*transpose((y-ref(2,1:N+1)))+(psi-ref(5,1:N+1))*transpose((psi-ref(5,1:N+1))));
opti.minimize(10*(x(2:N+1)-ref(1,1:N))*transpose((x(2:N+1)-ref(1,1:N)))+10*(y(2:N+1)-ref(2,1:N))*transpose((y(2:N+1)-ref(2,1:N)))+(vx(2:N+1)-ref(3,1:N))*transpose((vx(2:N+1)-ref(3,1:N)))+(vy(2:N+1)-ref(4,1:N))*transpose((vy(2:N+1)-ref(4,1:N)))+100*(psi(2:N+1)-ref(5,1:N))*transpose((psi(2:N+1)-ref(5,1:N)))+(psi_dot(2:N+1)-ref(6,1:N))*transpose((psi_dot(2:N+1)-ref(6,1:N))) + 5*sumsqr(throttle_dot) + 0.01*sumsqr(delta_dot));

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

% solver
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
%the same KKT conditions. 
% sqp is faster --> required for MPC.

% options.ipopt.print_level = 0;
% opti.solver('ipopt',options)
% ipopt --> this is a constraint relaxation method

%% Initial guesses
% states: x, y, vx, vy, psi, psi_dot
opti.set_initial(x, x_ref(1:N+1));
opti.set_initial(y, y_ref(1:N+1));
opti.set_initial(vx, vx_ref(1:N+1));
opti.set_initial(vy, vy_ref(1:N+1));
opti.set_initial(psi,psi_ref(1:N+1));
opti.set_initial(psi_dot,psi_dot_ref(1:N+1));
tic
sol = opti.solve();
toc

%%
% -----------------------------------------------
%    MPC loop
% -----------------------------------------------
x_sol_prev = sol.value(opti.x);
lam_prev = sol.value(opti.lam_g);
% hot start --> start optimization with solution of lagrange multipliers and optimal states of previous optimization.
inputs = {x0,ref,opti.x,opti.lam_g}; 
outputs = {X,U,opti.x,opti.lam_g};
tracking_lane_change = opti.to_function('tracking_lane_change',inputs,outputs);
if update_casadi_function == 1
    tracking_lane_change.save('tracking_lane_change.casadi');
end
% when loading -> DM.set_precision(15)


end