function [x_sol_prev,lam_prev] = Function_generation(data,N,update_casadi_function)
import casadi.*

%% Assumptions
% Front and rear wheels are powered
% Linear tyre model -> valid if slipangle < 5 degrees and lat_acc < 4 m/s^2
% Parameters of bicycle model based on data Siemens
% Suspension is neglected and assumed symmetrical behavior. (bicyle model)
% Mismatch between angle of front wheel and the steeringwheelangle is
% modelled by Gsteer = 16.96 --> delta modelled here is of the front wheel
% angle.

%% Get data
dt = data.time(2,1) - data.time(1,1);
vx_start = data.vx(1,1);
width_road = data.y(end,1);
x_ref = data.x';
y_ref = data.y';
vx_ref = data.vx';
vy_ref = data.vy';
psi_ref = data.psi';
psi_dot_ref = data.psi_dot';
throttle_ref = data.throttle';
delta_ref = data.delta';
throttle_dot_ref = data.throttle_dot';
delta_dot_ref = data.delta_dot';
ax_ref = data.ax';
ay_ref = data.ay';


%% Parameters of the optimization
disp('Start of Simulation')
disp('-----------------------')
fprintf('The control horizon in points: %i \n',N)
fprintf('The control horizon in seconds: %i \n',N*dt)
fprintf('Time discretization of dataset: %i \n',dt)

nx = 10;
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
pi = 3.14159265359;
Gsteering = 16.96; 

% Equations of the vehicle model
x = SX.sym('x') ;
y = SX.sym('y');
vx = SX.sym('vx'); 
vy = SX.sym('vy') ;
psi = SX.sym('psi'); 
psi_dot = SX.sym('psi_dot'); 
throttle = SX.sym('throttle');
delta = SX.sym('delta')  ; % This is the steeringwheelangle
ax_total = SX.sym('ax_tot');
ay_total = SX.sym('ay_tot');

% Controls
throttle_dot = SX.sym('throttle_dot');
delta_dot = SX.sym('delta_dot');

% Equations from Bicycle model
vx_glob = vx*cos(psi)-vy*sin(psi);
vy_glob = vx*sin(psi)+vy*cos(psi);
slipangle_f = atan2(vy+psi_dot*a,vx) - delta/Gsteering;
slipangle_r = atan2(vy-psi_dot*b,vx);
Fxf = throttle*Tmax/(2*rw);
Fxr = Fxf;
Fyf = -2*Kyf* slipangle_f;
Fyr = -2*Kyr* slipangle_r;
F_d = Cr0 + Cr2*vx*vx;
atx = (cos(delta/Gsteering)*Fxf-sin(delta/Gsteering)*Fyf+Fxr-F_d)/M +vy*psi_dot; 
aty = (sin(delta/Gsteering)*Fxf+cos(delta/Gsteering)*Fyf+Fyr)/M -vx*psi_dot;
psi_ddot = (sin(delta/Gsteering)*Fxf*a+cos(delta/Gsteering)*Fyf*a-b*Fyr)/Izz;

c = Tmax/(2*rw);
jx_total = (c*throttle_dot - 2*Cr2*vx*atx + 2*Kyf*sin(delta/Gsteering)*(((a*psi_ddot + aty)/vx - ((vy + a*psi_dot)*atx)/vx^2)/((vy + a*psi_dot)^2/vx^2 + 1) - delta_dot/Gsteering) + c*cos(delta/Gsteering)*throttle_dot - 2*Kyf*cos(delta/Gsteering)*(delta/Gsteering- atan2((vy + a*psi_dot),vx))*delta_dot/Gsteering - c*sin(delta/Gsteering)*throttle*delta_dot/Gsteering)/M;
jy_total = ((2*Kyr*((b*psi_ddot - aty)/vx + ((vy - b*psi_dot)*atx)/vx^2))/((vy - b*psi_dot)^2/vx^2 + 1) - 2*Kyf*cos(delta/Gsteering)*(((a*psi_ddot + aty)/vx - ((vy + a*psi_dot)*atx)/vx^2)/((vy + a*psi_dot)^2/vx^2 + 1) - delta_dot/Gsteering) + c*sin(delta/Gsteering)*throttle_dot - 2*Kyf*sin(delta/Gsteering)*(delta/Gsteering - atan2((vy + a*psi_dot),vx))*delta_dot/Gsteering + c*cos(delta/Gsteering)*throttle*delta_dot/Gsteering)/M;


% states: x, y, vx, vy, psi, psi_dot
% controls: torque, delta
rhs = [vx_glob; vy_glob; atx; aty; psi_dot; psi_ddot; throttle_dot;delta_dot;jx_total;jy_total];

% Continuous system dynamics as a CasADi Function
states = [x; y; vx; vy; psi; psi_dot; throttle; delta;ax_total;ay_total];
controls = [throttle_dot; delta_dot];
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
throttle = X(7,:);
delta = X(8,:);
ax = X(9,:);
ay = X(10,:);

% Decision variables for control vector
U =  opti.variable(nc,N);
throttle_dot = U(1,:);
delta_dot = U(2,:);

% Parameters
x0 = opti.parameter(nx);
ref = opti.parameter(nx+2,N);
opti.set_value(x0,[0;0;vx_start;0;0;0;0.0249913580246913;0;ax_ref(1,1);ay_ref(1,1)]);

opti.set_value(ref,[x_ref(2:N+1);y_ref(2:N+1);vx_ref(2:N+1);vy_ref(2:N+1);psi_ref(2:N+1);psi_dot_ref(2:N+1);throttle_ref(2:N+1);delta_ref(2:N+1);throttle_dot_ref(1:N);delta_dot_ref(1:N);ax_ref(2:N+1);ay_ref(2:N+1)]);

% Gap-closing shooting constraints
for k=1:N
   opti.subject_to(X(:,k+1) == F(X(:,k),U(:,k)));
end

% Path constraints
v_min = vx_start - 1;
v_max = vx_start + 1;
psi_max = 5*pi/180;
opti.subject_to(-width_road/2 <= y <= width_road*3/2) % SPECIFIC
opti.subject_to(0 <= x ) % vehicle has to drive forward
opti.subject_to(v_min <= vx); %local axis [m/s] SPECIFIC 
opti.subject_to(vx <= v_max); %local axis [m/s] SPECIFIC 
opti.subject_to(-1 <= throttle <= 1); %local axis [m/s^2]
opti.subject_to(-psi_max <= psi <= psi_max);% SPECIFIC

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
% mx = mean(x_ref);
% my = mean(y_ref);
% mpsi = mean(psi_ref);
% mthrottle_dot = mean(throttle_dot_ref);
% mdelta_dot = mean(delta_dot_ref);

% Objective: vehicle tries to follow a moving point model: x(t) and y(t)
% opti.minimize(10*(x(2:N+1)-ref(1,1:N))*transpose((x(2:N+1)-ref(1,1:N)))+10*(y(2:N+1)-ref(2,1:N))*transpose((y(2:N+1)-ref(2,1:N)))+100*(psi(2:N+1)-ref(5,1:N))*transpose((psi(2:N+1)-ref(5,1:N)))+10*(ax(2:N+1)-ref(11,1:N))*transpose((ax(2:N+1)-ref(11,1:N)))+(ay(2:N+1)-ref(12,1:N))*transpose((ay(2:N+1)-ref(12,1:N))));
opti.minimize(10*(x(2:N+1)-ref(1,1:N))*transpose((x(2:N+1)-ref(1,1:N)))+10*(y(2:N+1)-ref(2,1:N))*transpose((y(2:N+1)-ref(2,1:N)))+15*(vx(2:N+1)-ref(3,1:N))*transpose((vx(2:N+1)-ref(3,1:N)))+(vy(2:N+1)-ref(4,1:N))*transpose((vy(2:N+1)-ref(4,1:N)))+100*(psi(2:N+1)-ref(5,1:N))*transpose((psi(2:N+1)-ref(5,1:N)))+(psi_dot(2:N+1)-ref(6,1:N))*transpose((psi_dot(2:N+1)-ref(6,1:N))) + 5*sumsqr(throttle_dot) + 0.01*sumsqr(delta_dot)+0.01*sumsqr(ax));


% solve optimization problem
options = struct;
options.print_time = false;
options.expand = true; % expand makes function evaluations faster but requires more memory: MX --> SX
options.ipopt.print_level = 0;
opti.solver('ipopt',options)


%% Initial guesses
% states: x, y, vx, vy, psi, psi_dot
% opti.set_initial(x, x_ref(1:N+1));
% opti.set_initial(y, y_ref(1:N+1));
opti.set_initial(vx, vx_ref(1:N+1));
% opti.set_initial(vy, vy_ref(1:N+1));
% opti.set_initial(psi,psi_ref(1:N+1));
% opti.set_initial(psi_dot,psi_dot_ref(1:N+1));

tic
sol = opti.solve();
toc

%%
% -----------------------------------------------
%    MPC loop
% -----------------------------------------------
x_sol_prev = sol.value(opti.x);
opti.set_initial(opti.x,x_sol_prev);
lam_prev = sol.value(opti.lam_g);
opti.set_initial(opti.lam_g,lam_prev);

if update_casadi_function == 1
    inputs = {x0,ref,opti.x,opti.lam_g}; 
    outputs = {X,U,opti.x,opti.lam_g};
    tracking_lane_change = opti.to_function('tracking_lane_change',inputs,outputs);
    tracking_lane_change.save('tracking_lane_change.casadi');
end
% when loading -> DM.set_precision(15)

end