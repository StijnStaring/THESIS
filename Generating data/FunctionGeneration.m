%% CasADi Function generation of Optimal Planning Module
% this code is used for the planning

% addpath('C:\ProgramsX100\Matlab\casadi-windows-matlabR2016a-v3.5.1')
import casadi.*

N = 50;
Ts = 0.04;

%% Model Equations 

% Declare model variables (Point-mass model on tangent reference frame, neglected beta)

x1 = SX.sym('x1');          % X
x2 = SX.sym('x2');          % Y
x3 = SX.sym('x3');          % V
x4 = SX.sym('x4');          % Yaw
state = [x1; x2; x3; x4];   % states
u1 = SX.sym('u1');          % steering rate
u2 = SX.sym('u2');          % throttle
input = [u1; u2];           % control inputs

% Model equations
xdot = [x3*cos(x4);...      % State equation in X
        x3*sin(x4);...      % State equation in Y
        u2; ...             % State equation in V
        u1];                % State equation in Yaw

% Continous time dynamics
f = casadi.Function('f', {state, input}, {xdot},{'x','u'},{'Eq'});            

% Discrete time dynamics, fixed step Runge-Kutta integrator
M = 4;                          % Number of iterations
DT = Ts/M;                      % Discretization step
X0 = SX.sym('X0', 4);           % Initial state
U = SX.sym('U', 2);             % Input
X = X0;                         % Initial step in integration
for j=1:M
    k1 = f(X, U);
    k2 = f(X + DT/2 * k1, U);
    k3 = f(X + DT/2 * k2, U);
    k4 = f(X + DT * k3, U);
    X = X + DT/6*(k1 + 2*k2 + 2*k3 + k4);
end

% Discrete time dynamics funtion
F = Function('F', {X0, U}, {X}, {'x0','p'}, {'xf'});

%% Road Potential Field Function (Morse Function)

pos_x = SX.sym('posx');                     % Position in space, X coord
pos_y = SX.sym('posy');                     % Position in space, Y coord
coeff = SX.sym('coeff', 6);                 % Polynomial coefficients for road shape fit
sig = SX.sym('sig');                        % Sign determining which side of the road we are on and height of minimum
depth = SX.sym('depth');                    % Depth of well in PF function
m = -1/(2*coeff(1)*pos_x+coeff(3)+1e-5);        
Y = sum(coeff'*[pos_x^5;pos_x^4; pos_x^3;pos_x^2;pos_x;1]);
b = Y - m*pos_x;
pf = 5*(depth - exp(-sig*sign(pos_y - Y+1e-5)*sqrt(((pos_y - b)/m - pos_x)^2 + (Y - pos_y+1e-5)^2)))^2;
Pot = casadi.Function('Pot', {pos_x, pos_y, coeff, sig, depth}, {pf}, {'x', 'y', 'coeff', 'sign', 'depth'}, {'val'});
            
%% MPC Problem Definition

opti = casadi.Opti();               % Creating the opti structure

% Decision variables
x = opti.variable(4,N);             % State matrix across the whole horizon
u = opti.variable(2,N);             % Input matrix over whole horizon 

% Parameters changing between different problem evaluations
states = opti.parameter(4,1);       % Current car states
ref_v = opti.parameter(1,N);        % Reference velocity
rl = opti.parameter(2,N);           % Right line parameters
ml = opti.parameter(2,N);           % Middle line parameteers
ll = opti.parameter(2,N);           % Left line parameters
cen1 = opti.parameter(3,N);         % Left line parameters
% Road potential field parameters
coeffr = opti.parameter(6,1);           % Polynomial coefficients for right centerline fitting
coeffl = opti.parameter(6,1);           % Polynomial coefficients for left centerline fitting

% Weights
Msr = 150;                          % Maximum steering rate 
Mdec = -2;                          % Maximum deceleration
Macc = 1;                           % Maximum acceleration
Wv = 100;                           % Weight for velocity error
Wt = 500;                           % Weight for throttle
Ws = 200;                           % Weight for yaw
WF = 100;                          % Weight for road PF

J = 0;
opti.subject_to(x(:,1) - states == 0);  % Set initial states equal to current car states

for i = 1:N-1
    
     % Cost for error in velocity (computed according to road curvature and driver reference speed)
    J = J + Wv*(x(3,i)-ref_v(i))^2; 
    % Costs for inputs amplitudes
    J = J + Ws*u(1,i)^2 + Wt*u(2,i)^2;      

    % Right lane potential field
    Field_right = Pot('x', x(1,i), 'y', x(2,i), 'coeff', coeffr, 'sign', 1.5, 'depth', 1);
    % Left lane potential field
    Field_left = Pot('x', x(1,i), 'y', x(2,i), 'coeff', coeffl, 'sign', -1.5, 'depth', -1);
    % Cost for road potential field
    J = J + WF*(Field_right.val + Field_left.val);   
    
    % NON-PARAMETRIC CONSTRAINTS
    opti.subject_to( -Msr*pi/180 <= u(1,i) <= Msr*pi/180 );      % Constraint on Steering Rate
    opti.subject_to( -2 <= u(2,i) <= 1);                         % Constraint on Acceleration
    
    % Lane boundary constraints: Hyperplanes (PARAMETRIC CONSTRAINT)
    v1 = [rl(1,i+1)-rl(1,i);rl(2,i+1)-rl(2,i)];     % Vector of right line heading
    v2 = [x(1,i)-rl(1,i);x(2,i)-rl(2,i)];           % Vector of car position relative to right line
    v3 = [ll(1,i+1)-ll(1,i);ll(2,i+1)-ll(2,i)];     % Vector of left line heading
    v4 = [x(1,i)-ll(1,i);x(2,i)-ll(2,i)];           % Vector of car position relative to left line
    v1n = sqrt(v1(1)^2 + v1(2)^2);                  % Right line heading vector length
    v3n = sqrt(v3(1)^2 + v3(2)^2);                  % Left line heading vector length
    opti.subject_to( 0.5 <= (v1(1)*v2(2)-v1(2)*v2(1))/(v1n) );        % Constraint for staying on the left of right line
    opti.subject_to( (v3(1)*v4(2)-v3(2)*v4(1))/(v3n) <= -0.5 );       % Constraint for staying on the right of left line
    
    % Dynamic constraint
    x_new = F('x0', x(:,i), 'p', u(:,i));   
    opti.subject_to( x(:,i+1) - x_new.xf == 0 );                 
    
end

opti.minimize(J);

%% Code Generation

% Choose the NLP solver
opts = struct;
opts.ipopt.print_level = 0;
opts.print_time = false;
opti.solver('ipopt',opts);

% CasADi function object: map from initial states to first computed control
% input (receding horizon)

% Planner = opti.to_function('M',{states, ref_v, rl, ml, ll, cen1, coeffr, coeffl, x},{x},...
%                           {'initial_X', 'ref_v', 'rl', 'ml', 'll', 'cen', 'coeffr', 'coeffl', 'initGuess_X'},{'XSol'});
% 
% 
% save('Planner','Planner')