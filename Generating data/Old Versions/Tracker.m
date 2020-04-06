classdef Tracker < matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime

    properties (Access = private)
        u_old = [0;0]       % Stores the previous value of the control law
        N = 40              % Length of the MPC horizon
        
        Func
        
        optimizer           % Opti structure, necessary for the casadi optimisation
        cost                % Cost function of the MPC
        x                   % Car states
        u                   % Car inputs
        ref                 % Reference trajectory
        ref_v               % Velocity reference
        states              % Current car states
        Wx                  % Weight for x position
        Wy                  % Weight for y position
        Wv                  % Weight for speed
        Wh                  % Weight for heading
        Wyr                 % Weight for yaw rate
        Wt                  % Weight for throttle
        Wsr                 % Weight for steering rate
        Wsa                 % Weight for steering acceleration
        WxN                 % Final weights
        WyN
        WhN
        Wref_change         % Weight for change between references
    end
    
    methods (Access = protected)
        
        function num = getNumInputsImpl(~)
            num = 3;
        end
        function num = getNumOutputsImpl(~)
            num = 2;
        end
        function [dt1, dt2] = getOutputDataTypeImpl(~)
        	dt1 = 'double';
            dt2 = 'double';
        end
        function [dt1, dt2, dt3] = getInputDataTypeImpl(~)
        	dt1 = 'double';
            dt2 = 'double';
            dt3 = 'double';
        end
        function [sz1, sz2] = getOutputSizeImpl(~)
        	sz1 = [2,1];
            sz2 = [3,41];
        end
        function [sz1, sz2, sz3] = getInputSizeImpl(~)
        	sz1 = [1,7];
            sz2 = [4,100];
            sz3 = [1,1];
        end
        function [cp1, cp2, cp3] = isInputComplexImpl(~)
        	cp1 = false;
            cp2 = false;
            cp3 = false;
        end
        function [cp1, cp2] = isOutputComplexImpl(~)
        	cp1 = false;
            cp2 = false;
        end
        function [fz1, fz2, fz3] = isInputFixedSizeImpl(~)
        	fz1 = true;
            fz2 = true;
            fz3 = true;
        end
        function [fz1, fz2] = isOutputFixedSizeImpl(~)
        	fz1 = true;
            fz2 = true;
        end
        
        function setupImpl(obj,~,~, ~)
            
            m = 1430;           Iz = 1300;              % Mass inertia
            Lf = 1.056;         Lr = 2.4-1.056;         % Distance from center of mass to front and rear
            Kf = 41850.8527587; Kr = 51175.775017;      % Tire stiffness
            Cr2 = 0.1;                                  % Second order friction parameter
            Cr0 = 0.6;                                  % Zero order friction parameter
            R = 0.292;                                  % Wheel radius
            Tmax = 584;         G = 15.82;              % Maximum torque and steering wheel to steering angle coeff

            % Declare model variables -> Bicycle model on yaw frame
            x1 = casadi.SX.sym('x1');                       % X
            x2 = casadi.SX.sym('x2');                       % Y
            x3 = casadi.SX.sym('x3');                       % Vx
            x4 = casadi.SX.sym('x4');                       % Vy
            x5 = casadi.SX.sym('x5');                       % Yaw
            x6 = casadi.SX.sym('x6');                       % Yaw_rate
            x7 = casadi.SX.sym('x7');                       % Steering angle
            state = [x1; x2; x3; x4; x5; x6; x7];           % States
            u1 = casadi.SX.sym('u1');                       % Steering rate
            u2 = casadi.SX.sym('u2');                       % Throttle
            input = [u1; u2];                               % Control inputs

            % Model equations
            slip_f = -atan2((x6*Lf + x4), (x3)) + x7/ G;   % Slip angle at front wheel
            slip_r =  atan2((x6*Lr - x4), (x3));           % Slip angle at rear wheel

            Fxf = 0.5*u2*Tmax/ R;                          % Tire force x front
            Fxr = Fxf;                                     % Tire force x rear

            Fyf = Kf*slip_f;                            % Tire force y front
            Fyr = Kr*slip_r;                            % Tire force y rear

            Fres = Cr0 + Cr2*x3*x3;                     % air resistance force

            xdot = [ x3*cos(x5) - x4*sin(x5);...                                                % State equation in x
                     x3*sin(x5) + x4*cos(x5);...                                                % State equation in y
                    ( 1/ m )*( Fxf*cos(x7/ G) + Fxr - Fyf*sin(x7/ G) - Fres + m*x6*x4 ); ...    % State equation in vx
                    ( 1/ m )*( Fxf*sin(x7/ G) + Fyf*cos(x7/ G) + Fyr - m*x6*x3 ) ; ...          % State equation in vy
                     x6 ; ...                                                                   % State equation in yaw
                    ( 1/ Iz )*( Lf*( Fyf*cos(x7/ G) + Fxf*sin(x7/ G)) - Lr*Fyr);                % State equation in yaw_rate
                     u1];                                                                       % State equation in steering

            % Continous time dynamics
            f = casadi.Function('f', {state, input}, {xdot},{'x','u'},{'Eq'});

            % Discrete time dynamics, fixed step Runge-Kutta integrator
            M = 4;                                      % Number of iterations
            DT = 0.01/M;                                  % Discretization step
            X0 = casadi.SX.sym('X0', 7);                % Initial state
            U = casadi.SX.sym('U', 2);                  % Input
            X = X0;                                     % Initial step in integration
            for j=1:M
                k1 = f(X, U);
                k2 = f(X + DT/2 * k1, U);
                k3 = f(X + DT/2 * k2, U);
                k4 = f(X + DT * k3, U);
                X = X + DT/6*(k1 + 2*k2 + 2*k3 + k4);
            end
            F = casadi.Function('F', {X0, U}, {X}, {'x0','p'}, {'xf'});         % Discrete time dynamics function
            obj.Func = F;
            
            
            opti = casadi.Opti();                       % Creating the opti structure
            obj.x = opti.variable(7,obj.N);             % State matrix across the whole horizon
            obj.u = opti.variable(2,obj.N);             % Input matrix over whole horizon
            obj.ref = opti.parameter(3,obj.N+1);        % Reference trajectory
            obj.ref_v = opti.parameter(1,obj.N+1);       % Reference velocity
            obj.states = opti.parameter(7,1);           % Current car states
            obj.Wx = opti.parameter();                  % weight for x
            obj.Wy = opti.parameter();                  % weight for y
            obj.Wv = opti.parameter();                  % weight for velocity
            obj.Wh = opti.parameter();                  % heading weight
            obj.Wt = opti.parameter();                  % weight for throttle
            obj.Wsr = opti.parameter();                 % weight for steering rate
            obj.Wsa = opti.parameter();                 % steering acceleration
            obj.Wref_change = opti.parameter();         % weight on changing the reference
            obj.WxN = opti.parameter();                 % weight on final position x
            obj.WyN = opti.parameter();                 % weight on final position y
            obj.WhN = opti.parameter();                 % weight on final heading
            
            
            J = (obj.u_old-obj.u(:,1))'*obj.Wref_change*(obj.u_old-obj.u(:,1));       % Cost of having a big change in input  
            opti.subject_to(obj.x(:,1) - obj.states == 0);

            
            for i = 1:obj.N-1
                J = J + obj.Wx*(obj.x(1,i)-obj.ref(1,i))^2;                                 % Cost for distance to ref in x
                J = J + obj.Wy*(obj.x(2,i)-obj.ref(2,i))^2;                                 % Cost for distance to ref in y
                J = J + obj.Wv*(sqrt(obj.x(3,i)^2+obj.x(4,i)^2)-obj.ref_v(i))^2;        % Cost for error in velocity
                J = J + obj.Wsr*obj.u(1,i)^2 + obj.Wt*obj.u(2,i)^2;                         % Costs on inputs
                J = J + obj.Wh*(obj.x(5,i)-obj.ref(3,i))^2;                                 % Cost on heading error
                J = J + (obj.u(1,i+1)-obj.u(1,i))'*obj.Wsa*(obj.u(1,i+1)-obj.u(1,i));       % Cost on steering acceleration

                opti.subject_to( 5 <= obj.x(3,i) <= 40);                                    % Constraint on speed
                opti.subject_to( obj.x(4,i) <= 1.5);
                opti.subject_to( -300*pi/180 <= obj.x(7,i) <= 300*pi/180 );                 % Contraint on steering angle
                opti.subject_to( -300*pi/180 <= obj.u(1,i) <= 300*pi/180 );                 % Contraint on steering rate
                opti.subject_to( -1 <= obj.u(2,i) <= 1 );                                   % Constraint on acceleration
                x_new = obj.Func('x0', obj.x(:,i), 'p', obj.u(:,i));                        % Calculating new states
                opti.subject_to( obj.x(:,i+1) - x_new.xf == 0 );                            % Set state equal to computed state
            end
            J = J + obj.WxN*(obj.x(1,obj.N)-obj.ref(1,obj.N))^2;                            % Final cost for distance to ref in x
            J = J + obj.WyN*(obj.x(2,obj.N)-obj.ref(2,obj.N))^2;                            % Final cost for distance to ref in y
            J = J + obj.WhN*(obj.x(5,obj.N)-obj.ref(3,obj.N))^2;                            % Final cost for heading error
            J = J + obj.Wv*(sqrt(obj.x(3,i)^2+obj.x(4,i)^2)-obj.ref_v(obj.N))^2;                                    % Final cost for velocity error
            obj.optimizer = opti();
            obj.cost = J;
        end

        function [control, current_ref] = stepImpl(obj, current_states, reference, ref_vel)
            % Calculating the distance to all points in the reference
            % vector
            dist = (reference(1,:)-current_states(1)).^2 + (reference(2,:)-current_states(2)).^2;
            % Finding the index of the closest point on the reference
            [~, Index] = min(dist);
            % Looking ahead on the reference
%             Index = Index + 1;
            if Index > length(reference)-obj.N                  % Prevents errors by limiting the look-ahead
                Index = length(reference) - obj.N-2;            % to the size of the reference vector
            end
            
            current_ref = reference(1:3,Index:Index+obj.N);     % The reference for the MPC is 40 points long and starts
                                                                % 30 points after the index of the closest point
            
            % Defining all the optimisation parameters
            opti = obj.optimizer;
            opti.set_value(obj.ref, current_ref);
            opti.set_value(obj.ref_v,reference(4,Index:Index+obj.N));
            opti.set_value(obj.states, current_states);
            opti.set_value(obj.Wx, 20);
            opti.set_value(obj.Wy, 20);
            opti.set_value(obj.Wv, 10);
            opti.set_value(obj.Wh, 200);
            opti.set_value(obj.Wt, 10);
            opti.set_value(obj.Wsr, 0.05);
            opti.set_value(obj.Wsa, 0.05);
            opti.set_value(obj.Wref_change, 0.05);
            opti.set_value(obj.WxN, 200);
            opti.set_value(obj.WyN, 200);
            opti.set_value(obj.WhN, 3000);
            
            % Providing an initial guess for the optimisation
            opti.set_initial(obj.x(1:3,:), [reference(1:2,Index:Index+obj.N-1);ref_vel*ones(1,obj.N)]);
            
            % Chosing the solver and performing the optimization
            opti.minimize(obj.cost);
            options.ipopt.print_level = 0;
            opti.solver('ipopt',options);
            
            try
                sol = opti.solve();
                % Outputing the control law
                control = sol.value(obj.u(:,1));
                obj.u_old = control;
            catch
                control = opti.debug.value(obj.u(:,1));
                obj.u_old = control;
                return
            end
                        
        end

        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end