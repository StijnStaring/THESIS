classdef Planning3_constspeed < matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime


    properties (Access = private)
        N = 100                                 % Length of the MPC horizon
        optimizer                               % Opti structure, necessary for casadi optimisation
        cost                                    % Cost function for MPC
        x                                       % Car states
        u                                       % Car inputs
        Ts = 0.01                               % Sampling time
        Func
        ref_v                                   % Velocity reference
        states                                  % Current car states
        cen
        Wv                                      % Weight for velocity error
        Wt                                      % Weight for throttle
        Wsr                                     % Weight for steering
        Wy                                      % Weight for y-coordinate centerline right error
        Wx                                      % Weight for y-coordinate centerline right error
        Wh
    end

    methods (Access = protected)
        
        function num = getNumInputsImpl(~)
            num = 7;
        end
        function num = getNumOutputsImpl(~)
            num = 1;
        end
        function [dt1] = getOutputDataTypeImpl(~)
        	dt1 = 'double';
        end
        function [dt1, dt2, dt3, dt4, dt5, dt6, dt7] = getInputDataTypeImpl(~)
        	dt1 = 'double';
            dt2 = 'double';
            dt3 = 'double';
            dt4 = 'double';
            dt5 = 'double';
            dt6 = 'double';
            dt7 = 'double';
        end
        function [sz1] = getOutputSizeImpl(~)
            sz1 = [3,400];
        end
        function [sz1, sz2, sz3, sz4, sz5, sz6, sz7] = getInputSizeImpl(~)
        	sz1 = [1,7];
            sz2 = [1,8000];
            sz3 = [3,10000];
            sz4 = [3,10000];
            sz5 = [2,10000];
            sz6 = [2,10000];
            sz7 = [2,10000];            
        end
        function [cp1, cp2, cp3, cp4, cp5, cp6, cp7] = isInputComplexImpl(~)
        	cp1 = false;
            cp2 = false;
            cp3 = false;
            cp4 = false;
            cp5 = false;
            cp6 = false;
            cp7 = false;
        end
        function [cp1] = isOutputComplexImpl(~)
        	cp1 = false;
        end
        function [fz1, fz2, fz3, fz4, fz5, fz6, fz7] = isInputFixedSizeImpl(~)
        	fz1 = true;
            fz2 = true;
            fz3 = true;
            fz4 = true;
            fz5 = true;
            fz6 = true;
            fz7 = true;
        end
        function [fz1] = isOutputFixedSizeImpl(~)
        	fz1 = true;
        end
        function setupImpl(obj,~,~,~,~,~,~,~)
            
            import casadi.*

            m = 1430;           Iz = 1300;              % Mass inertia
            Lf = 1.056;         Lr = 2.4-1.056;         % Distance from center of mass to front and rear
            Kf = 41850.8527587; Kr = 51175.775017;      % Tire stiffness
            Cr2 = 0.1;                                  % Second order friction parameter
            Cr0 = 0.6;                                  % Zero order friction parameter
            R = 0.292;                                  % Wheel radius
            Tmax = 584;         G = 16.96;              % Maximum torque and steering wheel to steering angle coeff

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
            DT = 0.04/M;                                % Discretization step
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
            obj.cen = opti.parameter(3,obj.N);        % Centerline points
            obj.ref_v = opti.parameter(1,obj.N);      % Reference velocity
            obj.states = opti.parameter(7,1);           % Current car states
            obj.Wx = opti.parameter();                  % weight for x
            obj.Wy = opti.parameter();                  % weight for y
            obj.Wh = opti.parameter();                  % weight for y
            obj.Wv = opti.parameter();                  % weight for velocity
            obj.Wt = opti.parameter();                  % weight for throttle
            obj.Wsr = opti.parameter();                 % weight for steering rate            
            
            J = 0;   
            opti.subject_to(obj.x(:,1) - obj.states == 0);

            for i = 1:obj.N-1
                
                J = J + obj.Wx*(obj.x(1,i)-obj.cen(1,i))^2;                                 % Cost for distance to ref in x
                J = J + obj.Wy*(obj.x(2,i)-obj.cen(2,i))^2;                                 % Cost for distance to ref in y
                J = J + obj.Wh*(obj.x(5,i)-obj.cen(3,i))^2;
                J = J + obj.Wv*(obj.x(3,i)-obj.ref_v(i))^2;                                 % Cost for error in velocity
                J = J + obj.Wsr*obj.u(1,i)^2 + obj.Wt*obj.u(2,i)^2;                         % Costs on inputs

                opti.subject_to( 5 <= obj.x(3,i) <= 40);                                    % Constraint on speed
                opti.subject_to( obj.x(4,i) <= 1.5);
                opti.subject_to( -300*pi/180 <= obj.x(7,i) <= 300*pi/180 );                 % Contraint on steering angle
                opti.subject_to( -300*pi/180 <= obj.u(1,i) <= 300*pi/180 );                 % Contraint on steering rate
                opti.subject_to( -1 <= obj.u(2,i) <= 1 );                                   % Constraint on acceleration
                x_new = obj.Func('x0', obj.x(:,i), 'p', obj.u(:,i));                        % Calculating new states
                opti.subject_to( obj.x(:,i+1) - x_new.xf == 0 );                            % Set state equal to computed state
            end

            obj.optimizer = opti();
            obj.cost = J;
        end

        function reference = stepImpl(obj, current_states, reference_velocity, centerline1, centerline2, left_line, middle_line, right_line)         
            
            global coefficients_poly;
            
            % Computing distance to all points on right centerline
            dist_to_c1 = (centerline1(1,:) - current_states(1)).^2 + (centerline1(2,:) - current_states(2)).^2;
            
            % Finding index of closest point on right centerline
            [dc1, I] = min(dist_to_c1);
            
            % Computing distance to all points on left centerline
            dist_to_c2 = (centerline2(1,:) - current_states(1)).^2 + (centerline2(2,:) - current_states(2)).^2;
            
            % Finding index of closest point on left centerline
            [dc2, ~] = min(dist_to_c2);
            
            center1 = zeros(2,100);
            center2 = zeros(2,100);
            rline = zeros(2,100);
            lline = zeros(2,100);
            mline = zeros(2,100);
            ref_vel = zeros(1,100);
            
            index = I;
            
            for i = 1:100
                
                if index > length(centerline1(1,:))
                    index = 1;
                end
                center1(1:3,i) = centerline1(1:3,index);
                center2(1:3,i) = centerline2(1:3,index);
                rline(1:2,i) = right_line(1:2,index);
                lline(1:2,i) = left_line(1:2,index);
                mline(1:2,i) = middle_line(1:2,index);
                
                ref_vel(i) = reference_velocity;
                
                step = round(reference_velocity/5)*4;
                index = index + step;
            end

            
            % Initial guess for the optimization is chosen based on which
            % centerline is closer
            if dc1<dc2
                init_x = center1(:,1:100);
            else 
                init_x = center2(:,1:100);
            end
            
            % Defining all the optimisation parameters
            opti = obj.optimizer;
            opti.set_value(obj.ref_v, ref_vel);
            opti.set_value(obj.states, current_states);
            opti.set_value(obj.cen, center1(:,1:100));
            opti.set_value(obj.Wv, 10);
            opti.set_value(obj.Wt, 10);
            opti.set_value(obj.Wsr, 100);
            opti.set_value(obj.Wy, 20);
            opti.set_value(obj.Wx, 20);
            opti.set_value(obj.Wh, 0);
            
            % Providing an initial guess for the optimisation
            opti.set_initial(obj.x, [init_x(1:2,:); ref_vel; zeros(1,100); init_x(3,:); zeros(1,100); zeros(1,100)]);            

            % Chosing the solver and performing the optimization
            opti.minimize(obj.cost);
            options.ipopt.print_level = 0;
            opti.solver('ipopt',options);
            try
                sol = opti.solve();
                ref = sol.value(obj.x);
            catch 
                ref = opti.debug.value(obj.x);
            end
                       
            % Fitting the references with a spline and interpolating to
            % obtain 400 points
            L = 15.96;
            t = linspace(0,L,length(ref(1,:)));
            
            tq = linspace(0,L,400);

            reference(1,:) = spline(t,[0 ref(1,:) 0],tq);         % X
            reference(2,:) = spline(t,[0 ref(2,:) 0],tq);         % Y
            reference(3,:) = spline(t,[0 ref(3,:) 0],tq);         % Theta
            
            figure(1), plot(left_line(1,:), left_line(2,:), 'r', right_line(1,:), right_line(2,:), 'r', centerline1(1,:), centerline1(2,:),reference(1,:), reference(2,:),current_states(1), current_states(2), 'bo')
              
            mem_ref = reference(1:2,1:50);
            
            ROT = [cos(current_states(4)) sin(current_states(4));
                    -sin(current_states(4)) cos(current_states(4))];
                    
            for i = 1:length(mem_ref)
                mem_ref(:,i) = ROT*mem_ref(:,i) - ROT*[current_states(1); current_states(2)];
            end
            
            t = 0:0.01:0.49;
            ty = linspace(0,0.49,3);
            
            coefficients_poly(1:3) = polyfit(t,mem_ref(1,:),2);
            pp = spline(ty,mem_ref(2,[1 25 50]));
            Bs = fn2fm(pp, 'B-');
            coefficients_poly(4:6) = Bs.coefs;

        end
        
        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj,'Type','Discrete',...
                      'SampleTime',obj.Ts, ...
                      'OffsetTime',0);
        end

        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
        
        
    end
end