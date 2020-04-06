classdef Planning1 < matlab.System & matlab.system.mixin.Propagates

    % This code creates a reference path of 10s using bycicle model of the car to keep the lane

    properties (Access = private)
        % Global variables are defined here
        old_reference = zeros(6,100)                     % Store the value of the previous reference
        N = 100                                        % Length of the MPC horizon
        number_of_points = 100                         % Number of points in the reference
        optimizer                                       % Opti structure, necessary for casadi optimisation
        cost                                            % Cost function for MPC
        x                                               % Car states
        u                                               % Car inputs
        Ts = 0.01                                       % Sampling time

        ref_v                                   % Velocity reference
        states                                  % Current car states
        crl                                     % Centerline right lane parameters
        rl                                      % Right road lane parameters
        ll                                      % Left road lane parameters
        ml                                      % Middle road lane parameters       
        Func
        Msr                                     % Maximum Steering rate
        Wv                                      % Weight for velocity error
        Wvv                                     % Weight for velocity size
        Wt                                      % Weight for throttle
        Ws                                      % Weight for steering
        Wp
    end

    methods (Access = protected)
        % Here are defined the properties of all inputs and output of the block
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
            sz1 = [6,1];  % (Vx Vy r phi x y)
        end
        function [sz1, sz2, sz3, sz4, sz5, sz6, sz7] = getInputSizeImpl(~)
        	sz1 = [1,6];     % current states (X Y Vx Vy phi r)
            sz2 = [1,1];     % reference velocity
            sz3 = [2,19694]; % centerline right
            sz4 = [2,19694]; % centerline left
            sz5 = [2,19694]; % left line
            sz6 = [2,19694]; % middle line
            sz7 = [2,19694]; % right line
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
        function setupImpl(obj,~,~,~,~,~,~,~,~,~)
            % Initialization function, MPC setup using casadi
            
            import casadi.*

            m  = 1430; Iz = 1300;
            Lf = 1.056; 
            Lr = 1.344; 
            Kf= 41850.8527587; Kr = 51175.775017;

            Cr0  = 0.6; Cr2 = 0.1; R = 0.292;
            Tmax = 584; pi = 3.14159265359;
            G = 16.96;

            % Declare model variables
            x1 = casadi.SX.sym('x1');                       % X
            x2 = casadi.SX.sym('x2');                       % Y
            x3 = casadi.SX.sym('x3');                       % Vx
            x4 = casadi.SX.sym('x4');                       % Vy
            x5 = casadi.SX.sym('x5');                       % Yaw
            x6 = casadi.SX.sym('x6');                       % Yaw_rate
            state = [x1; x2; x3; x4; x5; x6];               % States
            u1 = casadi.SX.sym('u1');                       % Steering wheel angle
            u2 = casadi.SX.sym('u2');                       % Throttle
            input = [u1; u2];                               % Control inputs

            % Model equations
            slip_f = -atan2((x6*Lf + x4), (x3)) + u1/ G;   % Slip angle at front wheel
            slip_r =  atan2((x6*Lr - x4), (x3));           % Slip angle at rear wheel

            Fxf = 0.5*u2*Tmax/ R;                          % Tire force x front
            Fxr = Fxf;                                     % Tire force x rear

            Fyf = Kf*slip_f;                            % Tire force y front
            Fyr = Kr*slip_r;                            % Tire force y rear

            Fres = Cr0 + Cr2*x3*x3;                     % air resistance force

            xdot = [ x3*cos(x5) - x4*sin(x5);...                                                % State equation in x
                     x3*sin(x5) + x4*cos(x5);...                                                % State equation in y
                    ( 1/ m )*( Fxf*cos(u1/ G) + Fxr - Fyf*sin(u1/ G) - Fres + m*x6*x4 ); ...    % State equation in vx
                    ( 1/ m )*( Fxf*sin(u1/ G) + Fyf*cos(u1/ G) + Fyr - m*x6*x3 ) ; ...          % State equation in vy
                     x6 ; ...                                                                   % State equation in yaw
                    ( 1/ Iz )*( Lf*( Fyf*cos(u1/ G) + Fxf*sin(u1/ G)) - Lr*Fyr)];               % State equation in yaw_rate

            % Continous time dynamics
            f = casadi.Function('f', {state, input}, {xdot},{'x','u'},{'Eq'});

            % Discrete time dynamics, fixed step Runge-Kutta integrator
            M = 4;                                      % Number of iterations
            DT = obj.Ts/M;                              % Discretization step
            X0 = casadi.SX.sym('X0', 6);                % Initial state
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
            
            % MPC problem setup
            opti = casadi.Opti();                       % Creating the opti structure
            obj.x = opti.variable(6,obj.N);             % State matrix across the whole horizon
            obj.u = opti.variable(2,obj.N);             % Input matrix over whole horizon
            
            obj.ref_v = opti.parameter();               % Reference velocity
            obj.states = opti.parameter(6,1);           % Current car states
            obj.crl = opti.parameter(2,obj.N);
            obj.rl = opti.parameter(2,obj.N);           % Right line parameters
            obj.ml = opti.parameter(2,obj.N);           % Middle line parameters
            obj.ll = opti.parameter(2,obj.N);           % Left line parameters
            obj.Msr = opti.parameter();                 % Maximum steering rate
            
            obj.Wp = opti.parameter();
            obj.Wv = opti.parameter();                  % Weight for velocity error
            obj.Wt = opti.parameter();                  % Weight for throttle
            obj.Ws = opti.parameter();                  % Weight for yaw
            
            J = 0;
            opti.subject_to(obj.x(:,1) - obj.states == 0);                      % Set initial states equal to current car states
            
            for i = 1:obj.N-1
                x_new = F('x0', obj.x(:,i), 'p', obj.u(:,i));                   % Calculating new states
                
%                 J = J + obj.Wp*((sqrt(obj.x(1,i)^2 + obj.x(2,i)^2) - sqrt(obj.crl(1,i)^2 + obj.crl(2,i)^2))^2);
                J = J + obj.Wv*(obj.x(3,i)-obj.ref_v)^2;                        % Cost for error in velocity
                J = J + obj.Ws*obj.u(1,i)^2 + obj.Wt*obj.u(2,i)^2;              % Costs for inputs
                
                % Lane boundary constraints
                v1 = [obj.rl(1,i+1)-obj.rl(1,i);obj.rl(2,i+1)-obj.rl(2,i)];     % Vector of right line heading
                v2 = [obj.x(1,i)-obj.rl(1,i);obj.x(2,i)-obj.rl(2,i)];           % Vector of car position relative to right line
                v3 = [obj.ll(1,i+1)-obj.ll(1,i);obj.ll(2,i+1)-obj.ll(2,i)];     % Vector of left line heading
                v4 = [obj.x(1,i)-obj.ll(1,i);obj.x(2,i)-obj.ll(2,i)];           % Vector of car position relative to left line
                v1n = sqrt(v1(1)^2 + v1(2)^2);                                  % Right line heading vector length
                v3n = sqrt(v3(1)^2 + v3(2)^2);                                  % Left line heading vector length
                
%                 opti.subject_to( 0.5 <= (v1(1)*v2(2)-v1(2)*v2(1))/v1n );        % Constraint for staying on the left of right line
%                 opti.subject_to( (v3(1)*v4(2)-v3(2)*v4(1))/v3n <= -0.5 );       % Constraint for staying on the right of left line
%                            
%                 opti.subject_to( 0 <= obj.x(3,i) <= 40 );                       % constraint on velocity
%                 opti.subject_to( -obj.Msr*pi/180 <= obj.u(1,i) <= obj.Msr*pi/180 );  % constraint on steering angle
                opti.subject_to( -1 <= obj.u(2,i) <= 1 );                       % constraint on acceleration
                opti.subject_to( obj.x(:,i+1) - x_new.xf == 0 );                % set state equal to computed state
            end
            
            obj.optimizer = opti();
            obj.cost = J;
        end

        function [reference] = stepImpl(obj, current_states, reference_velocity, centerline1, centerline2, left_line, middle_line, right_line)
%             % Computing distance to all points on old reference
            dist_old_ref = (obj.old_reference(1,:) - current_states(1)).^2 + (obj.old_reference(2,:) - current_states(2)).^2;
%             
%             % Finding index of closest point on old reference
            [dof, Index_start] = min(dist_old_ref);
%             
%             % Computing distance to all points on right centerline
            dist_to_c1 = (centerline1(1,:) - current_states(1)).^2 + (centerline1(2,:) - current_states(2)).^2;
            
            % Finding index and distcance of closest point on right centerline
            [dc1, I] = min(dist_to_c1);
%             
%             % Computing distance to all points on left centerline
            dist_to_c2 = (centerline2(1,:) - current_states(1)).^2 + (centerline2(2,:) - current_states(2)).^2;
            
%             Finding distance of closest point on left centerline
            [dc2, ~] = min(dist_to_c2);
          
            
            length_ref = ceil(reference_velocity*100);                         % Computing length of vector needed
            step = floor(reference_velocity/5);                     % Computing step
            
            if (I + length_ref > length(centerline1(1,:)))
                I = 1;
            end
           
            cen1 = centerline1(1:2, I: step: I+length_ref);              % Taking the approptiate points from the road lines
            cen2 = centerline2(1:2, I: step: I+length_ref);
            rline = right_line(1:2, I: 5+step :I+length_ref);
            lline = left_line(1:2, I: 5+step :I+length_ref);
            mline = middle_line(1:2, I: 5+step :I+length_ref);

            
            th = -centerline1(3,I);
            ROT = [cos(th) -sin(th);                %Computing rotation matrix
                    sin(th) cos(th)];
    
            % Defining all rotation vectors in order to brind all values to
            % local frame of reference
            c1r = zeros(2,length(cen1)); c1rt = zeros(2,length(cen2));
            c2rt = zeros(2,length(cen2));  c2r = zeros(2,length(cen2));
            rlrt = zeros(2,length(rline));  rlr = zeros(2,length(rline)); 
            llrt = zeros(2,length(rline));  llr = zeros(2,length(rline)); 
            mlrt = zeros(2,length(rline));  mlr = zeros(2,length(rline));
            
            % Rotating both centerlines
            for i = 1:length(cen1)
                c1r(:,i) = ROT * cen1(:,i);
                c2r(:,i) = ROT * cen2(:,i);
            end 
            
            % Rotating road lines
            for i = 1:length(rline)
                rlr(:,i) = ROT * rline(:,i);
                llr(:,i) = ROT * lline(:,i);
                mlr(:,i) = ROT * mline(:,i);
            end
            
            % Translating all lines to start at zero
            c1rt(1,:) = c1r(1,:) - c1r(1,1); c1rt(2,:) = c1r(2,:) - c1r(2,1);
            c2rt(1,:) = c2r(1,:) - c1r(1,1); c2rt(2,:) = c2r(2,:) - c1r(2,1);
            rlrt(1,:) = rlr(1,:) - c1r(1,1); rlrt(2,:) = rlr(2,:) - c1r(2,1);
            llrt(1,:) = llr(1,:) - c1r(1,1); llrt(2,:) = llr(2,:) - c1r(2,1);
            mlrt(1,:) = mlr(1,:) - c1r(1,1); mlrt(2,:) = mlr(2,:) - c1r(2,1);
            
%             Choosing the initial states for the MPC. If the car is close
%             to the previous reference, then the closest point of the
%             previous reference becomes the initial state. Otherwise the
%             states of the car are taken
%             if dof < 1
%                 orefr = ROT*obj.old_reference(1:2);
%                 x_lane_ref = orefr(1) - c1r(1,1);
%                 y_lane_ref = orefr(2) - c1r(2,1);
%                 th_lane_ref = obj.old_reference(3) + th;
%                 st = [x_lane_ref; y_lane_ref; obj.old_reference(4); 0; th_lane_ref; 0];
%             else
%                 orefr = ROT*current_states(1:2);
%                 x_lane_ref = orefr(1) - c1r(1,1);
%                 y_lane_ref = orefr(2) - c1r(2,1);
%                 th_lane_ref = current_states(4) + th;
%                 st = [x_lane_ref; y_lane_ref; current_states(3); 0; th_lane_ref; 0];
%             end
            
            st = current_states;
            % Initial guess for the optimization is chosen based on which
            % centerline is closer
            if dc1<dc2
                init_x = c1rt(:,1:100);
            else 
                init_x = c2rt(:,1:100);
            end
           
            
            % Defining all the optimisation parameters
            opti = obj.optimizer;
            opti.set_value(obj.ref_v, reference_velocity);
            opti.set_value(obj.states, st);
            opti.set_value(obj.crl, c1rt(:,1:100));
            opti.set_value(obj.rl, rlrt(:,1:100));
            opti.set_value(obj.ll, llrt(:,1:100));
            opti.set_value(obj.ml, mlrt(:,1:100));
            opti.set_value(obj.Wv, 10); % Weight for velocity error
            opti.set_value(obj.Wt, 10);  % Weight for throttle
            opti.set_value(obj.Ws, 10);  % Weight for steering angle
            opti.set_value(obj.Msr, 150);  % Maximum steering angle
            opti.set_value(obj.Wp, 1000);
            
            % Providing an initial guess for the optimisation
            opti.set_initial(obj.x, [init_x; reference_velocity*ones(1,obj.N); zeros(1,obj.N); zeros(1,obj.N); zeros(1,obj.N)]);            

            % Chosing the solver and performing the optimization
            opti.minimize(obj.cost);
            options.ipopt.print_level = 0;
            opti.solver('ipopt',options);
            sol = opti.solve();
            ref = sol.value(obj.x);
            
            reference = [ref(3,1); ref(4,1); ref(5,1); ref(6,1); ref(1,1); ref(2,1)];
            
            % Saving reference for next time step
            obj.old_reference = ref;
        end

        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end