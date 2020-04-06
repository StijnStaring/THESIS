classdef Planning2 < matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime


    properties (Access = private)
        N = 100                                 % Length of the MPC horizon
        optimizer                               % Opti structure, necessary for casadi optimisation
        cost                                    % Cost function for MPC
        x                                       % Car states
        u                                       % Car inputs
        Ts = 0.01                               % Sampling time

        ref_v                                   % Velocity reference
        states                                  % Current car states
        rl                                      % Right road lane parameters
        ll                                      % Left road lane parameters
        ml                                      % Middle road lane parameters        
        cen1                                    % Centerline right parameters
        
        Msr                                     % Maximum Steering rate
        WF                                      % Weight for road potential field
        Wv                                      % Weight for velocity error
        Wvv                                     % Weight for velocity size
        Wt                                      % Weight for throttle
        Ws                                      % Weight for steering
        Wceny                                   % Weight for y-coordinate centerline right error
        Wcenx                                   % Weight for y-coordinate centerline right error
    end

    methods (Access = protected)
        
        function num = getNumInputsImpl(~)
            num = 7;
        end
        function num = getNumOutputsImpl(~)
            num = 1;
        end
        function [dt1, dt2] = getOutputDataTypeImpl(~)
        	dt1 = 'double';
            dt2 = 'double';
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
            sz1 = [4,400];
        end
        function [sz1, sz2, sz3, sz4, sz5, sz6, sz7] = getInputSizeImpl(~)
        	sz1 = [1,6];
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

            % Declare model variables (Point-mass model on tangent
            % reference frame, neglected beta) 
            x1 = SX.sym('x1');          % X
            x2 = SX.sym('x2');          % Y
            x3 = SX.sym('x3');          % Vx
            x4 = SX.sym('x4');          % Yaw
            state = [x1; x2; x3; x4];   % states
            u1 = SX.sym('u1');          % steering rate
            u2 = SX.sym('u2');          % throttle
            input = [u1; u2];           % control inputs

            % Model equations
            xdot = [x3*cos(x4);...      % State equation in x
                    x3*sin(x4);...      % State equation in y
                    u2; ...             % State equation in v
                    u1];                % State equation in yaw

            % Continous time dynamics
            f = casadi.Function('f', {state, input}, {xdot},{'x','u'},{'Eq'});            
            
            % Discrete time dynamics, fixed step Runge-Kutta integrator
            M = 4;                          % Number of iterations
            DT = 0.04/M;                    % Discretization step
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
            %Discrete time dynamics funtion
            F = Function('F', {X0, U}, {X}, {'x0','p'}, {'xf'});
                       
            
            % MPC problem setup
            opti = casadi.Opti();                       % Creating the opti structure
            obj.x = opti.variable(4,obj.N);             % State matrix across the whole horizon
            obj.u = opti.variable(2,obj.N);             % Input matrix over whole horizon
            
            obj.ref_v = opti.parameter(1,obj.N);        % Reference velocity
            obj.states = opti.parameter(4,1);           % Current car states
            obj.rl = opti.parameter(2,obj.N);           % Right line parameters
            obj.ml = opti.parameter(2,obj.N);           % Middle line parameteers
            obj.ll = opti.parameter(2,obj.N);           % Left line parameters
            obj.cen1 = opti.parameter(2,obj.N);         % Left line parameters
            obj.Msr = opti.parameter();                 % Maximum steering rate
            
            obj.Wv = opti.parameter();                  % Weight for velocity error
            obj.Wvv = opti.parameter();                 % Weight for velocity amplitude
            obj.Wt = opti.parameter();                  % Weight for throttle
            obj.Ws = opti.parameter();                  % Weight for yaw
            obj.Wcenx = opti.parameter();               % Weight for x-coordinate centerline right error
            obj.Wceny = opti.parameter();               % Weight for y-coordinate centerline right error
            
            J = 0;
            opti.subject_to(obj.x(:,1) - obj.states == 0);              % Set initial states equal to current car states
            
            for i = 1:obj.N-1
                
                x_new = F('x0', obj.x(:,i), 'p', obj.u(:,i));           % Calculating new states
                
                J = J + obj.Wv*(obj.x(3,i)-obj.ref_v(i))^2;                % Cost for error in velocity
                J = J + obj.Ws*obj.u(1,i)^2 + obj.Wt*obj.u(2,i)^2;      % Costs for inputs
                
                J = J + obj.Wceny*((obj.x(2,i)-obj.cen1(2,i))^2);       % Cost for staying on the right lane
                J = J + obj.Wcenx*((obj.x(1,i)-obj.cen1(1,i))^2);
               
               
                opti.subject_to( 5 <= obj.x(3,i) <= 40 );                                % constraint on velocity
                opti.subject_to( -obj.Msr*pi/180 <= obj.u(1,i) <= obj.Msr*pi/180 );      % constraint on steering rate
                opti.subject_to( -1 <= obj.u(2,i) <= 1);                                 % constraint on acceleration
                opti.subject_to( obj.x(:,i+1) - x_new.xf == 0 );                         % set state equal to computed state
            end
            
            for i = 1:50
                
                opti.subject_to( obj.rl(2,i) + 0.45 <= obj.x(2,i) );
                opti.subject_to( obj.x(2,i) <= obj.ll(2,i) - 0.45 );
                
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
                    I = 1;
                end
                center1(1:2,i) = centerline1(1:2,index);
                center2(1:2,i) = centerline2(1:2,index);
                rline(1:2,i) = right_line(1:2,index);
                lline(1:2,i) = left_line(1:2,index);
                mline(1:2,i) = middle_line(1:2,index);
                
                ref_vel(i) = reference_velocity;
                
                step = round(reference_velocity/5)*4;
                index = index + step;
            end
            
            th = -centerline1(3,I);
            ROT = [cos(th) -sin(th);                %Computing rotation matrix
                    sin(th) cos(th)];
    
            % Defining all rotation vectors in order to brind all values to
            % local frame of reference
            c1r = zeros(2,length(center1)); c1rt = zeros(2,length(center2));
            c2rt = zeros(2,length(center2));  c2r = zeros(2,length(center2));
            rlrt = zeros(2,length(rline));  rlr = zeros(2,length(rline)); 
            llrt = zeros(2,length(rline));  llr = zeros(2,length(rline)); 
            mlrt = zeros(2,length(rline));  mlr = zeros(2,length(rline));
            
            % Rotating both centerlines
            for i = 1:length(center1)
                c1r(:,i) = ROT * center1(:,i);
                c2r(:,i) = ROT * center2(:,i);
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
     
            orefr = ROT*current_states(1:2);
            x_lane_ref = orefr(1) - c1r(1,1);
            y_lane_ref = orefr(2) - c1r(2,1);
            th_lane_ref = current_states(4) + th;
            st = [x_lane_ref; y_lane_ref; current_states(3); th_lane_ref];

            
            % Initial guess for the optimization is chosen based on which
            % centerline is closer
            if dc1<dc2
                init_x = c1rt(:,1:100);
            else 
                init_x = c2rt(:,1:100);
            end
            
            % Defining all the optimisation parameters
            opti = obj.optimizer;
            opti.set_value(obj.ref_v, ref_vel);
            opti.set_value(obj.states, st);
            opti.set_value(obj.rl, rlrt(:,1:100));
            opti.set_value(obj.ll, llrt(:,1:100));
            opti.set_value(obj.ml, mlrt(:,1:100));
            opti.set_value(obj.cen1, c1rt(:,1:100));
            opti.set_value(obj.Wv, 20);
            opti.set_value(obj.Wvv, 0);
            opti.set_value(obj.Wt, 10);
            opti.set_value(obj.Ws, 10);
            opti.set_value(obj.Wceny, 20);
            opti.set_value(obj.Wcenx, 1);
            opti.set_value(obj.Msr, 150);
            
            % Providing an initial guess for the optimisation
            opti.set_initial(obj.x, [init_x; ref_vel; th_lane_ref*ones(1,100)]);            

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
            
            theta = ref(4,:)'-th;

            tq = linspace(0,L,400);

            ref_rot(1,:) = spline(t,[0 ref(1,:) 0],tq);         % X
            ref_rot(2,:) = spline(t,[0 ref(2,:) 0],tq);         % Y
            ref_rot(3,:) = spline(t,[0 theta' 0],tq);           % Theta
            
            % Rotating and translating the reference back to global
            % coordinates
            ref_trans = zeros(2,length(ref_rot));
            reference = zeros(4,length(ref_rot));
            
            ref_trans(1,:) = ref_rot(1,:) + c1r(1,1);
            ref_trans(2,:) = ref_rot(2,:) + c1r(2,1);
            
            for i = 1:length(reference)
                reference(1:2,i) = ROT\ref_trans(1:2,i);
            end 

            reference(3,:) = ref_rot(3,:);
            
            % Interpolating velocity vector
            s = 1:4:400;
            ss = 1:400;
            fv = fit(s',ref(3,:)','poly2');
            reference(4,:) = fv(ss');
            
            figure(1), plot(left_line(1,:), left_line(2,:), 'r', right_line(1,:), right_line(2,:), 'r', centerline1(1,:), centerline1(2,:),reference(1,:), reference(2,:),current_states(1), current_states(2), 'bo')
%             figure(2), plot(1:400, reference(4,:), 'r', 1:4:400, ref_vel, 'b')
%             figure(2), plot(1:40, ref(3,1:40))
              
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