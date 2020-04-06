classdef Planing < matlab.System & matlab.system.mixin.Propagates

    % This code create a reference path using potential field functions to
    % keep the car within the road and around obstacles

    properties (Access = private)
        % Global variables are defined here
        old_reference = [zeros(3,400);10*ones(1,400)] %Store the value of the previous reference
        N = 100                                 % Length of the MPC horizon
        number_of_points = 399                  % Number of points in the reference
        optimizer                               % Opti structure, necessary for casadi optimisation
        cost                                    % Cost function for MPC
        x                                       % Car states
        u                                       % Car inputs
        Field                                   % Road otential field value
        Obs_field                               % Obstacle potential field value
        Ts = 0.04                               % Sampling time

        ref_v                                   % Velocity reference
        states                                  % Current car states
        coeffr                                  % Plynomial coefficients for right lane fit
        coeffl                                  % Plynomial coefficients for right lane fit
        obs                                     % Obstacle states (position, velocity and heading)
        obs2                                    % same for a second obstacle
        obs_param                               % Obstacle parameters (width length etc..)
        rl                                      % Right road lane parameters
        ll                                      % Left road lane parameters
        ml                                      % Middle road lane parameters        
        
        beta                                    % Variable used for polyhedron collision avoidance 
        beta2                                   % Same but for a second obstacle
        
        Msr                                     % Maximum Steering rate
        WF                                      % Weight for road potential field
        WFo                                     % Weight for obstacle potential field
        Wv                                      % Weight for velocity error
        Wvv                                     % Weight for velocity size
        Wt                                      % Weight for throttle
        Ws                                      % Weight for steering
    end

    methods (Access = protected)
        % Here are defined the properties of all inputs and output of the block
        function num = getNumInputsImpl(~)
            num = 9;
        end
        function num = getNumOutputsImpl(~)
            num = 1;
        end
        function [dt1] = getOutputDataTypeImpl(~)
        	dt1 = 'double';
        end
        function [dt1, dt2, dt3, dt4, dt5, dt6, dt7, dt8, dt9] = getInputDataTypeImpl(~)
        	dt1 = 'double';
            dt2 = 'double';
            dt3 = 'double';
            dt4 = 'double';
            dt5 = 'double';
            dt6 = 'double';
            dt7 = 'double';
            dt8 = 'double';
            dt9 = 'double';
        end
        function [sz1] = getOutputSizeImpl(~)
            sz1 = [4,400];
        end
        function [sz1, sz2, sz3, sz4, sz5, sz6, sz7, sz8, sz9] = getInputSizeImpl(~)
        	sz1 = [1,5];
            sz2 = [1,1];
            sz3 = [5,10000];
            sz4 = [5,10000];
            sz5 = [2,10000];
            sz6 = [2,10000];
            sz7 = [2,10000];
            sz8 = [8,100];
            sz9 = [1,1];            
        end
        function [cp1, cp2, cp3, cp4, cp5, cp6, cp7, cp8, cp9] = isInputComplexImpl(~)
        	cp1 = false;
            cp2 = false;
            cp3 = false;
            cp4 = false;
            cp5 = false;
            cp6 = false;
            cp7 = false;
            cp8 = false;
            cp9 = false;
        end
        function [cp1] = isOutputComplexImpl(~)
        	cp1 = false;
        end
        function [fz1, fz2, fz3, fz4, fz5, fz6, fz7, fz8, fz9] = isInputFixedSizeImpl(~)
        	fz1 = true;
            fz2 = true;
            fz3 = true;
            fz4 = true;
            fz5 = true;
            fz6 = true;
            fz7 = true;
            fz8 = true;
            fz9 = true;
        end
        function [fz1] = isOutputFixedSizeImpl(~)
        	fz1 = true;
        end
        function setupImpl(obj,~,~,~,~,~,~,~,~,~)
            % Initialization function, MPC setup using casadi
            
            import casadi.*

            % Declare model variables
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
            DT = obj.Ts/M;                  % Discretization step
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
            
            % Road potential field definition
            pos_x = SX.sym('posx');         % Position in space, x coord
            pos_y = SX.sym('posy');         % Position in space, y coord
            coeff = SX.sym('coeff',8);      % Polinomial coefficient for road shape fit
            sig = SX.sym('sig');            % Sign determining which side of the road we are on and height of minimum
            depth = SX.sym('depth');        % Depth of well in PF function
            
            % Additional calculations
            m = -1/(2*coeff(1)*pos_x+coeff(3)+1e-5);        
            Y = sum(coeff'*[pos_x^7;pos_x^6;pos_x^5;pos_x^4;pos_x^3;pos_x^2;pos_x;1]);
            b = Y - m*pos_x;
            % PF function
            pf = 5*(depth - exp(-sig*sign(pos_y - Y+1e-5)*sqrt(((pos_y - b)/m - pos_x)^2 + (Y - pos_y+1e-5)^2)))^2;

            Pot = casadi.Function('Pot', {pos_x, pos_y, coeff, sig, depth}, {pf}, {'x', 'y', 'coeff', 'sign', 'depth'}, {'val'});
            
            % Obstacle potential field definition
            o_x = SX.sym('o_x');            % Obstacle position x
            o_y = SX.sym('o_y');            % Obstacle position y
            o_h = SX.sym('o_h');            % Obstacle heading
            A0 = SX.sym('A0');              % Height of potential field
            a = SX.sym('a');                % Ellipse semi-major axis
            b = SX.sym('b');                % Ellipse semi-minor axis
            alpha = cos(o_h)^2/(2*a^2)+sin(o_h)^2/(2*b^2);              % Rotation coefficient alpha
            betaa = -(-sin(2*o_h)^2/(4*a^2)+sin(2*o_h)^2/(4*b^2));      % Rotation coefficient beta
            gama = sin(o_h)^2/(2*a^2)+cos(o_h)^2/(2*b^2);               % Rotation coefficient gama
            
            % PF function
            pfo = A0*exp(-(alpha*(pos_x-o_x)^2 + 2*betaa*(pos_x-o_x)*(pos_y-o_y) + gama*(pos_y-o_y)^2));

            Pot_o = casadi.Function('Pot_O', {pos_x, pos_y, o_x, o_y, o_h, A0, a, b}, {pfo}, {'x', 'y', 'ox', 'oy', 'oh', 'height', 'length', 'width'}, {'val'});
            
            % MPC problem setup
            opti = casadi.Opti();                       % Creating the opti structure
            obj.x = opti.variable(4,obj.N);             % State matrix across the whole horizon
            obj.u = opti.variable(2,obj.N);             % Input matrix over whole horizon
            obj.Field = opti.variable(1,obj.N);         % Road potential field vector over horizon
            obj.Obs_field = opti.variable(1,obj.N);     % Obstacle potential field vector over horizon
            
            obj.ref_v = opti.parameter();               % Reference velocity
            obj.states = opti.parameter(4,1);           % Current car states
            obj.coeffr = opti.parameter(8,1);           % Polinomial coefficients for right centerline fitting
            obj.coeffl = opti.parameter(8,1);           % Polinomial coefficients for left centerline fitting
            obj.rl = opti.parameter(2,obj.N);           % Right line parameters
            obj.ml = opti.parameter(2,obj.N);           % Middle line parameteers
            obj.ll = opti.parameter(2,obj.N);           % Left line parameters
            obj.Msr = opti.parameter();                 % Maximum steering rate
            obj.obs = opti.parameter(4,obj.N);          % Obstacle states
            obj.obs_param = opti.parameter(3);          % Obstacle parameters
            obj.beta = opti.variable(8,obj.N);          % Polyhedron collision avoidance constraints
            
            obj.Wv = opti.parameter();                  % Weight for velocity error
            obj.Wvv = opti.parameter();                 % Weight for velocity amplitude
            obj.Wt = opti.parameter();                  % Weight for throttle
            obj.Ws = opti.parameter();                  % Weight for yaw
            obj.WF = opti.parameter();                  % Weight for road PF
            obj.WFo = opti.parameter();                 % Weight for obstacle PF
            
            J = 0;
            opti.subject_to(obj.x(:,1) - obj.states == 0);              % Set initial states equal to current car states
            for i = 1:obj.N-1;
                x_new = F('x0', obj.x(:,i), 'p', obj.u(:,i));           % Calculating new states
                % Right lane potential field
                Field_right = Pot('x', obj.x(1,i), 'y', obj.x(2,i), 'coeff', obj.coeffr, 'sign', 1.5, 'depth', 1);
                % Left lane potential field
                Field_left = Pot('x', obj.x(1,i), 'y', obj.x(2,i), 'coeff', obj.coeffl, 'sign', -1.5, 'depth', 0);
                % Obstacle potential field
                Field_obs = Pot_o('x', obj.x(1,i), 'y', obj.x(2,i), 'ox', obj.obs(1,i), 'oy', obj.obs(2,i), 'oh', obj.obs(3,i),...
                    'height', obj.obs_param(1), 'length', obj.obs_param(2), 'width', obj.obs_param(3));
                % Total potential field
                obj.Field(i) = Field_right.val + Field_left.val;
                obj.Obs_field(i) = Field_obs.val;
                
                J = J + obj.WF * obj.Field(i);                                  % Cost for road PF
                J = J + obj.WFo * obj.Obs_field(i);                             % Cost for obstacle PF
                J = J + obj.Wv*(obj.x(3,i)-obj.ref_v)^2;                        % Cost for error in velocity
                J = J + obj.Wvv*obj.x(3,i);                                     % Cost for velocity magnitude
                J = J + obj.Ws*obj.u(1,i)^2 + obj.Wt*obj.u(2,i)^2;              % Costs for inputs
                
                % Lane boundary constraints
                v1 = [obj.rl(1,i+1)-obj.rl(1,i);obj.rl(2,i+1)-obj.rl(2,i)];     % Vector of right line heading
                v2 = [obj.x(1,i)-obj.rl(1,i);obj.x(2,i)-obj.rl(2,i)];           % Vector of car position relative to right line
                v3 = [obj.ll(1,i+1)-obj.ll(1,i);obj.ll(2,i+1)-obj.ll(2,i)];     % Vector of left line heading
                v4 = [obj.x(1,i)-obj.ll(1,i);obj.x(2,i)-obj.ll(2,i)];           % Vector of car position relative to left line
                v1n = sqrt(v1(1)^2 + v1(2)^2);                                  % Right line heading vector length
                v3n = sqrt(v3(1)^2 + v3(2)^2);                                  % Left line heading vector length
                
                opti.subject_to( 0.5 <= (v1(1)*v2(2)-v1(2)*v2(1))/v1n );        % Constraint for staying on the left of right line
                opti.subject_to( (v3(1)*v4(2)-v3(2)*v4(1))/v3n <= -0.5 );       % Constraint for staying on the right of left line
                
                A = [ sin(obj.x(4,i)),    -cos(obj.x(4,i));...                  % Car orientation matrix
                     -sin(obj.x(4,i)),     cos(obj.x(4,i));...
                      cos(obj.x(4,i)),     sin(obj.x(4,i));...
                     -cos(obj.x(4,i)),    -sin(obj.x(4,i))];
                A1 = [sin(obj.obs(3,i)),    -cos(obj.obs(3,i));...              % Obstacle orientation matrix
                     -sin(obj.obs(3,i)),     cos(obj.obs(3,i));...
                      cos(obj.obs(3,i)),     sin(obj.obs(3,i));...
                     -cos(obj.obs(3,i)),    -sin(obj.obs(3,i))];
                b = [1.5; 1.5; 2.5; 2.5] + A*obj.x(1:2,i);                      % Car shape and position vector
                % Obstacle shape and position vector
                b1 = [obj.obs_param(3); obj.obs_param(3); obj.obs_param(2); obj.obs_param(2)] + A1*obj.obs(1:2,i);
                
                opti.subject_to( [A' A1']*obj.beta(:,i) == 0);                  % Constraints for poolyhedral intersection
                opti.subject_to( [b' b1']*obj.beta(:,i) < 0);
                opti.subject_to( obj.beta(:,i) > 0);
                
                
                opti.subject_to( -40 <= obj.x(3,i) <= 40 );                      % constraint on velocity
                opti.subject_to( -obj.Msr*pi/180 <= obj.u(1,i) <= obj.Msr*pi/180 );      % constraint on steering rate
                opti.subject_to( -1 <= obj.u(2,i) <= 1 );                        % constraint on acceleration
                opti.subject_to( obj.x(:,i+1) - x_new.xf == 0 );                 % set state equal to computed state
            end
            Field_right = Pot('x', obj.x(1,obj.N), 'y', obj.x(2,obj.N), 'coeff', obj.coeffr, 'sign', 0.5);
            Field_left = Pot('x', obj.x(1,obj.N), 'y', obj.x(2,obj.N), 'coeff', obj.coeffl, 'sign', -0.5);
            obj.Field(i) = Field_right.val + Field_left.val;
% 
            J = J + obj.WF * obj.Field(obj.N);                                  % Final PF cost
            obj.optimizer = opti();
            obj.cost = J;
        end

        function [reference] = stepImpl(obj, current_states, reference_velocity, centerline1, centerline2, left_line, middle_line, right_line, obstacle, obs_type)
            % Computing distance to all points on old reference
            dist_old_ref = (obj.old_reference(1,:) - current_states(1)).^2 + (obj.old_reference(2,:) - current_states(2)).^2;
            
            % Finding index of closest point on old reference
            [dof, Index_start] = min(dist_old_ref);
            
            % Computing distance to all points on right centerline
            dist_to_c1 = (centerline1(1,:) - current_states(1)).^2 + (centerline1(2,:) - current_states(2)).^2;
            
            % Finding index of closest point on right centerline
            [dc1, I] = min(dist_to_c1);
            
            % Computing distance to all points on left centerline
            dist_to_c2 = (centerline2(1,:) - current_states(1)).^2 + (centerline2(2,:) - current_states(2)).^2;
            
            % Finding index of closest point on left centerline
            [dc2, ~] = min(dist_to_c2);
            
            legth = reference_velocity*100;                         % Computing length of vector needed
            step = floor(reference_velocity/5);                     % Computing step
            cen1 = centerline1(1:2, I: step: I+legth);              % Taking the approptiate points from the road lines
            cen2 = centerline2(1:2, I: step: I+legth);
            rline = right_line(1:2, I: 5+step :I+legth-1);
            lline = left_line(1:2, I: 5+step :I+legth-1);
            mline = middle_line(1:2, I: 5+step :I+legth-1);
            
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
            obsrt = zeros(4,length(obstacle)); obsr = zeros(4,length(obstacle));
            
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
            
            % Rotating and translating obstacle states
            for i = 1:length(obstacle)
                obsr(1:2,i) = ROT * obstacle(1:2,i);
                obsrt(1,i) = obsr(1,i) - c1r(1,1);
                obsrt(2,i) = obsr(2,i) - c1r(2,1);
                obsrt(3,i) = obstacle(3,1) + th;
                obsrt(4,i) = obstacle(4,1);
            end
            
            % Translating all lines to start at zero
            c1rt(1,:) = c1r(1,:) - c1r(1,1); c1rt(2,:) = c1r(2,:) - c1r(2,1);
            c2rt(1,:) = c2r(1,:) - c1r(1,1); c2rt(2,:) = c2r(2,:) - c1r(2,1);
            rlrt(1,:) = rlr(1,:) - c1r(1,1); rlrt(2,:) = rlr(2,:) - c1r(2,1);
            llrt(1,:) = llr(1,:) - c1r(1,1); llrt(2,:) = llr(2,:) - c1r(2,1);
            mlrt(1,:) = mlr(1,:) - c1r(1,1); mlrt(2,:) = mlr(2,:) - c1r(2,1);

            % Polynomial fitting of both centerlines
            coeff_right = polyfit(c1rt(1,:),c1rt(2,:),7);   
            coeff_left = polyfit(c2rt(1,:),c2rt(2,:),7);
            
            % Choosing the initial states for the MPC. If the car is close
            % to the previous reference, then the closest point of the
            % previous reference becomes the initial state. Otherwise the
            % states of the car are taken
            if dof < 1
                orefr = ROT*obj.old_reference(1:2,Index_start);
                x_lane_ref = orefr(1) - c1r(1,1);
                y_lane_ref = orefr(2) - c1r(2,1);
                th_lane_ref = obj.old_reference(3,Index_start+10) + th;
                st = [x_lane_ref; y_lane_ref; obj.old_reference(4,Index_start); th_lane_ref];
            else
                orefr = ROT*current_states(1:2);
                x_lane_ref = orefr(1) - c1r(1,1);
                y_lane_ref = orefr(2) - c1r(2,1);
                th_lane_ref = current_states(4) + th;
                st = [x_lane_ref; y_lane_ref; current_states(3); th_lane_ref];
            end
            
            % Initial guess for the optimization is chosen based on which
            % centerline is closer
            if dc1<dc2
                init_x = c1rt(:,1:100);
            else 
                init_x = c2rt(:,1:100);
            end
            
            % Defining the parameters of the obstacle based on its type
            if obs_type == 1
                lengtho = 3;
                widtho = 1.5;
                heighto = 3*reference_velocity;         % Obs PF height is based on the vehicle's speed
            elseif obs_type == 4
                lengtho = 1.5;
                widtho = 1.5;
                heighto = 3*reference_velocity;
            else
                lengtho = 0.1;
                widtho = 0.1;
                heighto = 0.1;
            end
            
            % Defining all the optimisation parameters
            opti = obj.optimizer;
            opti.set_value(obj.ref_v, reference_velocity);
            opti.set_value(obj.states, st);
            opti.set_value(obj.coeffr, coeff_right);
            opti.set_value(obj.coeffl, coeff_left);
            opti.set_value(obj.obs, obsrt);
            opti.set_value(obj.obs_param, [heighto, lengtho, widtho]);
            opti.set_value(obj.rl, rlrt(:,1:100));
            opti.set_value(obj.ll, llrt(:,1:100));
            opti.set_value(obj.ml, mlrt(:,1:100));
            opti.set_value(obj.Wv, 100);
            opti.set_value(obj.Wvv, 50);
            opti.set_value(obj.Wt, 10);
            opti.set_value(obj.Ws, 10);
            opti.set_value(obj.WF, 3000);
            opti.set_value(obj.WFo, 10000);
            opti.set_value(obj.Msr, 15);
            
            % Providing an initial guess for the optimisation
            opti.set_initial(obj.x, [init_x; reference_velocity*ones(1,obj.N); h_ref(1:100)]);            

            % Chosing the solver and performing the optimization
            opti.minimize(obj.cost);
            options.ipopt.print_level = 0;
            opti.solver('ipopt',options);
            sol = opti.solve();
            ref = sol.value(obj.x);
            
            % Fitting the references with a spline and interpolating to
            % obtain 400 points
            L = 15.96;
            t = linspace(0,L,length(ref(1,:)));
            
            theta = ref(4,:)'-th;

            tq = 0:obj.Ts:L;

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

            % Saving reference for next time step
            obj.old_reference = reference;
        end

        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end