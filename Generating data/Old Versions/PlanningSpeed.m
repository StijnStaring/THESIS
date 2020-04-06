classdef PlanningSpeed < matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime


    properties (Access = private)
        N = 10                                  % Length of the MPC horizon
        optimizer                               % Opti structure, necessary for casadi optimisation
        cost                                    % Cost function for MPC
        v                                       % Car states
        Ts = 0.01                               % Sampling time
    
        ref_v                                   % Velocity reference
        v0
        curv = zeros(1,25)
        mu 
        ay_lim
        Wv
        Wacc 
        vmax = 40
        vlim
    end

    methods (Access = protected)
        
        function num = getNumInputsImpl(~)
            num = 3;
        end
        function num = getNumOutputsImpl(~)
            num = 1;
        end
        function [dt1] = getOutputDataTypeImpl(~)
        	dt1 = 'double';
        end
        function [dt1, dt2, dt3] = getInputDataTypeImpl(~)
        	dt1 = 'double';
            dt2 = 'double';
            dt3 = 'double';
        end
        function [sz1] = getOutputSizeImpl(~)
            sz1 = [1,1];
        end
        function [sz1, sz2, sz3] = getInputSizeImpl(~)
        	sz1 = [1,3];
            sz2 = [1,1];
            sz3 = [3,10000];           
        end
        function [cp1, cp2, cp3] = isInputComplexImpl(~)
        	cp1 = false;
            cp2 = false;
            cp3 = false;
        end
        function [cp1] = isOutputComplexImpl(~)
        	cp1 = false;
        end
        function [fz1, fz2, fz3] = isInputFixedSizeImpl(~)
        	fz1 = true;
            fz2 = true;
            fz3 = true;
        end
        function [fz1] = isOutputFixedSizeImpl(~)
        	fz1 = true;
        end
        function setupImpl(obj,~,~,~,~,~,~,~)
            
            import casadi.*          
            
            % MPC problem setup
            opti = casadi.Opti();                       % Creating the opti structure
            obj.v = opti.variable(1,obj.N);             % State matrix across the whole horizon
            
            obj.ref_v = opti.parameter();               
            obj.v0 = opti.parameter();           
            obj.mu = opti.parameter();         
            obj.ay_lim = opti.parameter();          
            obj.Wv = opti.parameter(); 
            obj.Wacc = opti.parameter(); 
            obj.vlim = opti.parameter(1,obj.N);
            
            J = 0;
            opti.subject_to(obj.v(1) - obj.v0 == 0);              % Set initial states equal to current car states
            J = J + obj.Wacc*(obj.v(2)-obj.v(1))^2;
            opti.subject_to( -1 <= obj.v(2)-obj.v(1) <= 1 );
            
            for i = 2:obj.N-1
                                
                J = J + obj.Wv*(obj.v(i)-obj.ref_v)^2;         
                J = J + obj.Wacc*(obj.v(i+1)-obj.v(i))^2; 
                opti.subject_to( -1 <= obj.v(i+1)-obj.v(i) <= 1 );
                opti.subject_to( obj.v(i) <= obj.vlim(i) );
                opti.subject_to( obj.v(i) <= obj.vlim(i+1) );
%                 opti.subject_to( obj.v(i) <= obj.vlim(i+2) );
            end
            
%             J = J + obj.Wv*(obj.v(obj.N-1)-obj.ref_v)^2;         
%             J = J + obj.Wacc*(obj.v(obj.N)-obj.v(obj.N-1))^2; 
%             opti.subject_to( obj.v(obj.N-1) <= obj.vlim(obj.N-1) );
%             opti.subject_to( obj.v(obj.N-1) <= obj.vlim(obj.N) );
            
            J = J + obj.Wv*(obj.v(obj.N)-obj.ref_v)^2;
            opti.subject_to( obj.v(obj.N) <= obj.vlim(obj.N) ); 
   
            obj.optimizer = opti();
            obj.cost = J;
        end

        function [v_ref_out] = stepImpl(obj, current_states, reference_velocity, centerline1)         
            

            % Computing distance to all points on right centerline
            dist_to_c1 = (centerline1(1,:) - current_states(1)).^2 + (centerline1(2,:) - current_states(2)).^2;
            
            % Finding index of closest point on right centerline
            [~, I] = min(dist_to_c1);
  
            legth = 4000;
            step = legth/obj.N;
   
            if (I + legth > length(centerline1(1,:)))
                center1 = [centerline1(1:2, I: step: end) centerline1(1:2, 1: step: I+legth-length(centerline1(1,:))-2)];              
            else
                center1 = centerline1(1:2, I: step: I+legth-1);              
            end
            
            curvatures = LineCurvature2D(center1(1:2,:)');
            
            v_lim = 40*ones(1,obj.N);
            
            for i = 1:obj.N
                if abs(curvatures(i)) < 1e-5
                    continue
                end   
                v_lim(i) = sqrt(0.7*9.81/abs(curvatures(i)));
                
                if v_lim(i) > 40
                    v_lim(i) = 40;
                end
            end
           
             
            % Defining all the optimisation parameters
            opti = obj.optimizer;
            opti.set_value(obj.ref_v, reference_velocity);
            opti.set_value(obj.v0, current_states(3));
            opti.set_value(obj.Wv, 10);
            opti.set_value(obj.Wacc, 500);
            opti.set_value(obj.mu, 1);
            opti.set_value(obj.ay_lim, 0.7*9.81);
            opti.set_value(obj.vlim, v_lim);
            
            
            % Providing an initial guess for the optimisation
            opti.set_initial(obj.v, reference_velocity*ones(1,obj.N));            

            % Chosing the solver and performing the optimization
            opti.minimize(obj.cost);
            options.ipopt.print_level = 0;
            opti.solver('ipopt',options);
            try
                sol = opti.solve();
                ref = sol.value(obj.v);
            catch 
                ref = opti.debug.value(obj.v);
            end
            
            % Interpolating velocity vector
%             s = 1:step:legth;
%             ss = 1:legth;
%             p = polyfit(s,ref,2);
%             v_ref_out_total = polyval(p,ss);
%             v_ref_out = v_ref_out_total(4);

            v_ref_out = ref(2);
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