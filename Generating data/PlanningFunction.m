function [Xref] = PlanningFunction(current_states, reference_velocity)
    
    global Planner centerline1 centerline2 left_line right_line middle_line
        
    reference_velocity = reference_velocity/3.6;
    
    N = 50;
    T = 50*0.04;
    
    % Computing distance to all points on right centerline
    dist_to_c1 = (centerline1(1,:) - current_states(1)).^2 + (centerline1(2,:) - current_states(2)).^2;

    % Finding index of closest point on right centerline
    [dc1, I] = min(dist_to_c1);

    % Computing distance to all points on left centerline
    dist_to_c2 = (centerline2(1,:) - current_states(1)).^2 + (centerline2(2,:) - current_states(2)).^2;

    % Finding index of closest point on left centerline
    [dc2, ~] = min(dist_to_c2);

    center1 = zeros(3,N);
    center2 = zeros(2,N);
    rline = zeros(2,N);
    lline = zeros(2,N);
    mline = zeros(2,N);
    ref_vel = zeros(1,N);

    index = I;
    
    % Get the points of the road lane. The road lane points are
    % obtained offline in Prescan at constant speed of 5 m/s speed and
    % at 100Hz.
    for i = 1:N

        if index > length(centerline1(1,:))
            index = 1;
        end

        center1(1:2,i) = centerline1(1:2,index);
        center1(3,i) = centerline1(3,index);
        center2(1:2,i) = centerline2(1:2,index);
        rline(1:2,i) = right_line(1:2,index);
        lline(1:2,i) = left_line(1:2,index);
        mline(1:2,i) = middle_line(1:2,index);

        % See curvature ahead (100m) to set the reference velocity
        % accordingly
        if index + 2000 > length(centerline1(1,:))
            remain = 2000 - (length(centerline1(1,:)) - index);
            curvature1 = LineCurvature2D(centerline1(1:2,index:50:end)');
            curvature2 = LineCurvature2D(centerline1(1:2,1:50:remain)');
            curvature = [curvature1 ; curvature2];
        else
            curvature = LineCurvature2D(centerline1(1:2,index:50:index+2000)');
        end

        if abs(curvature(1)) < 0.01
            ref_vel(i) = reference_velocity;
        else
            ref_vel(i) = min([min(sqrt(0.3*8.81./abs(curvature))), reference_velocity]);
        end

        % Compute the step for the next road line point to follow
        % based on the reference velocity
        step = round(ref_vel(i)/5)*4;
        index = index + step;
    end

    % Transformation to the local vehicle reference frame
    th = current_states(4);
    ROT = [cos(th) sin(th);                
            -sin(th) cos(th)];
    pos = [current_states(1); current_states(2)];
    pos_rot = ROT*pos;

    c1r = zeros(3,length(center1)); c1rt = zeros(3,length(center2));
    c2r = zeros(2,length(center2)); rlr = zeros(2,length(rline)); 
    llr = zeros(2,length(rline)); mlr = zeros(2,length(rline));

    for i = 1:length(center1)
        c1r(1:2,i) = ROT*center1(1:2,i);
        c2r(:,i) = ROT*center2(:,i);
        rlr(:,i) = ROT * rline(:,i);
        llr(:,i) = ROT * lline(:,i);
        mlr(:,i) = ROT * mline(:,i);
    end 

    c1rt(1:2,:) = c1r(1:2,:) - pos_rot;
    c2rt = c2r - pos_rot;
    rlrt = rlr - pos_rot;
    llrt = llr - pos_rot;
    mlrt = mlr - pos_rot;

    c1rt(3,:) = center1(3,:) - th;

    % Polynomial fitting of both centerlines
    coeff_right = polyfit(c1rt(1,:),c1rt(2,:),5);   
    coeff_left = polyfit(c2rt(1,:),c2rt(2,:),5);

    st = [0; 0; current_states(3); 0];

    % Initial guess for the optimization is chosen based on which
    % centerline is closer
    if dc1<dc2
        init_x = c1rt(1:2,1:N);
    else 
        init_x = c2rt(:,1:N);
    end
    
    initial_guess = [init_x; ref_vel(1:N); zeros(1,N)];
    
    % Solving the optimization problem by calling the CasADi function,
    % previously implemented and loaded as a global variable
    sol = Planner(st, ref_vel, rlrt, mlrt, llrt, c1r, coeff_right, coeff_left, initial_guess);
    
    ref = full(sol);
    
    % Rotating and translating the reference back to global
    % coordinates
    reference = zeros(4,N);

    for i = 1:N
        reference(1:2,i) = ROT\ref(1:2,i) + pos;
    end 

    reference(4,:) = ref(4,1:N) + th;    
    reference(3,:) = ref(3,1:N);
    
    % Reference state trajectory: 50 poses with sampling time Ts = 0.04
    Xref = reference;
end