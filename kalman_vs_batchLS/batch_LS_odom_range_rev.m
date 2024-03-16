function batch_LS_odom_range_rev(N)
    scenario_setup

    t_new = t_vector+100;
    t_new = [t_vector t_new(1,2:end)];
    
    x_flip = flip(x_vector(1:end));
    x_new = [x_vector x_flip];           % flip x vector for driving back down hallway
                                         % combine x vector to get robot's pose for the entire trip
    
    m = 2*(n_states+(n_marks*n_read)+n_marks);          % sensor observations
    n = 2*n_states+n_marks;                         % each discrete time state and static landmark states
    
    mean_error = zeros(N,2*n_states+n_marks); 
    
    A = zeros(m,n);
    b_vector = zeros(m,1); 
    
    % when landmarks are in range of sensor on way back down the hallway;
    % loop closure points
    l1_x_rev = n_states - n_read*landmarks(1)+n_range; 
    l2_x_rev = l1_x_rev - 2*n_read+ 1; 
    l3_x_rev = l2_x_rev - 2*n_read +1;
    
    l1_fwd_s = 2*n_states;              % m sensor observation indexes for landmarks driving towards end of hallway
    l1_fwd_e = l1_fwd_s + n_read;
    l2_fwd_s = l1_fwd_e + readings+1;
    l2_fwd_e = l2_fwd_s + n_read;
    l3_fwd_s = l2_fwd_e + readings+1;
    l3_fwd_e = l3_fwd_s + n_read;
    
    l1_rev_s = 2*n_states+readings;     % m sensor observation indexes for landmarks driving back to origin
    l1_rev_e = l1_rev_s + n_read;
    l2_rev_s = l2_fwd_e + 1;
    l2_rev_e = l2_rev_s + n_read;
    l3_rev_s = l3_fwd_e + 1;
    l3_rev_e = l3_rev_s + n_read;
    
    for j = 1:2*n_states
        if j == 1 || j == 2*n_states    % known initial condition, then reobserved
            A(j,j) = 1;
        elseif j < n_states         % driving towards end of hallway
            A(j,j) = 1;
            A(j+1,j) = -1;
        else                        % driving back to start
           A(j,j) = -1;
           A(j,j-1) = 1; 
        end
    end
    
    % range sensor readings for landmarks while driving to the end of 
    % the hallway and back
    % ----------- forward landmarks
    for j = l1_fwd_s:l1_fwd_e               % when within the senors' range for the landmark
        A(j,j-n_states-l1_x_rev+1) = -1;    % update A matrix for the 50 readings taken when approaching 
                                            % and departing from each landmark
    end
    
    for j = l2_fwd_s:l2_fwd_e
        A(j,j-n_states-l2_x_rev-n_read) = -1;
    end
    
    for j = l3_fwd_s:l3_fwd_e 
        A(j,j-n_states-l3_x_rev-2*n_read-1) = -1;
    end
    % ----------- reverse landmarks
    for j = l1_rev_s:l1_rev_e 
        A(j,j-l3_x_rev+readings) = 1;           % 1 instead of -1 since driving -0.1 m/s, negative
                                                % motion relative to the origin/re-approaching start
    end
    
    for j = l2_rev_s:l2_rev_e
        A(j,j-l2_x_rev-readings*2) = 1;
    end
    
    for j = l3_rev_s:l3_rev_e
        A(j,j-l1_x_rev-readings*5) = 1;
    end
     % ----------- range sensor readings      
    for j = 2*n_states:m-1            
        if j < 2*n_states+n_read+1
            A(j, n-2)=1;
        elseif j < 2*n_states+ 2*(n_read+1)
            A(j,n-2)=-1;
        elseif j < 2*n_states+ 3*(n_read+1)
            A(j,n-1)=1;
        elseif j < 2*n_states+ 4*(n_read+1)
            A(j,n-1)=-1;
        elseif j < 2*n_states+ 5*(n_read+1)
            A(j,n)=1;
        else
            A(j,n)=-1;
        end
    end
    
    A_pinv = pinv(A);
    
    for i = 1:N       % ------------- N trials
         
        for j = 1:2*n_states-1              % b vector for odometry readings
            b_vector(1,1) = 0;
            b_vector(j,1) = (v_robot + stdev_odom*randn)*delta_t; 
        end
    
        for j = 2*n_states:m-1              % b vector for range sensor readings
                
            range_noise = stdev_range*randn; % range sensor measurement noise
            
            % if in range of a landmark, take range sensor reading and populate
            % b vector with observation
            if j >= l1_fwd_s && j <= l1_fwd_e 
                b_vector(j,1) = landmarks(1)-x_new(j-n_states-l1_x_rev)+range_noise;
    
            elseif j >= l2_fwd_s && j <= l2_fwd_e 
                b_vector(j,1) = landmarks(2)-x_new(j-n_states-l2_x_rev-n_read)+range_noise;
    
            elseif j >= l3_fwd_s && j <= l3_fwd_e 
                b_vector(j,1) = landmarks(3)-x_new(j-n_states-l3_x_rev-2*n_read-1)+range_noise;
            
            % reobserving landmarks in reverse
            elseif j >= l1_rev_s && j <= l1_rev_e 
                b_vector(j,1) = x_new(j-l3_x_rev+readings)-landmarks(1)+range_noise;
    
            elseif j >= l2_rev_s && j <= l2_rev_e 
                b_vector(j,1) = x_new(j-l2_x_rev-readings*2)-landmarks(2)+range_noise;
    
            else 
                b_vector(j,1) = x_new(j-l1_x_rev-readings*5)-landmarks(3)+range_noise;
            end
        end
     
        x_sol5 = A_pinv*b_vector; 
        x_land = [x_new,landmarks']; % doubled x vector + static landmarks
        abs_error = x_sol5 - x_land.'; 
        mean_error(i,:) = abs(abs_error);
    end
    
    me = mean(mean_error); %Mean Absolute Error for Each Time Step
    
    % plot # 5
    plot(t_new,me(1:2*n_states-1))
    legend('Mean Absolute Error (m)')
    title('Least Squares Mean Absolute Error (Odometry & Range Sensing)')
    xlabel('Time (s)')
    ylabel('Absolute Position Error (m)')
end