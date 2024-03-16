function batch_LS_odom_range(N)
    
    scenario_setup

    mean_error = zeros(n_states);    

    for i = 1:N
        a1 = ones(n_states,1);
        A_init = diag(a1);      
        A1 = A_init;                                            % 1001 x 1001 matrix of zeros and ones on the diagonal
        
        for j = 1:n_states-1                                  % diagonal -1s underneath diagonal 1s
            A1(j+1,j) = -1;
        end
        
        n = n_states + n_marks + 100*n_marks; % odometry readings and range readings for each landmark
        m = n_states + n_marks; % all system states and landmark states
        
        A2 = zeros(n,m);
        A2(1:n_states, 1:n_states) = A1;
        l_s = ones(n_read+1,1)*-1;                            
        l_sensed = diag(l_s);                       % diagonal -1s for rows when landmarks are sensed
        
        n_mark_states = (n_read+1)*3;       % 50 readings to each side of each landmark, plus each static landmark state
        l_read = zeros(n_mark_states , n_marks);                      % distance to each individual landmark over t
    
        for j = 1:n_marks         % while in range, sensor takes 50 readings while approaching and leaving each landmark
            A2(n_states+j+n_read*(j-1):n_states+j+n_read*j, n_states+j) = 1; % columns for landmark sensing
            A2(n_states+j+n_read*(j-1):n_states+j+n_read*j, l_left(j):l_right(j)) = l_sensed; % diagonal matrices for landmark sensing
        end
        
        l_dist = zeros(n_states,1);    % initialize matrix for sensor readings
    
        for j = 1:n_states
            range_noise = stdev_range*randn; % range measurement corruption
            odom_noise = randn(1,n_states)*stdev_odom;       % sensor noise corrupting odometry measurement
            
            b_vector = (v_vector + odom_noise)*delta_t;             % delta x sensed by odometer
            b_vector(1) = x_initial;                                % initial position at origin is known
            b_vector = transpose(b_vector);                         % know that the error at initial condition is zero
    
            l1 = (j >= l_left(1) && j <= l_right(1));   % indexes of states within range of a landmark
            l2 = (j >= l_left(2) && j <= l_right(2));
            l3 = (j >= l_left(3) && j <= l_right(3));
           
            if  l1                        %if landmark within range, take sensor measurement
                l_dist(j,1) = landmarks(1) - x_vector(j) + range_noise;   % landmark x - robot x = landmark x - robot x + range error
                l_read(j,1) = 1;
            elseif l2
                l_dist(j,1) = landmarks(2) - x_vector(j) + range_noise;
                l_read(j,2) = 1;
            elseif l3
                l_dist(j,1) = landmarks(3) - x_vector(j) + range_noise;
                l_read(j,3) = 1;
            end 
        end
        
        l_dist = nonzeros(l_dist);        % remove zero elements when sensor is on but no landmark is present
        lb_vector = [b_vector; l_dist];
        
        ATA = transpose(A2)*A2;
        ATB = transpose(A2)*lb_vector;
        x_sol2 = ATA\ATB;
        x_sol2 = transpose(x_sol2); % least squares estimate
    
        abs_error2 = abs(x_vector - x_sol2(1,1:n_states)); % row vector for absolute error for each trial
        mean_error(i+1,1:n_states) = abs_error2; % mean average per time step 
    end
    me = mean(mean_error);

    figure
    plot(t_vector, me); % absolute position error over time
    legend('Mean Absolute Error (m)')
    title('Least Squares Mean Absolute Error (Odometry & Range Sensing)')
    xlabel('Time (s)')
    ylabel('Absolute Position Error (m)')
end