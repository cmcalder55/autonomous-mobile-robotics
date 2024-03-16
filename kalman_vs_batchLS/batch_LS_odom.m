function batch_LS_odom(N)
    scenario_setup

    % batch least squares, odometry sensing only
    mean_error = zeros(n_states);
    
    for i = 1:N
        
        odom_noise = randn(1,n_states).*stdev_odom;           % sensor noise corrupting odometry measurement
        b_vector = (v_vector + odom_noise)*delta_t;                 % delta x sensed by odometer
        b_vector(1) = x_initial;                                    % initial position at origin is known; 0
        b_vector = transpose(b_vector);                            
        
        a1 = ones(n_states,1);
        A_init = diag(a1);      
        A1 = A_init;                            % 1001 x 1001 matrix of zeros and ones on the diagonal
        
        for j = 1:n_states-1
            A1(j+1,j) = -1;
        end
       
        x_sol1 = A1\b_vector;
        x_sol1 = transpose(x_sol1);             % least squares estimate
      
        abs_error1 = abs(x_vector - x_sol1);    % row vector for absolute error per trial
        mean_error(i+1,:) = abs_error1;        % mean average per time step 
    end
    
    me = mean(mean_error);                    % mean absolute error

    figure
    plot(t_vector, me);
    legend('Mean Absolute Error (m)')
    title('Least Squares Mean Absolute Error (Odometry Sensing Only)')
    xlabel('Time (s)')
    ylabel('Absolute Position Error (m)')
end