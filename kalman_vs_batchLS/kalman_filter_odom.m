function kalman_filter_odom(N)
    
    scenario_setup

    mean_error = zeros(N,n_states);

    dev = 1e-4;
    stdev_kal3 = zeros(1,N);
    stdev_kal3(1,1) = dev;
    
    A = [1 0; delta_t 1];
    
    Q = zeros(2,2);
    Q(1,1) = 10^-8;
    
    P = [1,dev];
    H = [1 0];
    R = stdev_odom^2;
    
    I = eye(2);
    
    x_true = zeros(5,n_states); %for z_k
    x_true(1,:)= v_vector;
    x_true(2,:)= x_vector;
    
    % For N trials
    for i = 1:N
        % Inital prediction/guess
        x_k = [v_robot;0]; 
        P_k = diag(P);

            for j = 1:n_states-1
            % ----- process model ------
            % time update; "predict"
            % (1) project the state ahead
            x_kp = A*x_k;                   % B*u_k = 0       
            
            % (2) project the error covariance ahead
            P_kp = A*P_k*A.' + Q;
    
            % ----- measurement model ----- 
            % measurement update; "correct"
            z_k = H * x_true(1:2,j)+stdev_odom*randn;
   
            % (1) compute the gain; solve for k to minimize p
            K_k = P_kp*H.'*(H*P_kp*H.'+R)^-1; 
            
            % (2) update state estimate with z_k
            x_k = x_kp+K_k*(z_k-H*x_kp);  
            
            % (3) update error covariance
            P_k = (I-K_k*H)*P_kp;
    
            % --------- data store for each time step ----------------
            abs_error3 = abs(x_vector(1,j+1)-x_k(2));
            mean_error(i,j+1) = abs_error3;
    
            stdev_kal3(1,j+1) = sqrt(P_k(2,2));
            end
    end
    me = mean(mean_error);

    figure
    hold on
    plot(t_vector,me)
    plot(t_vector,stdev_kal3)
    legend('Mean Absolute Error (m)','Standard Deviation')
    title('Kalman Filter Mean Absolute Error (Odometry Sensing Only)')
    xlabel('Time (s)')
    ylabel('Absolute Position Error (m)')
    hold off
end