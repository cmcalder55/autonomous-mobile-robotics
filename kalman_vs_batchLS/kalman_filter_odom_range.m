function kalman_filter_odom_range(N)
    scenario_setup

    mean_error = zeros(N,n_states);
    
    dev = 1e-4;
    stdev_kal4 = zeros(1,N);
    stdev_kal4(1,1) = dev;
    
    Q = zeros(5,5);
    Q(1,1) = 10^-8;

    a4 = ones(5,1);
    a4 = diag(a4);      
    A4 = a4;                            % 1001 x 1001 matrix of zeros and ones on the diagonal
    A4(2,1) = delta_t;
    
    P = [1,dev,1,1,1];
    
    I = eye(5);
    
    x_actual = zeros(5,n_states); %for z_k
    x_actual(1,:)= v_vector;
    x_actual(2,:)= x_vector;
    
    % For N trials
    for i = 1:N
        % Inital prediction/guess
        x_k = [v_robot;0;0;0;0]; 
        P_k = diag(P);

            for j = 1:n_states-1

                range_noise = stdev_range*randn;
                odom_noise = stdev_odom*randn;
                noise = [odom_noise; range_noise];
                
                R = stdev_odom^2;
                H = [1 0 0 0 0];
                
                l1 = (j >= l_left(1) && j <= l_right(1));   % states within range of a landmark
                l2 = (j >= l_left(2) && j <= l_right(2));
                l3 = (j >= l_left(3) && j <= l_right(3)); 

                if  l1 || l2 || l3
                    R = [stdev_odom^2 0;0 stdev_range^2];
                    H = [1 0 0 0 0; 0 -1 0 0 0];
                    z_k = H * x_actual(:,j)+noise;
                    if  l1
                        H(2,3) = 1;
                    elseif l2
                        H(2,4) = 1;
                    elseif l3
                        H(2,5) = 1;
                    end
                else 
                    z_k = H * x_actual(:,j)+noise(1);   % state estimate with odometry only
                end     
           
            % ----- process model ------
            % time update; "predict"
            % (1) project the state ahead
            x_kp = A4*x_k;                   % B*u_k = 0       
            
            % (2) project the error covariance ahead
            P_kp = A4*P_k*A4.' + Q;
    
            % ----- measurement model ----- 
            % measurement update; "correct"
            % (1) compute the gain; solve for k to minimize p
            K_k = P_kp*H.'*(H*P_kp*H.'+R)^-1; 
            
            % (2) update state estimate with z_k
            x_k = x_kp+K_k*(z_k-H*x_kp);  
            
            % (3) update error covariance
            P_k = (I-K_k*H)*P_kp;
    
            % --------- data store for each time step ----------------
            abs_error4 = abs(x_vector(1,j+1)-x_k(2));
            mean_error(i,j+1) = abs_error4;
    
            stdev_kal4(1,j+1) = sqrt(P_k(2,2));
            end
    end
    me = mean(mean_error);

    figure
    hold on
    plot(t_vector,me)
    plot(t_vector,stdev_kal4)
    legend('Mean Absolute Error (m)','Standard Deviation')
    title('Kalman Filter Mean Absolute Error (Odometry and Range Sensing)')
    xlabel('Time (s)')
    ylabel('Absolute Position Error (m)')
    hold off
end