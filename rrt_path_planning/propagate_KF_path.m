
% This function will propagate the KF across the entire path generated by  
% RRT, given the path and obstacles as input. Internally, this function 
% will call propagate_KF to perform every measurement update. It will 
% return the error covariance matrix at the terminal state of the path,  
% which will be used to assess the path s localization uncertainty.

function p_final = propagate_KF_path(path,obstacles)

dt = 1;                 % time step
range = 5;              % sensor range
dims = length(path);    

A = eye(4);
A(1,2) = dt;
A(3,4) = dt;

Q = eye(4);

p_kf = eye(4);
p_final = zeros(dims,2);

R = eye(2);

%For Loop for entire path step
for i =dims:-1:1 

    path_x = path(i,1);
    path_y = path(i,2);
    
    % check for obstacles using 4 range sensors in +/- x and y directions
    x_fwd = collision_check_segment(path_x,path_y,path_x+range,path_y,obstacles); %x direction  forward check
    x_back = collision_check_segment(path_x,path_y,path_x-range,path_y,obstacles); %x direction  backwards check 
    x_check = x_fwd + x_back;        % number of x axis sensors triggered
    
    y_fwd = collision_check_segment(path_x,path_y,path_x,path_y+range,obstacles); %y direction  up check
    y_back = collision_check_segment(path_x,path_y,path_x,path_y-range,obstacles); %y direction  down check 
    y_check = y_fwd + y_back;       % number of x axis sensors triggered
    
    multi_obs = x_check + y_check; % range sensors triggered on both axes

    H_k = [0 1 0 0; 0 0 0 1];               % initialize observability matrix for odometry measurements only
    
    if x_check > 0                          % range sensor active in x
        H_k(1,4) = 1;
        H_k(2,:) = [1 0 0 0];

    elseif y_check > 0                       % range sensor active in y
        H_k(1,4) = 1;
        H_k(2,:) = [0 0 1 0];

    elseif multi_obs > 2                 % range sensor active in x and y
        H_k(1,4) = 1;
        H_k(2,:) = [1 0 1 0];

    end

    p_kf = ((A*p_kf*A.' + Q)^-1 +H_k.'*R^-1*H_k)^-1; % propegate linear filter over one time-step
    
    p_final(i,1) = p_kf(1);        % x error covariance for ellipse radius 1
    p_final(i,2) = p_kf(3,3);      % y error covariance for ellipse radius 2

end