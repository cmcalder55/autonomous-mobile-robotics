beep off

generate_obstacles % create the robot workspace

N = 100;

all_paths = zeros(N,2); 
lengths = zeros(2,N);
vars = zeros(N,1);

p_kf = zeros(N,2*N);

for i = 1:N             % for N trials, find RRT for shortest path to goal region
    
    [path, path_length] = RRT(start_state, goal_region, obstacles); 
    
    n_nodes = length(path); % find number of steps travelled

    p_final = propagate_KF_path(path,obstacles);

    if i == 1                               % special condition for first path 
        all_paths(1:n_nodes,i:i+1) = path; 
        p_kf(1:n_nodes,i:i+1) = p_final;
    end

    all_paths(1:n_nodes,2*i:2*i+1) = path; % robot poses over the shortest path for each trial

    lengths(1,i) = path_length; % find total lengths and number of of nodes for each trial's path
    lengths(2,i) = n_nodes;


    p_kf(1:n_nodes,2*i:2*i+1)= p_final; % covariance data at each node per path
    vars(i,1) = sum(p_kf(n_nodes,2*i:2*i+1));
end

% shortest path
[~,dim] = min(lengths(1,:)); % length of shortest path
n_last = lengths(2,dim);      % number of nodes inside the shortest path

x_short = 2*dim;
y_short = x_short + 1;

p_short = all_paths(:,x_short:y_short); % path of shortest distance

radii = sqrt(p_kf(:,x_short:y_short)); %P_k Ellipse Value Calculation

plot(p_short(1:n_last,1),p_short(1:n_last,2),'.-k','MarkerSize',20); % plot nodes and edges

for i = 1:n_last                        % plot ellipse for each path node
    uncertainty = ellipse(radii(i,1), radii(i,2), 0, p_short(i,1), p_short(i,2),'r');
end


% max uncertainty
[~,max_unc] = max(sum(p_kf)); %Max Value of Uncertainty

% figure
% p_max_unc = all_paths(:,2*max_unc:2*max_unc+1); %path of max uncertainty
% plot(p_max_unc(1:n_last,1),p_max_unc(1:n_last,2),'.-k','MarkerSize',20); % plot nodes and edges
% 
% for i = 1:n_last                        % plot ellipse for each path node
%     h = ellipse(radii(i,1),radii(i,2),0,p_max_unc(i,1),p_max_unc(i,2),'r');
% end

% min uncertainty
[~,min_unc] = min(sum(p_kf)); %Max Value of Uncertainty

% figure
% p_min_unc = all_paths(:,2*min_unc:2*max_unc+1); %path of max uncertainty
% plot(p_min_unc(1:n_last,1),p_min_unc(1:n_last,2),'.-k','MarkerSize',20); % plot nodes and edges

% for i = 1:n_last                        % plot ellipse for each path node
%     h = ellipse(radii(i,1),radii(i,2),0,p_min_unc(i,1),p_min_unc(i,2),'r');
% end
