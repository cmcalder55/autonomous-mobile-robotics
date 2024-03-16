
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Environmental variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_initial = 0;     % known initial state at origin, uncorrupted measurement
length_hallway = 10;
sensor_range = 0.5;

landmarks = [2; 5; 8];

num_landmarks = length(landmarks);
n_marks = length(landmarks); % number of landmark nodes

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot's "Ground Truth" Trajectory when it travels the hallway 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

delta_t = 0.1; % time is discretized with a time-step of 0.1 seconds
v_robot = 0.1; % robot travels at a constant speed of 0.1 m/s

t_terminal = length_hallway/v_robot; % time that we reach the end of hall

t_vector = 0:delta_t:t_terminal; % time vector for robot's trajectory
x_vector = 0:v_robot*delta_t:length_hallway; % robot's true position in time
v_vector = ones(1,length(t_vector)).*v_robot;  % robot's true velocity in time

n_states = length(x_vector); % number of discrete-time states in trajectory

delta_x = length_hallway/(n_states-1); % distance change per time step

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sensor variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
stdev_odom = 0.1; % m/s
stdev_range = 0.01; % m

% range sensor
n_range = sensor_range/delta_x;              % number of readings taken for the range sensor over its range
n_read = n_range*2;                         % number of range readings to left and right of landmarks
readings = n_read+1;

[yn, l_right] = ismembertol(landmarks, x_vector);
l_left = l_right - n_range;
l_right = l_right + n_range;



