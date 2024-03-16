beep off    % Turn alert beep off
close all   % Close any open figures, clear workspace and command window
clear
clc

N = 1000; % number of trials

% Function calls for each scenario
batch_LS_odom(N);
batch_LS_odom_range(N);
kalman_filter_odom(N);
kalman_filter_odom_range(N);
batch_LS_odom_range_rev(N);









