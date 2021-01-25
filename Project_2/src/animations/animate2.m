clc; clear;

load animation2
figure;
simulate(base_of_robot', obstacle_center', obstacle_radius, start_point', end_point', [L1_length; L2_length], pos_list(1:50:end, :));
figure;
simulate_cspace(cspace,start_ik, end_ik, obstacle_center, obstacle_radius,pos_list(1:100:end, 2), pos_list(1:100:end, 1), increment)
