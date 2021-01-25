clc; clear;

load animation_wavefront
figure;
simulate(base_of_robot', obstacle_center', obstacle_radius, start_point', end_point', [L1_length; L2_length], pos_list(1:1:end, :));
pause(2.0)
figure;
simulate_cspace(cspace,start_ik, end_ik, obstacle_center, obstacle_radius,pos_list(1:1:end, 2), pos_list(1:1:end, 1), increment)
