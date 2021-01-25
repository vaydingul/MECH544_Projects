clear all force;

%% Constants

% 35 - 35
% 40 - 40 - base = 30 -0, start = 30 - 80
% 40 - 40 - base = 30 -0, start = 30 - 80, end = 50 - 40
%
increment = 3;
L1_length = 38;
L2_length = 38;
obstacle_radius = 10;
obstacle_center = [40, 60];
base_of_robot = [20, 0];
start_point = [20, 70];
end_point = [60, 40]; %60 40

%% C-space construction
[cspace, start_ik, end_ik] = generate_cspace(increment, L1_length, L2_length, obstacle_radius, obstacle_center, base_of_robot, start_point, end_point);
%
% meshc(cspace) % Please uncomment for visualization
% xticklabels(gca, (xticks(gca)-1) * increment + 1)
% yticklabels(gca, -180.0 + (yticks(gca)-1) * increment + 1)
% title("Configuration Space")
% xlabel("\alpha")
% ylabel("\beta")
%% Potantial field construction
[P] = extract_potential(cspace, end_ik, increment);
%
% figure;
% meshc(P) % Please uncomment for visualization
% xticklabels(gca, (xticks(gca)-1) * increment + 1)
% yticklabels(gca, -180.0 + (yticks(gca)-1) * increment + 1)
% title("Potential Field")
% xlabel("\alpha")
% ylabel("\beta")
% zlabel("Potential value")
%
figure;
hold on;
contour(P) % Please uncomment for visualization
plot((end_ik(2) )/increment + 1, (end_ik(1) + 180.0 )/increment + 1, "ro", 'LineWidth', 4)
text((end_ik(2) )/increment + 1 + 1, (end_ik(1) + 180.0 )/increment + 1 + 1, "End Point")
plot((start_ik(2) )/increment + 1, (start_ik(1) + 180.0 )/increment + 1, "g^", 'LineWidth', 4)
text((start_ik(2) )/increment + 1 + 1, (start_ik(1) + 180.0 )/increment + 1 + 1, "Start Point")

xticklabels(gca, (xticks(gca)-1) * increment + 1)
yticklabels(gca, -180.0 + (yticks(gca)-1) * increment + 1)
title("Potential Field Contour")
xlabel("\alpha")
ylabel("\beta")

%% Motion planning via Potential Field method
[pos_list, F] = motion_plan(cspace, start_ik, end_ik, increment);
% Since, we are using the (beta, alpha) convention in above calculations, we
% switch columns during the animation. It is due to our convention confusion.
% Not important detail :)
pos_list = [pos_list(:, 2), pos_list(:, 1)];
beep; % Beep when is done :p

%% Simulation routine for C-space
simulate_cspace(cspace, start_ik, end_ik, obstacle_center, obstacle_radius, pos_list(1:100:end, 2), pos_list(1:100:end, 1), increment)

%% Simulation routine for motion planning

simulate(base_of_robot', obstacle_center', obstacle_radius, start_point', end_point', [L1_length; L2_length], pos_list(1:50:end, :));
