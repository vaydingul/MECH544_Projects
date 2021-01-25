clear all force;

%% Constants
increment = 1;
L1_length = 38;
L2_length = 38;
obstacle_radius = 10;
obstacle_center = [40, 60];
base_of_robot = [20, 0];
start_point = [20, 70];
end_point = [60, 40]; %60 40

%% C-space construction
[cspace, start_ik, end_ik] = generate_cspace(increment, L1_length, L2_length, obstacle_radius, obstacle_center, base_of_robot, start_point, end_point);
% meshc(cspace) % Please uncomment for visualization

%% Potantial field construction
%[P] = extract_potential(cspace, end_ik, increment);
% meshc(P) % Please uncomment for visualization

%% Motion planning via Potential Field method
%[pos_list, F] = motion_plan(cspace, start_ik, end_ik, increment);
% Since, we are using the (beta, alpha) convention in above calculations, we
% switch columns during the animation. It is due to our convention confusion.
% Not important detail :)

%% Motion planning via Wave-Front method
[pos_list] = wavefront(cspace, start_ik, end_ik, increment);

pos_list = [(pos_list(:, 2) - 1) * increment, (pos_list(:, 1) - 1) * increment - 180];
beep; % Beep when is done :p

%% Simulation routine for motion planning
simulate(base_of_robot', obstacle_center', obstacle_radius, start_point', end_point', [L1_length; L2_length], pos_list(1:1:end, :));
