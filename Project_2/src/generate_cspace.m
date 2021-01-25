function [cspace, start_ik, end_ik] = generate_cspace(increment, L1_, L2_, radius, center_of_obs, base_of_robot, start_, end_)

    % C-space matrix preallocation, in a discrete way.
    cspace = zeros(360 / increment + 1, 180 / increment + 1);

    %%%%%%%%%%%%%%%%%%% IK of Start and End points %%%%%%%%%%%%%
    % Elbow down
    % start
    [s1_alpha, s1_beta] = inverse_kinematics(start_ - base_of_robot, L1_, L2_, -1);
    % end
    [s2_alpha, s2_beta] = inverse_kinematics(end_ - base_of_robot, L1_, L2_, -1);
    % Elbow up
    % start
    [s3_alpha, s3_beta] = inverse_kinematics(start_ - base_of_robot, L1_, L2_, 1);
    % end
    [s4_alpha, s4_beta] = inverse_kinematics(end_ - base_of_robot, L1_, L2_, 1);

    start_ik = [s1_beta, s1_alpha];
    end_ik = [s2_beta, s2_alpha];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%  Plot generation %%%%%%%%
    % hold on;
    % plot(s1x, s1y, 'gh','MarkerSize', 10, 'linewidth', 3); text(s1x-40, s1y + 15, "Start Point Elbow Up Solution" );
    % plot(s2x, s2y, 'bh','MarkerSize', 10, 'linewidth', 3); text(s2x-20, s2y + 15, "End Point Elbow Up Solution" );
    % plot(s3x, s3y, 'gs','MarkerSize', 10, 'linewidth', 3); text(s3x-20, s3y + 15, "Start Point Elbow Down Solution" );
    % plot(s4x, s4y, 'bs','MarkerSize', 10, 'linewidth', 3); text(s4x-20, s4y + 15, "End Point Elbow Down Solution" );
    %
    % xlabel('\alpha');
    % ylabel('\beta');
    % title('Configuration Space');
    % grid on;
    %%%%%%%%%%%%%%%%%%%%%%%%%

    % Counter initialization for the loops
    alpha_counter = 1;
    beta_counter = 1;

    for alpha = 0:increment:180

        for beta = -180:increment:180
            % Preallocate original L1 and L2 length values
            L1 = L1_;
            L2 = L2_;
            % Forward kinematics of the end effector
            x = base_of_robot(1) + L1 * cosd(alpha) + L2 * cosd(alpha - beta); y = base_of_robot(2) + L1 * sind(alpha) + L2 * sind(alpha - beta);

            while (~collision_exists(x, y, radius, center_of_obs))
                % If there is not any collision, then, look for the other
                % points in the arms
                if L2 > 0
                    L2 = L2 - 1;
                else
                    L1 = L1 - 1;
                end

                if L1 <= 0
                    break
                end

                % Then, recalculate forward kinematics for the new point on the
                % arm.
                x = base_of_robot(1) + L1 * cosd(alpha) + L2 * cosd(alpha - beta); y = base_of_robot(2) + L1 * sind(alpha) + L2 * sind(alpha - beta);

            end

            % If the arm still exists, and does not collide with itself
            if L1 > 0%|| abs(beta) == 180
                %plot(alpha, beta, 'r+')
                cspace(beta_counter, alpha_counter) = 1;
            end

            beta_counter = beta_counter + 1;
        end

        beta_counter = 1;
        alpha_counter = alpha_counter + 1;
    end

    %hold off;
end
