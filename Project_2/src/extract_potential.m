function [P] = extract_potential(cspace, end_ik, increment)
    %extract_potential
    % This function calculates the cumulative potential

    % Get attractive potential
    [P_att] = extract_attractive_potential_(cspace, end_ik, increment);
    % Get repulsive potential
    [P_rep] = extract_repulsive_potential_(cspace, increment);
    % Output overall potential
    P = P_att + P_rep;
    %F = F_att + F_rep;
end

function [P_att] = extract_attractive_potential_(cspace, end_ik, increment)
    % Attractive potential constant
    zeta = .01;
    % Discrete dimensions of the C-Space
    beta_r = size(cspace, 1);
    alpha_r = size(cspace, 2);

    % Attractive potential matrix preallocation
    P_att = zeros(beta_r, alpha_r);
    %F_att = zeros(beta_r, alpha_r, 2);

    for k = 1:beta_r

        for m = 1:alpha_r

            % Conversion from discrete array indexes to joint angle space
            beta_ = (k - 1) * increment;
            alpha_ = (m - 1) * increment;

            % Attractive potential calculation
            P_att(k, m) = 0.5 * zeta * norm(end_ik - [beta_, alpha_])^2;
            %F_att(k, m, :) = -zeta * (end_ik - [beta_, alpha_]);

        end

    end

end

function [P_rep] = extract_repulsive_potential_(cspace, increment)

    % Repulsive potential coefficient
    eta = 2000.0;
    % Distance of influence
    % It should be bigger than the grid size, namely ´increment´
    rho_0 = 10.0;

    % Discrete dimensions of the C-Space
    beta_r = size(cspace, 1);
    alpha_r = size(cspace, 2);

    % Repulsive potential matrix preallocation
    P_rep = zeros(beta_r, alpha_r);
    %F_rep = zeros(beta_r, alpha_r, 2);

    % Following four rows calculates the obstacle coordinates in the joint angle space
    [obstacle_beta, obstacle_alpha, ~] = find(cspace);
    obstacle_beta = (obstacle_beta - 1) * (increment);
    obstacle_alpha = (obstacle_alpha - 1) * (increment);
    obstacle = [obstacle_beta, obstacle_alpha];

    % Loop over all discrete coordinates and obstacle coordinates
    for k = 1:beta_r

        for m = 1:alpha_r

            for n = 1:size(obstacle, 1)

                % Conversion from discrete array indexes to joint angle space
                beta_ = (k - 1) * increment;
                alpha_ = (m - 1) * increment;

                % Define rho_q as Euclidian norm
                rho_q = norm([beta_, alpha_] - obstacle(n, :));

                % Also, the minimum approach was tried
                %rho_q = min([beta_, alpha_] - obstacle(n, :));

                if rho_q <= rho_0 && rho_q > 1e-7

                    % Calculate the repulsive potential
                    P_rep(k, m) = P_rep(k, m) + 0.5 * eta * ((1 / rho_q) - (1 / rho_0))^2;
                    %F_rep(k, m, :) = eta * [ ((beta_ - obstacle(n, 1)) ^ (-3)) - ((-0.5 / rho_0) * ((alpha_ - obstacle(n, 1)) ^(-3/2))), ((beta_ - obstacle(n, 1)) ^ (-3)) - ((-0.5 / rho_0) * ((alpha_ - obstacle(n, 1)) ^(-3/2))) ];

                end

            end

        end

    end

end
