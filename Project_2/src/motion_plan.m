function [pos, F_tot] = motion_plan(varargin)

    % Variable assignments
    if nargin == 4
        cspace = varargin{1};
        start_ik = varargin{2};
        end_ik = varargin{3};
        increment = varargin{4};
        tol = 0.5; %1e+1;
        h = 1e-2;
    else
        disp("Wrong input set!")
    end

    % The list of the obstacle of the workspace
    [obstacle_beta, obstacle_alpha, ~] = find(cspace);
    obstacle_beta = -180.0 + (obstacle_beta - 1) * (increment);
    obstacle_alpha = (obstacle_alpha - 1) * (increment);
    obstacle = [obstacle_beta, obstacle_alpha];

    % Initial position of the end effector
    pos = start_ik;
    % Initial velocity of the end effector
    vel = [0, 0];
    % Error initialization
    err = tol * 2;
    % Counter initiliazation
    cnt = 1;

    while (err > tol && cnt < 20000)% Set max iteration threshold
        % Calculate total force acting on the body
        F_tot(cnt, :) = calculate_force(pos(cnt, :), obstacle, end_ik);

        % Move End Effector in the direction of the total force
        % Euler integration trial.
        %[pos(cnt + 1, :), vel(cnt+1, :)] = propagate_ee(pos(cnt, :), vel(cnt, :), F_tot(cnt, :), h);

        % Move End Effector in the direction of the total force
        [pos(cnt + 1, :)] = propagate_ee(pos(cnt, :), F_tot(cnt, :), h);

        % Error calculation as an Euclidian norm
        err = norm(end_ik - pos(cnt + 1, :));

        cnt = cnt + 1

    end

end

function F_tot = calculate_force(cur_pos, obstacle, end_ik)
    % Coefficient of attractive force
    zeta = 1;
    % Coefficient of repulsive force
    eta = 2000;
    % Distance of influence
    rho_0 = 10.0;
    % Hybrid method for attractive force
    d = 10.0;
    % Repulsive force preallocation
    %F_rep = zeros(size(obstacle, 1), 2);
    F_rep = [0, 0];
    % After that point, we tried a lot of things to obtain a
    % proper repulsive force. So, the below part might look a little bit confusing.
    % In the end, the best result we obtain came from the uncommented part.

    % Again, sorry for the confusion below :(

    %     norms = zeros(size(obstacle,1),1);
    %     for n = 1:size(obstacle, 1)
    %         norms(n) = norm(cur_pos - obstacle(n, :));
    %     end
    %
    %     [rho_q, min_ix] = min(norms);
    %
    %
    %     if rho_q <= rho_0
    %         F_rep = eta * ((1 / rho_q) - (1 / rho_0)) * (rho_q ^ (-2)) *  (cur_pos - obstacle(min_ix,:))/norm(cur_pos - obstacle(min_ix,:)); % grad_min(-cur_pos + obstacle(min_ix, :));
    %     end

    %Loop over all obstacles
    for n = 1:size(obstacle, 1)

        % Euclidian distance between current position of the end effector and
        % indvidual obstacle
        rho_q = norm(cur_pos - obstacle(n, :));

        %rho_q = min(-cur_pos + obstacle(n, :)); % Another method

        if rho_q <= rho_0 && rho_q > 1e-7

            x = cur_pos(1);
            y = cur_pos(2);
            x_i = obstacle(n, 1);
            y_i = obstacle(n, 2);
            %F_rep(n, :) = + eta * ((1 / rho_q) - (1 / rho_0)) * (rho_q ^ (-2)) * (cur_pos - obstacle(n,:));%/norm(cur_pos - obstacle(n,:));;

            % The gradient of the repulsive potential which is calculated based on the Euclidian norm
            F_rep(n, 1) = -(eta * (2 * x - 2 * x_i) * (1 / rho_0 - 1 / ((x - x_i)^2 + (y - y_i)^2)^(1/2))) / (2 * ((x - x_i)^2 + (y - y_i)^2)^(3/2));
            F_rep(n, 2) = -(eta * (2 * y - 2 * y_i) * (1 / rho_0 - 1 / ((x - x_i)^2 + (y - y_i)^2)^(1/2))) / (2 * ((x - x_i)^2 + (y - y_i)^2)^(3/2));

        end

    end

    % Attractive force calculation :)
    %if norm(end_ik - cur_pos) <= d
    F_att = zeta * (end_ik - cur_pos);
    %else
    % F_att = zeta * d * ((end_ik - cur_pos) / (norm(end_ik - cur_pos)));
    %end
    % We choose the maximum among the individual repulsive forces, which kind of represents
    % the closest distance
    F_tot = max(F_rep) + F_att;

end

function [x_next] = propagate_ee(x_prev, F, h)

    % The below is the Adam optimization algorithm, one of our
    % trials :)
    % beta_1 = 0.009; beta_2 = 0.00999; epsilon = 1e-8;
    % m_t = beta_1*m_t+(1-beta_1)*F/norm(F);
    % v_t = beta_1*v_t+(1-beta_2)*(F/norm(F)).^2;
    % m_t_hat = m_t/(1-beta_1^t);
    % v_t_hat = v_t/(1-beta_2^t);
    % x_next = x_prev - h*m_t_hat/(sqrt(v_t_hat)+epsilon);

    % The currently used iteration scheme
    x_next = x_prev +h * F / norm(F);

    %v_next = v_prev + h * F;
    %x_next = x_prev + h * v_next;

    % As a side note, we also tried Euler propagation of
    % velocity and displacement (as a harmonic oscillator),
    % but it also does not work well.
end
