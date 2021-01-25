function bool = collision_exists(x, y, radius, center_of_obs)

    % Distance between center of obstacle and end effector
    temp_radius = (x - center_of_obs(1))^2 + (y - center_of_obs(2))^2 - radius^2;

    % Check collision with obstacle and the borderlines of the workspace
    if (temp_radius <= 0) || x <= 0 || y <= 0 || x >= 100 || y >= 100

        bool = true;

    else

        bool = false;

    end

end
