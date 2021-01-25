function simulate_cspace(cspace, start_ik, end_ik, obs_center, radius, beta_, alpha_, increment)
    [nx, ny] = size(cspace);

    hold on;
    cnt = 1;

    for i = 1:1:nx

        for k = 1:1:ny

            if cspace(i, k)

                x_ = -180.0 + (i - 1) * increment;
                y_ = (k - 1) * increment;
                temp(cnt, :) = [y_, x_];
                cnt = cnt + 1;
            end

        end

    end

    scatter(temp(:, 1), temp(:, 2), "r.");

    %circle(obs_center(1), obs_center(2), radius, 1);
    plot(start_ik(2), start_ik(1), 'bh', 'MarkerSize', 10, 'linewidth', 3);
    text(start_ik(2) - 5, start_ik(1) + 5, "Start Point");
    plot(end_ik(2), end_ik(1), 'gh', 'MarkerSize', 10, 'linewidth', 3);
    text(end_ik(2) - 5, end_ik(1) + 5, "End Point");
    %axis([0 180 -180 180]);
    grid on;
    history_ee = zeros(size(beta_, 1), 2);

    for i = 1:size(beta_, 1)
        history_ee(i, :) = [alpha_(i), beta_(i)];

        if i > 1
            history = [history_ee(i - 1, :)', history_ee(i, :)'];
            line(history(1, :), history(2, :), 'color', 'b', 'linewidth', 0.1);
        end

        pause(0.00000001);
    end

end
