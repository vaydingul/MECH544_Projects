function simulate(base, obs_center, radius, start_, end_, links_length, angles);

    % BIG ANIMATION SCRIPT

    plot(base(1), base(2), 'ro', 'MarkerSize', 6, 'linewidth', 6);
    hold on;
    %plot(obs_center(1),obs_center(2),'ko','MarkerSize',radius,'linewidth',radius);
    circle(obs_center(1), obs_center(2), radius)
    plot(start_(1), start_(2), 'bh', 'MarkerSize', 10, 'linewidth', 3);
    text(start_(1) - 5, start_(2) + 5, "Start Point");
    plot(end_(1), end_(2), 'gh', 'MarkerSize', 10, 'linewidth', 3);
    text(end_(1) - 5, end_(2) + 5, "End Point");
    axis([0 100 0 100]);
    grid on;
    history_ee = zeros(size(angles, 1), 2);

    for i = 1:size(angles, 1)
        %FK for first Link
        L1_XY = base + [links_length(1) * cosd(angles(i, 1)); links_length(2) * sind(angles(i, 1))];
        L2_XY = L1_XY +[links_length(2) * cosd(angles(i, 1) - angles(i, 2)); links_length(2) * sind(angles(i, 1) - angles(i, 2))];
        shoulder = [base, L1_XY];
        elbow = [L1_XY, L2_XY];
        L1 = line(shoulder(1, :), shoulder(2, :), 'color', 'r', 'linewidth', 3);
        J1 = plot(L1_XY(1), L1_XY(2), 'mo', 'MarkerSize', 5, 'linewidth', 5);
        L2 = line(elbow(1, :), elbow(2, :), 'color', 'm', 'linewidth', 3);
        J2 = plot(L2_XY(1), L2_XY(2), 'kx', 'MarkerSize', 15, 'linewidth', 3);
        history_ee(i, :) = L2_XY';

        if i > 1
            history = [history_ee(i - 1, :)', history_ee(i, :)'];
            line(history(1, :), history(2, :), 'color', 'r', 'linewidth', 0.1);
        end

        pause(0.00000001);
        %a=input('Press any key to continue...');
        if i ~= size(angles, 1)
            delete(L1); delete(L2); delete(J1); delete(J2);
        end

    end
    
    title(['$L_1$ = ', num2str(links_length(1)),'   ', '$L_2$ = ', num2str(links_length(2))], "interpreter", "latex")
    hold off;

end
