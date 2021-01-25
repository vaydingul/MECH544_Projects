function opts = generate_ngbrhood(counter)

    % It generates counter x counter peripheral square
    % for wavefront method
    n = 8 * counter - 6;
    opts = zeros(n, 2);
    x_counter = 1;

    for r = -counter:counter

        if r == -counter || r == counter

            for c = -counter:counter
                opts(x_counter, :) = [r c];
                x_counter = x_counter + 1;

            end

        else

            for c = [-counter, counter]
                opts(x_counter, :) = [r c];
                x_counter = x_counter + 1;

            end

        end

    end

end
