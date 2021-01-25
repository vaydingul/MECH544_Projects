function [pos] = wavefront(varargin)
    % Variable assignments
    if nargin == 4
        cspace = varargin{1};
        start_ik = varargin{2};
        end_ik = varargin{3};
        increment = varargin{4};
    else
        disp("Wrong input set!")
    end

    map = cspace;
    end_ik(1) = end_ik(1) + 180.0; %Conversions from the inverse kinematics

    map(round(end_ik(1)), round(end_ik(2))) = 2; %Value of the goal position
    [ny, nx] = size(cspace);
    cur_pos = round(end_ik);
    counter = 1;

    %% Generating the Map
    %  Assign the values of the moore neighbours cells
    while counter <= max(nx, ny)
        opts = generate_ngbrhood(counter);
        dir_ = cur_pos + opts;
        previous_map = map;

        for k = 1:size(opts, 1)

            if ~(dir_(k, 2) < 1 || dir_(k, 2) > nx || dir_(k, 1) < 1 || dir_(k, 1) > ny)

                if (cspace(dir_(k, 1), dir_(k, 2)) == 0)

                    if counter == 1
                        map(dir_(k, 1), dir_(k, 2)) = map(cur_pos(1), cur_pos(2)) + 1;
                    else
                        dir__ = [dir_(k, 1), dir_(k, 2)] + generate_ngbrhood(1);
                        minimum = 9999;

                        for i = 1:size(dir__, 1)

                            if ~(dir__(i, 2) < 1 || dir__(i, 2) > nx || dir__(i, 1) < 1 || dir__(i, 1) > ny)

                                if ~(previous_map(dir__(i, 1), dir__(i, 2)) == 1 || previous_map(dir__(i, 1), dir__(i, 2)) == 0)
                                    minimum = min(minimum, previous_map(dir__(i, 1), dir__(i, 2)));
                                end

                            end

                        end

                        if minimum == 9999
                            map(dir_(k, 1), dir_(k, 2)) = 0;
                        else
                            map(dir_(k, 1), dir_(k, 2)) = minimum + 1;
                        end

                    end

                end

            end

        end

        counter = counter +1;
    end

    disp('WaveFront Map Layer 1 is Done!');
    % Fill the free cells
    [row, col, ~] = find(map == 0);

    while (~isempty(row))

        for r = 1:size(row)
            filldir_ = [row(end - r + 1), col(end - r + 1)] + generate_ngbrhood(1);
            previous_map = map;

            for k = 1:size(filldir_, 1)

                if ~(filldir_(k, 2) < 1 || filldir_(k, 2) > nx || filldir_(k, 1) < 1 || filldir_(k, 1) > ny)

                    if (previous_map(filldir_(k, 1), filldir_(k, 2)) == 0)
                        filldir__ = [filldir_(k, 1), filldir_(k, 2)] + generate_ngbrhood(1);
                        minimum = 9999;

                        for i = 1:size(filldir__, 1)

                            if ~(filldir__(i, 2) < 1 || filldir__(i, 2) > nx || filldir__(i, 1) < 1 || filldir__(i, 1) > ny)

                                if ~(previous_map(filldir__(i, 1), filldir__(i, 2)) == 1 || previous_map(filldir__(i, 1), filldir__(i, 2)) == 0)
                                    minimum = min(minimum, previous_map(filldir__(i, 1), filldir__(i, 2)));
                                end

                            end

                        end

                        if minimum ~= 9999
                            map(filldir_(k, 1), filldir_(k, 2)) = minimum + 1;
                        end

                    end

                end

            end

        end

        [row, col, ~] = find(map == 0);
    end

    disp('WaveFront Map is Done!\n');
    %% Solving the Path
    start_ik(1) = start_ik(1) + 180.0;
    cur_pos = round(start_ik);

    pos = cur_pos; %initial position
    counter = 1;
    value = map(cur_pos(1), cur_pos(2));

    while value ~= 2 && counter < 10000
        dir_ = cur_pos + generate_ngbrhood(1);
        next_pos = cur_pos;
        minimum_value = 9999;

        for k = 1:size(dir_, 1)

            if ~(dir_(k, 2) < 1 || dir_(k, 2) > nx || dir_(k, 1) < 1 || dir_(k, 1) > ny)

                if (map(dir_(k, 1), dir_(k, 2)) ~= 1)

                    if (map(dir_(k, 1), dir_(k, 2)) < minimum_value)
                        minimum_value = map(dir_(k, 1), dir_(k, 2));
                        next_pos = [dir_(k, 1), dir_(k, 2)];
                    end

                end

            end

        end

        value = minimum_value;
        sprintf("Value: %d\n", value)
        counter = counter + 1;
        cur_pos = next_pos;
        pos(counter, :) = next_pos;
    end

end
