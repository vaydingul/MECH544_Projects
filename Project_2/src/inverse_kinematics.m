function [q1, q2] = inverse_kinematics(xy, L1, L2, type)
    % type = -1 ==> elbow down
    % type = 1 ==> elbow up
    x = xy(1);
    y = xy(2);

    if type == 1
        cosq2 = (-L1^2 - L2^2 + x^2 + y^2) / (2 * L1 * L2);
        sinq2 = sqrt(1 - cosq2^2);
        q2 = atan2(sinq2, cosq2);
        q2 = -q2;
        a = L2 * sinq2;
        b = L1 + L2 * cosq2;
        c = y;
        q1 = atan2(c, sqrt(a^2 + b^2 - c^2)) - atan2(a, b);

    else
        cosq2 = (-L1^2 - L2^2 + x^2 + y^2) / (2 * L1 * L2);
        sinq2 = -sqrt(1 - cosq2^2);
        q2 = atan2(sinq2, cosq2);
        q2 = -q2;
        a = L2 * sinq2;
        b = L1 + L2 * cosq2;
        c = y;
        q1 = atan2(c, sqrt(a^2 + b^2 - c^2)) - atan2(a, b);
    end

    q1 = q1 * 180 / pi;
    q2 = q2 * 180 / pi;

end
