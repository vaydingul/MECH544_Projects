% Equation checker

syms theta_1 theta_2 theta_3 L1 L2 q1 q2 q3
syms transform(a, alpha, theta, d) % syms function definiton, it is required, idk why?

% successive transformation matrix
transform(alpha, a, theta, d) = [cos(theta), -sin(theta), 0, a;
    sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d;
    sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), cos(alpha) * d;
    0, 0, 0, 1];

% Angle conversion for Phantom Model 1.0
q1 = -theta_1;
q2 = theta_2;
q3 = theta_3 - theta_2 - pi / 2;

% Successive transformation from 0 to 4
T_0_1 = transform(0.0, 0.0, q1, 0)
T_1_2 = transform(pi/2, 0, q2, 0)
T_2_3 = transform(0, L1, q3, 0)
T_3_4 = transform(0, L2, 0, 0)

T_0_4 = T_0_1 * T_1_2 * T_2_3 * T_3_4

% Inclusion of reference frame
T_ref_4 = [0, 1, 0, 0;
                0, 0, 1, L2;
                1, 0, 0, -L1;
                0 , 0, 0, 1] * T_0_4;


pretty(simplify(T_ref_4))

latex_cmd = latex((simplify(T_ref_4(1:3, 4)))) % ONE OF THE MOST USEFUL CODE IN THE WORLD


