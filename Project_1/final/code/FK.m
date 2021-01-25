function [x_EE, y_EE, z_EE] = FK(theta_1, theta_2 ,theta_3)
%FK This function calculates the forward kinematics of the Phantom Model
%1.0
%   
%   It basically calculates the end-effector position of the robotic system   
%   with respect to the reference system, which is positioned in the project
%   statement.
%
%   Input:
%       theta_1 = Joint angle 1 [rad]
%       theta_2 = Joint angle 2 [rad]
%       theta_3 = Joint angle 3 [rad]
%
%   Output:
%       x_EE = X position of end effector location w.r.t. reference frame
%       y_EE = Y position of end effector location w.r.t. reference frame
%       z_EE = Z position of end effector location w.r.t. reference frame
%
%   Syntax:
%       theta_1 = 0.312;
%       theta_2 = 1.421;
%       theta_3 = 0.817;
%       [x_EE, y_EE, z_EE] = FK(theta_1, theta_2, theta_3);
%


L_1 = 140; % in mm
L_2 = 140; % in mm

% Calculated forward kinematics equations.
% For more information, please refer to the report.
x_EE = -sin(theta_1) * (L_1 * cos(theta_2) + L_2 * sin(theta_3));
y_EE = L_2 - L_2 * cos(theta_3) + L_1 * sin(theta_2);
z_EE = L_1 * cos(theta_1) * cos(theta_2) - L_1 + L_2 * cos(theta_1) * sin(theta_3);


end

