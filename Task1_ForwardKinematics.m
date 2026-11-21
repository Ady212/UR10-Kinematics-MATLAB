% Task 1: Forward Kinematics Simulator for UR10 Manipulator
% This MATLAB script calculates and visualizes the forward kinematics of the UR10 arm.

clc
clear all

% Define UR10 DH parameters: [alpha, a, d, theta_offset]
DH = [
    pi/2,    0,      0.1273,   0;
    0,      -0.612,  0,        pi/2;
    0,      -0.57155,0,        0;
    pi/2,    0,      0.163941, 0;
   -pi/2,    0,      0.1157,   0;
    0,       0,      0.0922,   0
];

% Define joint angles in degrees (can be replaced with user input)
theta_deg = [0, -90, 90, 0, 45, 0];
theta = deg2rad(theta_deg);  % Convert to radians

% Initialize transformation matrices
T = eye(4);
positions = zeros(3, 7);  % Store joint positions including base
positions(:,1) = [0; 0; 0];

% Compute transformations
for i = 1:6
    alpha = DH(i,1);
    a     = DH(i,2);
    d     = DH(i,3);
    theta_i = theta(i) + DH(i,4);

    A = trotz(theta_i) * transl(a,0,0) * trotx(alpha) * transl(0,0,d);
    T = T * A;
    positions(:,i+1) = T(1:3,4);
end

% Plot the UR10 manipulator
figure('Name','UR10 Forward Kinematics','Color','w');
hold on; grid on; axis equal;
plot3(positions(1,:), positions(2,:), positions(3,:), '-o','LineWidth',2);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('UR10 Manipulator Configuration');
view(45,20);
hold off;

% --- Helper Functions ---
function T = trotz(theta)
    T = [cos(theta), -sin(theta), 0, 0;
         sin(theta),  cos(theta), 0, 0;
         0,           0,          1, 0;
         0,           0,          0, 1];
end

function T = trotx(alpha)
    T = [1, 0,           0,          0;
         0, cos(alpha), -sin(alpha), 0;
         0, sin(alpha),  cos(alpha), 0;
         0, 0,           0,          1];
end

function T = transl(x,y,z)
    T = eye(4);
    T(1:3,4) = [x; y; z];
end
