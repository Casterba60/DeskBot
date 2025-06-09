%Define DH Matrices for each transformation:
% Lengths are in mm, angles are in radians
clear all; close all;
syms theta1 theta2 theta3 theta4;

%shoulder yaw
%a = 0 mm
%d = 0 mm
%alpha = pi/2 rad
%theta = theta1
T1 = dh_transform(0,pi/2,0,theta1);

%shoulder pivot
%a = 150
%d = 0
%alpha = 0
%theta = theta2
T2 = dh_transform(150,0,0,theta2);

%elbow
%a = 150 
%d = 0
%beta = 0
%theta = theta3
T3 = dh_transform(150,0,0,theta3);

%wrist
%a = 0
%d = 0
%beta = 0
%theta = theta4
T4 = dh_transform(0,0,0,theta4);

T_tool = [1  0  0  120;  
          0  1  0  -24;   
          0  0  1    0;   
          0  0  0    1];

T4 = T4*T_tool;

%input joint angles here
vals = [deg2rad(30), deg2rad(67), deg2rad(0), deg2rad(-27)];

% Evaluate numerically
T1_eval = double(subs(T1, theta1, vals(1)));
T2_eval = double(subs(T2, theta2, vals(2)));
T3_eval = double(subs(T3, theta3, vals(3)));
T4_eval = double(subs(T4, theta4, vals(4)));
%cumulative transforms
T01 = T1_eval;
T02 = T01 * T2_eval;
T03 = T02 * T3_eval;
T04 = T03 * T4_eval;
%generate points
p0 = [0; 0; 0];
p1 = T01(1:3, 4);
p2 = T02(1:3, 4);
p3 = T03(1:3, 4);
p4 = T04(1:3, 4);  % end effector

%plot
X = [p0(1), p1(1), p2(1), p3(1), p4(1)];
Y = [p0(2), p1(2), p2(2), p3(2), p4(2)];
Z = [p0(3), p1(3), p2(3), p3(3), p4(3)];

figure;
plot3(X, Y, Z, '-o', 'LineWidth', 2, 'MarkerSize', 6);
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('4-DOF Robotic Arm');
axis equal      % Ensures units are the same in all directions
view(45, 30);  % adjustable angle

%joint axes
hold on;
frames = {eye(4), T01, T02, T03, T04};
colors = 'rgb';
scale = 30;

for i = 1:length(frames)
    R = frames{i}(1:3,1:3);
    o = frames{i}(1:3,4);
    for j = 1:3
        quiver3(o(1), o(2), o(3), R(1,j), R(2,j), R(3,j), scale, colors(j), 'LineWidth', 1.5);
    end
end