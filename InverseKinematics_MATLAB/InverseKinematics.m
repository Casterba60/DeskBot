clear; clc; close all;

%% ----------------- SYMBOLIC DEFINITIONS -----------------
syms theta1 theta2 theta3 theta4 real
q = [theta1; theta2; theta3; theta4];

% DH Transform function
dh_transform = @(a, alpha, d, theta) ...
    [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
     sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
     0,           sin(alpha),             cos(alpha),            d;
     0,           0,                      0,                     1];

% Tool transform: fixed geometry (no local orientation)
T_tool = [1 0 0 120;
          0 1 0 -24;
          0 0 1   0;
          0 0 0   1];

% Forward Kinematics
T1 = dh_transform(0, pi/2, 0, theta1);
T2 = dh_transform(150, 0, 0, theta2);
T3 = dh_transform(150, 0, 0, theta3);
T4 = dh_transform(0, 0, 0, theta4);
T4 = T4 * T_tool;
T_final = simplify(T1 * T2 * T3 * T4);

% End-effector position and orientation
p_ee = T_final(1:3, 4);
R_ee = T_final(1:3, 1:3);

%% ----------------- SYMBOLIC JACOBIAN -----------------
% Position Jacobian
Jp = sym(zeros(3,4));
for i = 1:4
    Jp(:,i) = diff(p_ee, q(i));
end

% Orientation Jacobian
Jo = sym(zeros(3,4));
for i = 1:4
    dR = diff(R_ee, q(i));
    w_hat = simplify(dR * R_ee');
    Jo(:,i) = [w_hat(3,2); w_hat(1,3); w_hat(2,1)];
end

J_full = simplify([Jp; Jo]);

% Function handles
jacobian_func = matlabFunction(J_full, 'Vars', {q});
position_func = matlabFunction(p_ee, 'Vars', {q});
rotation_func = matlabFunction(R_ee, 'Vars', {q});

%% ----------------- INVERSE KINEMATICS -----------------
% Global position target
target_pos = [325; 25; -50];  % mm

% Global orientation target (rotate -30° about global Z axis)
angle = deg2rad(-30);
target_rot = [cos(angle), -sin(angle), 0;
              sin(angle),  cos(angle), 0;
              0,           0,          1];

% Initial guess
q_curr = deg2rad([10; 10; -20; 0]);

% IK parameters
max_iter = 100;
alpha = 0.3;
tol = 1e-3;

for i = 1:max_iter
    p_now = position_func(q_curr);
    R_now = rotation_func(q_curr);
    J_now = jacobian_func(q_curr);

    % Errors
    e_pos = target_pos - p_now;

    R_err = target_rot * R_now';
    ang = acos((trace(R_err) - 1)/2);
    if abs(ang) < 1e-6
        e_ori = [0; 0; 0];
    else
        axis = 1/(2*sin(ang)) * [R_err(3,2) - R_err(2,3);
                                 R_err(1,3) - R_err(3,1);
                                 R_err(2,1) - R_err(1,2)];
        e_ori = axis * ang;
    end

    % Combined error
    e = [e_pos; e_ori];

    % Update
    dq = pinv(J_now) * e;
    q_curr = q_curr + alpha * dq;

    if norm(e) < tol
        break;
    end
end

disp('Final joint angles (degrees):');
disp(rad2deg(q_curr));

%% ----------------- VISUALIZATION -----------------
% Evaluate transforms
T1_eval = double(subs(dh_transform(0, pi/2, 0, theta1), theta1, q_curr(1)));
T2_eval = double(subs(dh_transform(150, 0, 0, theta2), theta2, q_curr(2)));
T3_eval = double(subs(dh_transform(150, 0, 0, theta3), theta3, q_curr(3)));
T4_eval = double(subs(dh_transform(0, 0, 0, theta4), theta4, q_curr(4)));
T4_eval = T4_eval * T_tool;

% Combine transforms
T01 = T1_eval;
T02 = T01 * T2_eval;
T03 = T02 * T3_eval;
T04 = T03 * T4_eval;

% Joint positions
p0 = [0; 0; 0];
p1 = T01(1:3, 4);
p2 = T02(1:3, 4);
p3 = T03(1:3, 4);
p4 = T04(1:3, 4);

% Plot robot
X = [p0(1), p1(1), p2(1), p3(1), p4(1)];
Y = [p0(2), p1(2), p2(2), p3(2), p4(2)];
Z = [p0(3), p1(3), p2(3), p3(3), p4(3)];

figure;
plot3(X, Y, Z, '-o', 'LineWidth', 2, 'MarkerSize', 6);
grid on; axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('4-DoF IK with Global Orientation Input');
view(45, 30); hold on;

% Draw joint frames
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

% Target position and orientation frame
scatter3(target_pos(1), target_pos(2), target_pos(3), 100, 'k', 'filled');
for j = 1:3
    quiver3(target_pos(1), target_pos(2), target_pos(3), ...
            target_rot(1,j), target_rot(2,j), target_rot(3,j), ...
            scale, '--k', 'LineWidth', 2);
end