% Link lengths
l1 = 0.22; % Length of first link (upper leg)
l2 = 0.22; % Length of second link (lower leg)

% Desired foot position
x_f = -0.1; % Foot X-coordinate
y_f = -0.3; % Foot Y-coordinate

% Inverse Kinematics
% Compute q2 (Elbow Joint)
cos_q2 = (x_f^2 + y_f^2 - l1^2 - l2^2) / (2 * l1 * l2);
q2 = acos(cos_q2); % Two solutions: q2 and -q2

% Compute q1 (Hip Joint)
beta = atan2(y_f, x_f);
alpha = atan2(l2 * sin(q2), l1 + l2 * cos(q2));
q1 = beta - alpha + pi/2;

% Convert to degrees for readability
q1_deg = rad2deg(q1);
q2_deg = rad2deg(q2);

% Display results
fprintf('Hip joint angle (q1): %.2f degrees\n', q1_deg);
fprintf('Knee joint angle (q2): %.2f degrees\n', q2_deg);

% Visualize the leg
q1_tmp = q1-pi/2;
x1 = l1 * cos(q1_tmp);
y1 = l1 * sin(q1_tmp);
x2 = x1 + l2 * cos(q1_tmp + q2);
y2 = y1 + l2 * sin(q1_tmp + q2);

figure;
plot([0, x1, x2], [0, y1, y2], '-o', 'LineWidth', 2);
hold on;
plot(x_f, y_f, 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Target foot position
axis equal;
grid on;
xlabel('X Position');
ylabel('Y Position');
title('2-DOF Leg Inverse Kinematics');
legend('Leg Configuration', 'Target Foot Position');