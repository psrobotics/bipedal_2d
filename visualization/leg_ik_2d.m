function [j_rad_arr] = leg_ik(leg_vec,l1,l2)
% get joint rad and pos for 3dof leg with a input leg vec

% Desired foot position
x_f = leg_vec(1)+1e-6;
y_f = leg_vec(2)+1e-6;

% Compute q2 (Elbow Joint)
cos_q2 = (x_f^2 + y_f^2 - l1^2 - l2^2) / (2 * l1 * l2);
cos_q2 = max(min(cos_q2, 1), -1);
q2 = acos(cos_q2); % Two solutions: q2 and -q2

% Compute q1 (Hip Joint)
beta = atan2(y_f, x_f);
alpha = atan2(l2 * sin(q2), l1 + l2 * cos(q2));
q1 = beta - alpha + pi/2;

j_rad_arr.a1 = q1;
j_rad_arr.a2 = q2;


end