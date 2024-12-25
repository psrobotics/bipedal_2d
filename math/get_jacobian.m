function J = get_jacobian(q0,q1,l)
% Generate the Jacobian for a foot based on joint angles
J = [l*cos(q0)+l*cos(q0+q1), l*cos(q0+q1);
    l*sin(q0)+l*sin(q0+q1), l*sin(q0+q1)];
end
