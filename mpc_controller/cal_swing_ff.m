function [low_ctr] = cal_swing_ff(x_sol, f_sol, fp_g_sol, body_p, ctr_p)

size_arr = size(x_sol);
len = size_arr(2);

dt = ctr_p.dt;

leg_l = 0.22;
body_h = 0.25;

% unpack data
x_s = x_sol(4,:);
z_s = x_sol(6,:);
theta_s = x_sol(2,:);
dx_s = x_sol(4+6,:);
dz_s = x_sol(6+6,:);
dtheta_s = x_sol(2+6,:);

% fpos
fp_x1 = fp_g_sol(1,:);
fp_x2 = fp_g_sol(4,:);
fp_z1 = fp_g_sol(3,:);
fp_z2 = fp_g_sol(6,:);
fp_s = [fp_x1; fp_z1; fp_x2; fp_z2];

% leg rad
hip_s = zeros(2,len-1);
for i = 1:len-1
    R_th = rot_theta(theta_s(i));
    % consider com to hip offset
    hip_s(:,i) = [x_s(i); z_s(i)] + R_th*[0; -body_h/2];
end
% leg vector array

lvec_l = fp_s(1:2,:) - hip_s;
lvec_r = fp_s(3:4,:) - hip_s;

rad_s = zeros(4,len-1);
for i = 1:len-1
    [rad_1] = leg_ik_2d(lvec_l(:,i),leg_l,leg_l);
    [rad_2] = leg_ik_2d(lvec_r(:,i),leg_l,leg_l);
    rad_s(:,i) = [rad_1.a1;...
                  rad_1.a2;...
                  rad_2.a1;...
                  rad_2.a2];
end

% dfpos, drad
dfp_s = zeros(size(fp_s));
drad_s = zeros(size(rad_s));
for i = 2:len-2
    dfp_s(:,i) = (fp_s(:,i+1) - fp_s(:,i-1)) / (dt + dt);
    drad_s(:,i) = (rad_s(:,i+1) - rad_s(:,i-1)) / (dt + dt);
end

% torque
tor_s = zeros(size(rad_s));
for i = 1:len-1
    J1 = get_jacobian(rad_s(1,i), rad_s(2,i), leg_l);
    J2 = get_jacobian(rad_s(3,i), rad_s(4,i), leg_l);
    R_th = rot_theta(theta_s(i));
    tor1 = J1'*R_th'*[f_sol(1,i);f_sol(3,i)];
    tor2 = J2'*R_th'*[f_sol(4,i);f_sol(6,i)];
    tor_s(:,i) = [tor1; tor2];
end

% passout
low_ctr.fp_s = fp_s;
low_ctr.rad_s = rad_s;
low_ctr.dfp_s = dfp_s;
low_ctr.drad_s = drad_s;
low_ctr.tor_s = tor_s;
% debug
low_ctr.hip_s = hip_s;
low_ctr.lvec_l = lvec_l;
low_ctr.lvec_r = lvec_r;

end