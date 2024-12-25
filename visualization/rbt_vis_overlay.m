function [] = rbt_vis_overlay(x_arr, f_arr, fp_arr, ref_traj_v, T, N,...
                        body_p, world_p, f_name)

size_arr = size(x_arr);
len = size_arr(2);

% color def
blue = [57 106 177]./255;
red = [204 37 41]./255;
black = [83 81 84]./255;
green = [62 150 81]./255;
brown = [146 36 40]./255;
purple = [107 76 154]./255;
orange = [230 97 1]./255;
yellow = [253 184 99]./255;
teal = [94 162 184]./255;
pink = [231 41 138]./255;
lightBlue = [166 206 227]./255;
darkGreen = [27 120 55]./255;
grey = [150 150 150]./255;

% rbt hw para
leg_l = 0.22;

% foot traj vis length
fps_traj_len = 100;
f_vis_k = 0.005;

% process terrain data
te_range = world_p.terrain.range;
te_gs = world_p.terrain.grid_size;
te_map = world_p.terrain.map;
% interpolate
vis_k = 100;
x_ori = linspace(te_range(1), te_range(2), te_gs);
x_vis = linspace(te_range(1), te_range(2), te_gs*vis_k);
% imple mapping using linear indexing
te_map_mapped = interp1(x_ori, te_map, x_vis, 'nearest');

% plot range
skip_k = 85;
s_s = 5;
s_end = 100;
x_range_min = min(x_arr(4,s_s:len-s_end))
x_range_max = max(x_arr(4,s_s:len-s_end))

res_y = 1080/1.5;
res_x = floor(((x_range_max-x_range_min)/1.2)*res_y);

figure(6);
set(gcf, 'Position', [100, 100, res_x, res_y]);

% animate seq
for k = s_s:skip_k:s_end-1

    x_t = x_arr(:,k);

    fp_w_1 = fp_arr(1:3,k);
    fp_w_2 = fp_arr(4:6,k);

    ref_fp_w_1 = ref_traj_v.fpos_ref_f(1:3,k);
    ref_fp_w_2 = ref_traj_v.fpos_ref_f(4:6,k);
    
    r_mat = rot_zyx(x_t(1:3));
    
    foot_pos = fp_arr(:,k);
    feetforce_used = f_arr(:,k)*0.5;

    % get leg ik info
    hip_g_r = x_t(4:6) + r_mat * [0; body_p.width/2; -body_p.height/2];
    hip_g_l = x_t(4:6) + r_mat * [0; -body_p.width/2; -body_p.height/2];
    leg_vec_r = fp_w_1 - hip_g_r;
    leg_vec_l = fp_w_2 - hip_g_l;

    [j_r_r, j_p_r] = leg_ik(leg_vec_r, leg_l, leg_l);
    [j_r_l, j_p_l] = leg_ik(leg_vec_l, leg_l, leg_l);

    p_knee_g_r = hip_g_r + r_mat*j_p_r.knee;
    p_knee_g_l = hip_g_l + r_mat*j_p_l.knee;

    p_leg_arr_r_1 = [hip_g_r, p_knee_g_r];
    p_leg_arr_r_2 = [p_knee_g_r, fp_w_1];
    p_leg_arr_l_1 = [hip_g_l, p_knee_g_l];
    p_leg_arr_l_2 = [p_knee_g_l, fp_w_2];
    
    %clf;
    hold on;
    axis equal;
    %grid on;
    
    % view through y axis
    view([0 1 0]);
    %view(90,0)

    % axis range
    axis([x_range_min-0.6, x_range_max+0.6,...
          -0.6, 0.6,...
          0.0, 1.2]);

    % terrain plot
    plot3(x_vis,...
          0.0*ones(size(te_map_mapped)),...
          te_map_mapped,...
          'k-', 'LineWidth', 1.5);
    hold on;
       
    % plot body
    plot_cube(r_mat,...
              body_p.width_x, body_p.width, body_p.height,...
              x_t(4:6)+r_mat*[0;0;0], black, 1.5);

    % leg 
    plot3(p_leg_arr_r_1(1,:), p_leg_arr_r_1(2,:), p_leg_arr_r_1(3,:),'color', blue,'linewidth',1.5);
    plot3(p_leg_arr_r_2(1,:), p_leg_arr_r_2(2,:), p_leg_arr_r_2(3,:),'color', orange,'linewidth',1.5);
    plot3(p_leg_arr_l_1(1,:), p_leg_arr_l_1(2,:), p_leg_arr_l_1(3,:),'color', darkGreen,'linewidth',1.5);
    plot3(p_leg_arr_l_2(1,:), p_leg_arr_l_2(2,:), p_leg_arr_l_2(3,:),'color', yellow,'linewidth',1.5);

    % foot
    plot3(fp_w_1(1),fp_w_1(2),fp_w_1(3),'o','linewidth',1,'color','b','markersize',4);
    plot3(fp_w_2(1),fp_w_2(2),fp_w_2(3),'o','linewidth',1,'color','b','markersize',4);

    % hip
    plot3(hip_g_r(1),hip_g_r(2),hip_g_r(3),'o','linewidth',1,'color','b','markersize',4);
    plot3(hip_g_l(1),hip_g_l(2),hip_g_l(3),'o','linewidth',1,'color','b','markersize',4);

    % knee
    plot3(p_knee_g_r(1),p_knee_g_r(2),p_knee_g_r(3),'o','linewidth',1,'color','b','markersize',4);
    plot3(p_knee_g_l(1),p_knee_g_l(2),p_knee_g_l(3),'o','linewidth',1,'color','b','markersize',4);

    % ref foot
    plot3(ref_fp_w_1(1),ref_fp_w_1(2),ref_fp_w_1(3),...
        'o','linewidth',2,'color',teal,'markersize',4);
    plot3(ref_fp_w_2(1),ref_fp_w_2(2),ref_fp_w_2(3),...
        'o','linewidth',2,'color',lightBlue,'markersize',4);
    
    % * 2 legs
    for i=1:2
        x_indx=3*(i-1)+1;
        y_indx=3*(i-1)+2;
        z_indx=3*(i-1)+3;
        
        % plot force
         plot3([foot_pos(x_indx), foot_pos(x_indx)+ f_vis_k*feetforce_used(x_indx)],...
               [foot_pos(y_indx), foot_pos(y_indx)+ f_vis_k*feetforce_used(y_indx)],...
               [foot_pos(z_indx), foot_pos(z_indx)+ f_vis_k*feetforce_used(z_indx)],...
               'linewidth', 1.0, 'color', red);
         hold on;
        
    end

    % fps traj, body ref traj
    if k<len-fps_traj_len-1
        % fpos 1
         plot3(ref_traj_v.fpos_ref_f(1,k:k+fps_traj_len),...
               ref_traj_v.fpos_ref_f(2,k:k+fps_traj_len),...
               ref_traj_v.fpos_ref_f(3,k:k+fps_traj_len),...
               'linewidth', 1.0, 'color', teal); 
         hold on;
         % fpos 2
         plot3(ref_traj_v.fpos_ref_f(4,k:k+fps_traj_len),...
               ref_traj_v.fpos_ref_f(5,k:k+fps_traj_len),...
               ref_traj_v.fpos_ref_f(6,k:k+fps_traj_len),...
               'linewidth', 1.0, 'color', lightBlue);  
         hold on;
         % body ref
         plot3(ref_traj_v.x_ref_f(4,k),...
               ref_traj_v.x_ref_f(5,k),...
               ref_traj_v.x_ref_f(6,k)-0.15,...
               'o','linewidth',1.0,'color',grey,'markersize',4);
         plot3(ref_traj_v.x_ref_f(4,k:k+fps_traj_len),...
               ref_traj_v.x_ref_f(5,k:k+fps_traj_len),...
               ref_traj_v.x_ref_f(6,k:k+fps_traj_len)-0.15,...
               'linewidth', 1.0, 'color', grey);  
         hold on;
    end
    
    % debug reverse x axis
    set(gca, 'XDir', 'reverse');
    
end

% save final figure
resolution = 600;
print(6, f_name, '-dpng', ['-r', num2str(resolution)]);
%ffig = gcf;
%exportgraphics(ffig, f_name, 'BackgroundColor', 'white', 'Resolution', 600);

end

