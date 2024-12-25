%% Add casadi path

clc; clear;
close all; warning off;

%% Get hardware params & set package path
[world_params, body_params, ctr_params, gait, path] = hardware_params();

addpath(path.casadi);
import casadi.*;

addpath('math/');
addpath('srb_dynamics/');
addpath('mpc_controller/');
addpath('fpp_planner/');
addpath('visualization/');

%% Get dynamics
[dyn_f] = get_srb_dynamics(world_params, body_params, path);

%% Form the mpc problem
[mpc_v, mpc_c, mpc_p] = form_mpc_prob(world_params, body_params, ctr_params, dyn_f, path);
%%
[boundray_v] = add_state_boundaries(mpc_v, mpc_c, world_params, body_params, ctr_params, path);
%%
x0 = [0;0;0; 0;0; ctr_params.b_h_tar; 0;0;0; 0;0;0];
x0_2d = [x0(4); x0(6); x0(2); x0(4+6); x0(6+6); x0(2+6)];
[ref_traj_v] = fpp_planner_n(world_params, body_params, ctr_params, gait, path, x0_2d);

%% Slove the NLP prob
sol = mpc_p.solver('x0',ref_traj_v.x0,...
                   'lbx',boundray_v.lbx,...
                   'ubx',boundray_v.ubx,...
                   'lbg',boundray_v.lbg,...
                   'ubg',boundray_v.ubg,...
                   'p',ref_traj_v.p);
               
  %% Unpack data
  [x_sol, f_sol, fp_l_sol, fp_g_sol] = unpacks_sol(sol, body_params, ctr_params, path);

  %% get low-level ctr
  [low_ctr] = cal_swing_ff(x_sol, f_sol, fp_g_sol, body_params, ctr_params);

  %% plot everything, check control limits
  plot_ctr(x_sol, f_sol, low_ctr,...
           body_params, ctr_params, "stair_2");

  %% visualize
  rbt_anime(x_sol, f_sol, fp_g_sol, ref_traj_v,...
            ctr_params.T, ctr_params.N,...
            body_params, world_params, 'stair_2');
  %% save plot
  rbt_vis_overlay(x_sol, f_sol, fp_g_sol, ref_traj_v,...
                  ctr_params.T, ctr_params.N,...
                  body_params, world_params, 'stair_2.png');
