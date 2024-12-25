function [world, body, ctr, gait, path] = hardware_params()
%% Casadi path
% Change to your casadi path
path.casadi = 'casadi-3.6.7-osx_arm64-matlab2018b/';

%% Simulation params
world.fk = 0.5; % friction
world.g = 9.81; % gravity constant

world.friction_cone = [1/world.fk, 0 -1;...
                      -1/world.fk, 0 -1;...
                      0, 1/world.fk, -1;...
                      0, -1/world.fk, -1];

% terrain info
world.terrain.range = [-20, 20]; % in meter
world.terrain.grid_size = 400; % 1 grid - 1cm

% simple terrain grid
world.terrain.stair = zeros(1, world.terrain.grid_size);
world.terrain.stair(205:207) = 0.1; % 0.1m in height
world.terrain.stair(207:209) = 0.2;
world.terrain.stair(209:211) = 0.3;
world.terrain.stair(211:213) = 0.4;
world.terrain.stair(213:400) = 0.5;

world.terrain.flat = zeros(1, world.terrain.grid_size);

world.terrain.blk = zeros(1, world.terrain.grid_size);
world.terrain.blk(220:230) = 0.4; % 0.1m in height
world.terrain.blk(250:254) = -0.4;

world.terrain.map = world.terrain.stair;


%% Controller params
% ctr.phase_num = 4;
% ctr.N = 10*10; % mpc period window
% ctr.T = 0.2*10; % mpc period time
% ctr.dt_val = (ctr.T/ctr.N) .* ones(ctr.N,1); % dt vector
% 
ctr.max_jump_z = 0.95; % max jumping height, constraints
ctr.min_dump_z = 0.15; % min standing height
ctr.max_lift_vel_z = 6.5; % max jumping velocity
ctr.init_z = 0.3;
% 
% ctr.x_init_tar_val = [0; 0; 0; 0; 0; ctr.init_z]; % init state
% ctr.dx_init_tar_val = [0; 0; 0; 0; 0; 0]; % init d state
% ctr.x_final_tar_val = [0; 0; 0; 0.05; 0; 0.41]; % final target state r p y; x y z
% ctr.dx_final_tar_val = [0; 0; 0; 0; 0; 0];
% 
% %ctr.contact_state_ratio = ctr.N.*[0.35 0.15 0.475 0.025]; % pull, jump, flight, impact
% ctr.contact_state_ratio = ctr.N.*[1/48 1/12 1/12 1/12 1/12 1/12 1/12 1/12 1/12 1/12 1/12 1/12]; % pull, jump, flight, impact
% 
% % ctr.contact_state_val = [ones(2, ctr.contact_state_ratio(1)),...
% %                              0 * ones(2, ctr.contact_state_ratio(2)),...
% %                              ones(2, ctr.contact_state_ratio(3)),...
% %                              0 * ones(2, ctr.contact_state_ratio(4)),...
% %                              ones(2, ctr.contact_state_ratio(1)),...
% %                              0 * ones(2, ctr.contact_state_ratio(2)),...
% %                              ones(2, ctr.contact_state_ratio(3)),...
% %                              0 * ones(2, ctr.contact_state_ratio(4)),...
% %                              ones(2, ctr.contact_state_ratio(1)),...
% %                              0 * ones(2, ctr.contact_state_ratio(2)),...
% %                              ones(2, ctr.contact_state_ratio(3)),...
% %                              0 * ones(2, ctr.contact_state_ratio(4))]; % no foot contact during last 2 phases
% 
% ctr.gait_num = 1;
% % ctr.contact_state_val = [repmat([1;0], 1, ctr.contact_state_ratio(1)),...
% %                          repmat([0;0], 1, ctr.contact_state_ratio(1)),...
% %                          repmat([0;1], 1, ctr.contact_state_ratio(1)),...
% %                          repmat([0;0], 1, ctr.contact_state_ratio(1))];
% ctr.contact_state_val = [repmat([1;1], 1, 2),...
%                          repmat([1;1], 1, 2),...
%                          repmat([1;1], 1, 2),...
%                          repmat([1;1], 1, 2)];
% 
% ctr.contact_state_val = [repmat([1;1], 1, ctr.N)];
% ctr.contact_state_val = repmat(ctr.contact_state_val, 1, ctr.gait_num);
% % no foot contact during last 2 phases

%% new Controller params
ctr.N = 500; % mpc period window 30
ctr.dt = 0.01; % mpc dt 0.1
ctr.T = ctr.N*ctr.dt; % mpc period time

ctr.vel_tar = 0.45;
ctr.x_p_init = [0.0; 0.45; 0.0]; % pos init
ctr.b_h_tar = 0.45; % body height target

ctr.CLK_G = 0.0; % global clock

% cost gains
ctr.weight.QX = 500.*[0 10 0, 10 0 10*9, 0 10 0, 10 0 10 ]'; % state error
ctr.weight.QN = 500.*[0 10 0, 50 0 50*9, 0 10 0, 10 0 10 ]'; % state error, final
ctr.weight.Qc = 1000*[50 50 50]'; % foot placement error on 3 axis
ctr.weight.Qf = 1e-3.*[0.1 0.1 0.1]'; % input error on 3 axis

%% casadi solver settings
ctr.opt_setting.expand =true;
ctr.opt_setting.ipopt.max_iter=1500;
ctr.opt_setting.ipopt.print_level=0;
ctr.opt_setting.ipopt.acceptable_tol=1e-4;
ctr.opt_setting.ipopt.acceptable_obj_change_tol=1e-6;
ctr.opt_setting.ipopt.tol=1e-4;
ctr.opt_setting.ipopt.nlp_scaling_method='gradient-based';
ctr.opt_setting.ipopt.constr_viol_tol=1e-3;
ctr.opt_setting.ipopt.fixed_variable_treatment='relax_bounds';

%% new gait params
gait.f = 2.8; % gait frequency
gait.t = 1.0/gait.f;
% time for ground/flight phase, 0~1
gait.flight_ph = 0.55;
gait.ground_ph = 1.0-gait.flight_ph;
gait.flight_t = gait.t*gait.flight_ph;

gait.phase_diff = 0.5; % 0~1
gait.L = gait.t * ctr.vel_tar/1.0; % gait length
gait.H = 0.20; % gait height

gait.clk_g = 0.0; % global clock

gait.x_p_1 = 0.0; % where we start rollout foot pos
gait.x_p_2 = gait.x_p_1 + gait.phase_diff*gait.L/3.5; % ori 1.5

%% Robot params
body.state_dim = 12; % number of dim of state, rpy xyz dot_rpy dot_xyz
body.f_dim = 6; % number of dim of leg force, 3*2, 2 leg
body.fp_dim = 6; % number of dim of leg pos, 3*2, 2 leg


body.m = 8;
body.i_vec = [0.0567 0.567 0.0567]*2;
body.i_mat = [body.i_vec(1) 0 0;... % roll
              0 body.i_vec(2) 0;... % pitch
              0 0 body.i_vec(3)]; % yaw
       
%body.length = 0.34;
body.height = 0.25; % h over z
body.width_x = 0.15; % width over x
body.width = 0.01; % distance between 2 legs over y

% leg length
body.l1 = 0.22;
body.l2 = 0.22;

% foot motion range, in m
% mod 1216, smaller y range for 2d case, y axis locked
body.foot_x_range = 0.30;
body.foot_y_range = 0.10;
body.foot_z_range = 0.30;

% output force range
body.max_zforce = 180;

% calaute 2 hip positions
body.hip_vec = [0; body.width/2; 0];
body.hip_dir_mat = [0 0; 1 -1; 0 0]; % 3,2 hip vec
body.hip_pos = body.hip_dir_mat .* repmat(body.hip_vec,1,2); % 3,2 hip pos vec
body.foot_pos = repmat([0; 0; -0.6*ctr.init_z],1,2); % init foot pos

body.phip_swing_ref = body.hip_pos + body.foot_pos;
% ref foot pos at swing phase

body.phip_swing_ref_vec = reshape(body.phip_swing_ref,[],1);

% the range foot can move within
body.foot_convex_hull = [1 0 0 -body.foot_x_range;
                        -1 0 0 -body.foot_x_range;
                         0 1 0 -body.foot_y_range;
                         0 -1 0 -body.foot_y_range;
                         0 0 1 -ctr.min_dump_z;
                         0 0 -1 -body.foot_z_range];
                

end