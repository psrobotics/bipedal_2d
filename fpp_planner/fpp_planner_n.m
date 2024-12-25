function [ref] = fpp_planner_n (world_p, body_p, ctr_p, gait, path, x_ref_init)
% generate ref and initial guess traj for 1 mpc cycle with target velocity
% x init - init pose, 6*1
% vel_tar_local - target input velocity in body coordinate, dot rpy, xyz, 6*1

% fpp deleted in opti target, it'll only be feeded as reference value

addpath(path.casadi);
import casadi.*;

N = ctr_p.N
dt = ctr_p.dt

% for 2d setup
state_dim = 6;
f_dim = 4;
p_dim = 4;
c_dim = 2;

% get contact seq and foot fpp
contact_ref = zeros(c_dim,N);
fpos_ref = zeros(p_dim,N);

% clk alignment
t_c = ctr_p.CLK_G;
t_start = t_c
t_end = t_start + N*dt

% previllaged terrain data
terrain_arr = world_p.terrain.map;
terrain_dx = (world_p.terrain.range(2)-world_p.terrain.range(1))/world_p.terrain.grid_size;
terrain_min = world_p.terrain.range(1);

cnt = 1;
for t_c = t_start:dt:t_end-dt
    gait.clk_g = t_c;
    clk_1 = mod(t_c, gait.t); % leg1
    clk_2 = mod(t_c + gait.t * gait.phase_diff, gait.t); % leg2
    % for each clk, t = t_flight+t_ground
    clk_t_1 = clk_1/gait.flight_t;
    clk_t_2 = clk_2/gait.flight_t;

    % clk 0~1.0 flight, contact = false
    contact_n = [(clk_t_1>1.0), (clk_t_2>1.0)];
    contact_ref(:,cnt) = contact_n;

    if clk_t_1 > 1.0
        clk_t_1 = 1.0;
    end
    if clk_t_2 > 1.0
        clk_t_2 = 1.0;
    end
    t_n = [clk_t_1; clk_t_2];

    % get x, y pos for both foot1, foot2
    x_n = gait.L .* (10.*t_n.^3 - 15.*t_n.^4 + 6.*t_n.^5);
    x_offset = [gait.x_p_1 + gait.L*floor(gait.clk_g/gait.t);...
                gait.x_p_2 + gait.L*floor((gait.clk_g+gait.t*gait.phase_diff)/gait.t)];
    fpos_x_n = x_n + x_offset;

    % get terrain index from current x_ns, x_n 1x2
    terrain_i = floor((fpos_x_n - terrain_min)./terrain_dx) + 1
    t_h = [terrain_arr(terrain_i(1));...
           terrain_arr(terrain_i(2))]
    % consider current t_h
    y_n = gait.H .* (16.*t_n.^2 .* (1.0 - t_n).^2) + t_h;
    fpos_y_n = y_n;

    fpos_ref(:,cnt) = [fpos_x_n', fpos_y_n'];

    cnt = cnt+1;
end

gait.x_p_1 =  gait.L*floor(gait.clk_g/gait.t);
gait.x_p_2 =  gait.L*floor((gait.clk_g+gait.t*gait.phase_diff)/gait.t);

% get body ref traj
x_ref = zeros(state_dim, N+1);

% clk alignment
t_c = ctr_p.CLK_G;
t_start = t_c;
t_end = t_start + N*dt;

cnt = 1;
for t_c = t_start:dt:t_end % since we have N+1
    if cnt>1
        % x position
        x_ref(1,cnt) = x_ref(1,cnt-1)+dt*ctr_p.vel_tar;
    end
    % get terrain index from current x_n
    terrain_i = floor((x_ref(1,cnt) - terrain_min)./terrain_dx) + 1;
    t_h = terrain_arr(terrain_i);
    % consider the terrain height
    x_ref(2,cnt) = ctr_p.b_h_tar + t_h; 
    % x vel
    x_ref(4,cnt) = ctr_p.vel_tar;
    cnt = cnt+1;
end

% smooth z-dir traj (with terrain height)
w_size = 35;
x_ref(2,:) = smoothdata(x_ref(2,:), 'gaussian', w_size);

%% inject current state
x_ref(:,1) = x_ref_init;

% copy out all traj ref generated
ref.x_ref = x_ref;
ref.f_ref = zeros(f_dim, N); % GRF ref
ref.fpos_ref = fpos_ref; % foot position
ref.contact_ref = contact_ref; % foot contact

% debug ----------------
%ref.contact_ref = ones(size(contact_ref));

%% recast into 3d traj
ref.x_ref_f = zeros(12, ctr_p.N+1);
ref.x_ref_f(2,:) = x_ref(3,:); % pitch
ref.x_ref_f(4,:) = x_ref(1,:); % x
ref.x_ref_f(6,:) = x_ref(2,:); % z
ref.x_ref_f(2+6,:) = x_ref(3+3,:); % dpitch
ref.x_ref_f(4+6,:) = x_ref(1+3,:); % dx
ref.x_ref_f(6+6,:) = x_ref(2+3,:); % dz

ref.f_ref_f = zeros(6, ctr_p.N);

ref.fpos_ref_f = zeros(6, ctr_p.N);
ref.fpos_ref_f(1,:) = fpos_ref(1,:); % x1
ref.fpos_ref_f(2,:) = -body_p.width/2*ones(1,ctr_p.N); % y1
ref.fpos_ref_f(3,:) = fpos_ref(3,:); % z1
ref.fpos_ref_f(4,:) = fpos_ref(2,:); % x2
ref.fpos_ref_f(5,:) = body_p.width/2*ones(1,ctr_p.N); % y2
ref.fpos_ref_f(6,:) = fpos_ref(4,:); % z2

ref.contact_ref_f = contact_ref;


% combine all ref traj, cast into 1d vec for solver
ref.p = [reshape(ref.x_ref_f, 12*(ctr_p.N+1), 1);...
         reshape(ref.f_ref_f, 6*ctr_p.N, 1);...
         reshape(ref.fpos_ref_f, 6*ctr_p.N, 1);...
         reshape(ref.contact_ref_f, 2*ctr_p.N, 1)]; % * 2 legs
% initial guess
ref.x0 = [reshape(ref.x_ref_f, 12*(ctr_p.N+1), 1);...
          reshape(ref.f_ref_f, 6*ctr_p.N, 1);...
          reshape(ref.fpos_ref_f, 6*ctr_p.N, 1)]; 


end


