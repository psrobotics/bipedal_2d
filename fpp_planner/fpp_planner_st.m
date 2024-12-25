function [ref] = fpp_planner_st (world_p, body_p, ctr_p, gait, path, x_ref_init)
% generate ref and initial guess traj for 1 mpc cycle with target velocity
% x init - init pose, 6*1
% vel_tar_local - target input velocity in body coordinate, dot rpy, xyz, 6*1

% fpp deleted in opti target, it'll only be feeded as reference value, for
% standing task

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
contact_ref = ones(c_dim,N); % standing, all 1 contact
% set static standing fpp
fpos_ref = zeros(p_dim,N);
fpos_ref(1,:) = -0.1;
fpos_ref(2,:) = 0.1;
fpos_ref(3,:) = 0.0;
fpos_ref(4,:) = 0.0;

% get body ref traj
x_ref = zeros(state_dim, N+1);
x_ref(2,:) = 0.45;
x_ref(2,50:99) = linspace(0.45, 0.55, 50);
x_ref(2,100:199) = linspace(0.55, 0.45, 100);


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


