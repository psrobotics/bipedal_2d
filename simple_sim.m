sim_t = 1.0;
dt = 0.01;
t_n = 0.0;

x0 = [0.0; 0.5; 0.0; 0.0; 0.0; 0.0]; % x theta dx dtheta
f0 = [0.0; 0.0; 0.0; 0.0];
fpos0 = [-0.1; 0.1; 0.0; 0.0];

x_seq = x0;
f_seq = f0;
t_seq = t_n;

%% Get hardware params & set package path
[world_p, body_p, ctr_p, gait, path] = hardware_params();

addpath(path.casadi);
import casadi.*;

%% Get dynamics
[dyn_f] = get_srb_dynamics(world_p, body_p, path);

%% Form the mpc problem
[mpc_v, mpc_c, mpc_p] = form_mpc_prob(world_p, body_p, ctr_p, dyn_f, path);
%% set up boundry
[boundray_v] = add_state_boundaries(mpc_v, mpc_c, world_p, body_p, ctr_p, path);

%% sim process

while t_n<sim_t

    t_span = [t_n, t_n+dt];
    % get current state
    x_n = x_seq(:,end);

    [ref_traj_v] = fpp_planner_n(body_p, ctr_p, gait, path, x_n);

    ref_traj_v.contact_ref_f = ones(size(ref_traj_v.contact_ref_f));

    % solve
    sol = mpc_p.solver('x0',ref_traj_v.x0,...
                       'lbx',boundray_v.lbx,...
                       'ubx',boundray_v.ubx,...
                       'lbg',boundray_v.lbg,...
                       'ubg',boundray_v.ubg,...
                       'p',ref_traj_v.p);
    % get result
    [x_sol, f_sol, fp_l_sol, fp_g_sol] = unpacks_sol(sol, body_p, ctr_p, path);
    fp_l_sol;
    fp_g_sol;
    f_sol;
    % first control
    f_1 = f_sol(:,1);
    f_1_2d = [f_1(1); f_1(4); f_1(3); f_1(6)];

    fp_1 = fp_g_sol(:,1);
    fp_1_2d = [fp_1(1); fp_1(4); fp_1(3); fp_1(6)];

    % sim next step
    %[t,y] = ode45(@(t,y) rbt_s_dyn(t, x_n, f_1_2d, fp_1_2d, [1; 1]), t_span, x_n);

    %x_next = y(end,:)';
    x_f_n = [0;x_n(3);0; x_n(1);0;x_n(2); 0;x_n(3+3);0; x_n(1+3);0;x_n(2+3)];
    x_f_next = x_f_n + dyn_f(x_f_n, f_1, fp_1)*dt;
    x_f_next = full(x_f_next);

    x_next = [x_f_next(4); x_f_next(6); x_f_next(2);x_f_next(4+6); x_f_next(6+6); x_f_next(2+6)];

    x_seq = [x_seq, x_next];
    f_seq = [f_seq, f_1_2d];

    t_n = t_n + dt;
    t_seq = [t_seq; t_n];

end

%%
figure;
subplot(5, 1, 1);
plot(t_seq, x_seq(1, :),'LineWidth',1.2);
xlabel('Time (s)');
ylabel('x pos (m)');

subplot(5, 1, 2);
plot(t_seq, x_seq(2, :),'LineWidth',1.2);
xlabel('Time (s)');
ylabel('y pos (m)');

subplot(5, 1, 3);
plot(t_seq, x_seq(3, :),'LineWidth',1.2);
xlabel('Time (s)');
ylabel('theta');

subplot(5, 1, 4);
plot( f_sol(4, :),'LineWidth',1.2);
xlabel('Time (s)');
ylabel('fz1');

subplot(5, 1, 5);
plot( f_sol(5, :),'LineWidth',1.2);
xlabel('Time (s)');
ylabel('fz2');
%%
% plot(t_seq, f_seq(1, :),'LineWidth',1.2);
% xlabel('Time (s)');
% ylabel('f N');


%%
%animate(x_seq, t, "rbt_body", 'r-');

%%
function dxdt = rbt_s_dyn(t, x_n, f_n, fpos_n, contact)

% physical params
I = 0.567;
M = 8.0;
l = 0.22;
g = 9.81;

% get fpos
p1x = fpos_n(1);
p1y = fpos_n(3);
p2x = fpos_n(2);
p2y = fpos_n(4);

c1 = contact(1);
c2 = contact(2);

% srb dynamic model
A = [0 0 0 1 0 0;...
     0 0 0 0 1 0;...
     0 0 0 0 0 1;...
     0 0 0 0 0 0;...
     0 0 0 0 0 0;...
     0 0 0 0 0 0];

B = [0,           0,           0,           0;...
     0,           0,           0,           0;...
     0,           0,           0,           0;...
     1/M,         1/M,         0,           0;...
     0,           0,           1/M,         1/M;...
     -p1y*c1/I,   -p2y*c2/I,   p1x*c1/I,    p2x*c2/I];

G = [0; 0; 0; 0; -g; 0];

dxdt = A*x_n + B*f_n + G;

end

%%
function animate(x_seq, t, name, color)

% video setup
fps = 52*2;

writer = VideoWriter(name, 'MPEG-4');
writer.FrameRate = fps;  
writer.Quality = 100;
open(writer);  

x_s = x_seq(1, :);  % x
y_s = x_seq(2, :);  % y
theta_s = x_seq(3, :); % pitch 

a = 0.15;
b = 0.25;
cart_frame = [-a/2, -a/2,   a/2,   a/2,   -a/2;  
              0,     0,     0,     0,      0;  
              b/2,  -b/2,   -b/2,  b/2,    b/2];

pole_frame = [-0.005, -0.005, 0.005,  0.005,   -0.005;  
              0,      0,      0,      0,       0;  
              0.3,    0,      0,      0.3,     0.3];

figure;
hold on;
axis equal;
grid on;

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
    
for i = 1:length(x_seq)

    x = x_s(i);
    y = y_s(i);
    theta = -1*theta_s(i);
        
    R_bi = eul2rotm([0.0, theta, 0.0]);
    cart_transformed =  R_bi*cart_frame;
        
    % body's frame
    cart_plot = plot3(cart_transformed(1, :) + x, ...
                      cart_transformed(2, :)  , ...
                      cart_transformed(3, :) + y, 'k-', 'LineWidth', 1.2);
      
    view([0, 0]);
  
    xlim([-0.5, 0.5]);
    ylim([-0.1, 0.1]);
    zlim([-0.1, 1.6]);

    frame = getframe(gcf);
    writeVideo(writer, frame);

    pause(0.01);

    if i < length(x_seq)
        delete(cart_plot);
    end
end

close(writer);

end
