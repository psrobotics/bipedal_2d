% MPC for 2D Biped Balancing
clear; clc;

% Parameters
dt = 0.1;              % Time step (s)
N = 10;                % MPC horizon
m = 50;                % Mass of the body (kg)
g = 9.81;              % Gravity (m/s^2)
h = 1.0;               % Height of center of mass (CoM) (m)
mu = 0.6;              % Friction coefficient (Coulomb friction)
Fz_max = m * g * 1.5;  % Max vertical force per leg
x_init = [0; 0; 0; 0]; % Initial state: [x; z; vx; vz] (position and velocity)
foot_pos = [-0.5, 0.5];% Foot positions on the ground (left and right feet, x-coordinates)

% State and Input Dimensions
nx = 4;   % State: [x; z; vx; vz]
nu = 4;   % Input: [Fx1; Fz1; Fx2; Fz2]

% MPC Problem Setup
A = [1, 0, dt,  0;  % Discrete-time dynamics (linearized around CoM)
     0, 1,  0, dt;
     0, 0,  1,  0;
     0, 0,  0,  1];
B = [0,    0,    0,    0;
     0,    0,    0,    0;
     dt/m, 0,  dt/m,  0;
     0, dt/m,   0, dt/m];

% Cost Function (Quadratic)
Q = diag([10, 100, 1, 1]);  % State cost weights
R = diag([0.01, 0.01, 0.01, 0.01]); % Input cost weights

% Constraints
u_min = [-mu*Fz_max; 0; -mu*Fz_max; 0];   % Min input [Fx1; Fz1; Fx2; Fz2]
u_max = [mu*Fz_max; Fz_max; mu*Fz_max; Fz_max]; % Max input
x_ref = [0; h; 0; 0]; % Reference CoM position and velocity

% Initialize Variables
x = x_init; % Current state
U_opt = zeros(nu, N); % Optimal control input over horizon
sim_time = 5; % Simulation time (s)
n_steps = sim_time / dt;

% Simulation Loop
for t = 1:n_steps
    % Build MPC Quadratic Programming (QP) problem
    H = blkdiag(kron(eye(N), R), kron(eye(N+1), Q)); % Quadratic cost
    f = zeros(N*nu + (N+1)*nx, 1);                  % Linear cost
    
    % Constraints
    Aeq = []; beq = [];
    Aineq = []; bineq = [];
    
    % Dynamics constraints (equality)
    for k = 1:N
        Aeq_k = [zeros(nx, (k-1)*nu), -B, eye(nx), zeros(nx, (N-k)*nx)];
        Aeq = [Aeq; Aeq_k];
        beq = [beq; -A*x]; % Linear dynamics constraint
    end
    
    % Input constraints (inequality)
    for k = 1:N
        Aineq_k = [zeros(nu, (k-1)*nu), eye(nu), zeros(nu, (N-k)*nu)];
        Aineq = [Aineq; Aineq_k];
        bineq = [bineq; u_max; -u_min];
    end
    
    % Solve QP
    options = optimoptions('quadprog', 'Display', 'off');
    H_final = 2 * H; % Adjusted for quadprog format
    U_opt = quadprog(H_final, f, Aineq, bineq, Aeq, beq, [], [], [], options);
    
    % Apply First Control Input
    u = U_opt(1:nu);
    Fx1 = u(1); Fz1 = u(2);
    Fx2 = u(3); Fz2 = u(4);
    
    % Update State Using Dynamics
    x = A*x + B*u;
    
    % Display Results
    fprintf('Step %d: x = %.2f, z = %.2f, Fx1 = %.2f, Fz1 = %.2f, Fx2 = %.2f, Fz2 = %.2f\n', ...
        t, x(1), x(2), Fx1, Fz1, Fx2, Fz2);
    
    % Visualization
    clf;
    plot([-0.5, 0.5], [0, 0], 'ks-', 'LineWidth', 2); hold on; % Ground and foot positions
    plot(x(1), x(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % CoM position
    xlim([-1, 1]); ylim([-0.1, 2]);
    title('2D Bipedal Balancing with MPC');
    xlabel('X'); ylabel('Z');
    pause(0.1);
end