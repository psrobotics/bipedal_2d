% Terrain parameters
world.terrain.range = [-20, 20]; % in meters
world.terrain.grid_size = 400;   % Number of grid points
grid_step = (world.terrain.range(2) - world.terrain.range(1)) / world.terrain.grid_size; % Grid step size

% Terrain height: Example with variable heights
world.terrain.stair = zeros(1, world.terrain.grid_size);
world.terrain.stair(10:100) = 0.2;  % 0.2m height
world.terrain.stair(150:200) = 0.5; % 0.5m height
world.terrain.stair(250:300) = -0.5; % -0.5m height

% X-axis positions (terrain grid)
x_terrain = linspace(world.terrain.range(1), world.terrain.range(2), world.terrain.grid_size);

% Define the "below level" and create masks
below_level = -1; % Define level below which to highlight
above_mask = world.terrain.stair > below_level; % Logical mask for valid terrain

% Plot the terrain
figure;
hold on;

% 1. Highlight the region below the terrain (Fill area below current terrain height)
fill([x_terrain, fliplr(x_terrain)], ...
     [world.terrain.stair, below_level*ones(size(world.terrain.stair))], ...
     [0.8, 0.8, 1.0], 'EdgeColor', 'none'); % Light blue color

% 3. Plot the actual terrain
plot(x_terrain, world.terrain.stair, 'k-', 'LineWidth', 2);

% Add labels and grid
title('Terrain with Highlighted Regions');
xlabel('X Position (meters)');
ylabel('Height (meters)');
ylim([-1.5, 1.5]); % Adjust Y-axis range
grid on;
hold off;

