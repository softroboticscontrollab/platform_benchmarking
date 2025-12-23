%% Soft 2-link planar robot with pressure-based nominal control (CBF-ready)
clear all; close all; clc;

% Paths (modify if needed)
basedir = '..';
addpath(genpath(basedir));
rmpath(genpath(strcat(basedir, '\.git')));
addpath(genpath('../../helpers'));
addpath(genpath('../../plotting_soft'));
addpath(genpath('../../polyhedron_constraints'));

disp('Running simulation with pressure-based nominal control input...');

%% Load and preprocess pressure data
% data = readtable('ezloophw_closedloop_ros2_pneumatics_2025-4-8_133407.csv', ...
%     'HeaderLines', 2, 'VariableNamingRule', 'preserve');

data = readtable('ezloophw_closedloop_ros2_pneumatics_2025-4-9_110543.csv', ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');

time     = data.("Test time");
theta_0  = data.theta_0;
theta_1  = data.theta_1;
mpr_0    = data.MPRpressure_0;
mpr_1    = data.MPRpressure_1;
mpr_2    = data.MPRpressure_2;
mpr_3    = data.MPRpressure_3;

% Convert angles from degrees to radians
theta_0 = deg2rad(theta_0);
theta_1 = deg2rad(theta_1);

time_data = data.("Test time");
smoothed_theta_0 = theta_0;
smoothed_theta_1 = theta_1;
smoothed_mpr_0 = smoothdata(mpr_0, 'sgolay', 12);
smoothed_mpr_1 = smoothdata(mpr_1, 'sgolay', 12);
smoothed_mpr_2 = smoothdata(mpr_2, 'sgolay', 12);
smoothed_mpr_3 = smoothdata(mpr_3, 'sgolay', 12);

%% Simulation parameters and constants
tmax = 1; dt = 0.0001; n = round(tmax/dt);

m2 = 0.13; m6 = 0.13;
damping = 10.0;
% k1 = 119.81792; k2 = 159.20054;
% k1 = 133.80554; k2 = 201.62900;
k1 = 130.85249; k2 = 165.12965;
l1 = 0.122; l2 = 0.122;
g = 9.81; tol = 1e4;

F_max = 10; k_env = 10000;

% aE = 35; bE = 35; gam = 70;
% aE = 100; bE = 100; gam = 500;
% aE = 50; bE = 50; gam = 100;
% aE = 32; bE = 32; gam = 60;
% aE = 60; bE = 30; gam = 60;
% aE = 60; bE = 60; gam = 30;
% aE = 70; bE = 80; gam = 150;
% aE = 50; bE = 30; gam = 1;
aE = 10; bE = 1; gam = 1;


% aE = 10; bE = 10; gam = 1;
% aE = 8; bE = 8; gam = 0.4;


% constants for the dynamics
c.m2 = m2;
c.m6 = m6;
c.damping = damping;
c.k1 = k1;
c.k2 = k2;
c.g = g;
c.tol = tol;
c.l1 = l1;
c.l2 = l2;

c.k_env = k_env;
c.F_max = F_max;

c.aE = aE;
c.bE = bE;
c.gam = gam;

%% Initial conditions
theta1_0 = theta_0(1); 
theta2_0 = theta_1(1);
dottheta1_0 = 0; dottheta2_0 = 0;
x0 = [theta1_0; theta2_0; dottheta1_0; dottheta2_0];
x_traj = zeros(4, n+1); x_traj(:,1) = x0;
u_traj = zeros(2, n); 

u_offset = [1135.0, 1130.0, 1125.0, 1130.0];
p1 = mpr_0 - u_offset(1) - (mpr_1 - u_offset(2));
p2 = mpr_2 - u_offset(3) - (mpr_3 - u_offset(4));

p_raw = [p1, p2]';

lambda = diag([1.0, 1.0]);

% Interpolate pressure signals to match simulation time steps
t_vec_sim = (0:dt:tmax);
% Remove duplicate time entries
[time_unique, unique_idx] = unique(time_data, 'first');
p_unique = p_raw(:, unique_idx);

% Interpolate to match simulation time
p_interp = interp1(time_unique, p_unique', t_vec_sim, 'linear', 'extrap')';  % [2 x n+1]


%% Simulate
disp('Simulation time (seconds):')
for t = 1:n
    if mod(t, 1/dt) == 0
        disp(string(t*dt));
    end

    % Get nominal input from pressure data at this time
    p_nom_t = p_interp(:, t);
    u_nom_t = lambda * p_nom_t;

    % Optionally apply safety constraints:
    u_t = soft_u_cbf_polygonal_safeset(u_nom_t, x_traj(:,t), c);
    % u_t = u_nom_t;
    u_traj(:, t) = u_t;

    [~, bolddotx_t] = dynamics_soft(x_traj(:,t), c, u_t);

    % Forward Euler integration
    x_traj(:,t+1) = x_traj(:,t) + dt * bolddotx_t;

    % Safety check for instability
    if any(abs(x_traj(:,t+1)) > tol)
        warning("Simulation unstable at t = %.4f sec", t*dt);
        x_traj = x_traj(:,1:t);  
        break;
    end
end



%% Plot
% Extract joint angles
q1 = x_traj(1, :);
q2 = x_traj(2, :);

% Avoid division by zero (use small epsilon)
epsilon = 1e-6;
q1_safe = q1;
q2_safe = q2;
q1_safe(q1 == 0) = epsilon;
q2_safe(q2 == 0) = epsilon;

% Compute end-effector positions using element-wise operations
x_r = c.l2 * ((cos(q1) .* sin(q2) + sin(q1) .* cos(q2) - sin(q1)) ./ q2_safe) ...
    + c.l1 * (sin(q1) ./ q1_safe);

y_r = c.l2 * ((sin(q1) .* sin(q2) - cos(q1) .* cos(q2) + cos(q2)) ./ q2_safe) ...
    + c.l1 * ((1 - cos(q1)) ./ q1_safe);

time = linspace(0, tmax, size(x_traj, 2));

%% Save outputs to .mat files

% Create an identifier string for current parameters
param_id = sprintf('aE%.0f_bE%.0f_gam%.0f', aE, bE, gam);

% Define file names
save_dir = 'sim_outputs'; % Optional: make sure this folder exists
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

% Save variables
save(fullfile(save_dir, ['x_r_' param_id '.mat']), 'x_r');
save(fullfile(save_dir, ['y_r_' param_id '.mat']), 'y_r');
save(fullfile(save_dir, ['time_' param_id '.mat']), 'time');
u_t = u_traj(:, 1:size(x_traj, 2) - 1);
save(fullfile(save_dir, ['u_t_' param_id '.mat']), 'u_t');


% figure;
% plot(time, x_r, 'b', 'LineWidth', 1.5); hold on;
% plot(time, y_r, 'r', 'LineWidth', 1.5);
% xlabel('Time [s]');
% ylabel('End-Effector Position [m]');
% legend('x_r', 'y_r');
% title('Soft Robot End-Effector Position vs Time');
% grid on;
% 
% time_u = linspace(0, tmax - dt, n);  % or:  time_u = dt * (0:n-1);
% figure;
% plot(time_u, u_traj(1, :), 'r', 'LineWidth', 1.5); hold on;
% plot(time_u, u_traj(2, :), 'b', 'LineWidth', 1.5);
% xlabel('Time [s]');
% ylabel('Control Input [N]');
% legend('u_1', 'u_2');
% title('Control Inputs over Time');
% grid on;

% Soft_CBF_plotting(x_traj(1:2, :),[c.l1, c.l2],V,F_max,k_env);



% (C) Soft Robotics Control Lab, Boston University, 2024-

% Simulate a two-link planar robot, per the Siciliano book, with a CBF
% setting a maximum value for the q1 angle.
% Nominal controller is the sinusoidal motor babbling.




