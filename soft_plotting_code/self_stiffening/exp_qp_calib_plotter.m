
%% ========================================================================
% STATIC PRESSURE-CONFIGURATION MODEL CALIBRATION
% =========================================================================
%
% New experiment:
%   p1 = 0:10:200 hPa
%   p2 = 0:10:200 hPa
%
% Models:
%
% Linear:
%   q = beta0 + beta1*p1 + beta2*p2
%
% Quadratic:
%   q = beta0 + beta1*p1 + beta2*p2
%       + beta3*p1^2 + beta4*p1*p2 + beta5*p2^2
%
% Outputs:
%   Figure 1: All experimental data (including transients)
%   Figure 2: Equilibrium points + linear fit
%   Figure 3: Equilibrium points + quadratic fit
%
%   Linear and quadratic coefficients, R^2, and RMSE
%   Equilibrium data table
%
% =========================================================================

clear;
clc;
close all;

%% ========================================================================
% USER SETTINGS
% =========================================================================

filename = 'data/ezloophw_self_stiffening_2026-11-12_modelcalib.csv';

% Experimental pressure range [hPa]
p_min = 0;
p_max = 200;
p_step = 10;

% Static criterion [rad/s]
dq_static_tol = 0.002;

% Minimum settling period after command change [s]
min_hold_time = 5;

% Average final equilibrium measurements over this interval [s]
equilibrium_window = 2.0;

% Minimum samples required within equilibrium window
min_static_samples = 5;

% Exclude initial waiting and cooling intervals.
% This also excludes the (0,0) calibration point.
exclude_zero_command = true;

% Tolerance for small pressure measurement deviations [hPa]
pressure_tol = 2;

% Command-change tolerance [hPa]
command_tol = 1e-6;

% Fitted surface resolution
grid_resolution = 50;

%% ========================================================================
% LOAD CSV
% =========================================================================

% Logger CSV:
% Row 1-2: Metadata
% Row 3: Variable names

opts = detectImportOptions(filename, ...
    'NumHeaderLines',2, ...
    'VariableNamingRule','preserve');

data = readtable(filename,opts);

%% ========================================================================
% EXTRACT DATA
% =========================================================================

% Time [s]
time = data.("Test time");
time = time(:);

% Measured chamber pressures
p1_raw = data.("MPRpressure_0");
p2_raw = data.("MPRpressure_1");

% Configuration and angular velocity
q_rad = data.("smoothed_q_0");
dq_rad = data.("smoothed_dq_0");

% Commanded chamber pressures
u1 = data.("u_t(0)");
u2 = data.("u_t(1)");

% Ensure column vectors
p1_raw = p1_raw(:);
p2_raw = p2_raw(:);
q_rad = q_rad(:);
dq_rad = dq_rad(:);
u1 = u1(:);
u2 = u2(:);

% Convert configuration to degrees
q_deg = rad2deg(q_rad);

%% ========================================================================
% ESTIMATE PRESSURE SENSOR BASELINES
% =========================================================================

% MPR pressure sensors are assumed to measure absolute pressure.
%
% Estimate the zero-gauge baseline using zero-command samples.
%
% This follows the previous calibration methodology.

zero_command_mask = ...
    abs(u1) < command_tol & ...
    abs(u2) < command_tol & ...
    isfinite(p1_raw) & ...
    isfinite(p2_raw);

if ~any(zero_command_mask)

    error('No zero-command samples found for baseline estimation.');

end

p1_zero = median(p1_raw(zero_command_mask),'omitnan');
p2_zero = median(p2_raw(zero_command_mask),'omitnan');

% Convert measured pressures to approximate gauge pressure
p1 = p1_raw-p1_zero;
p2 = p2_raw-p2_zero;

fprintf('\n');
fprintf('====================================================\n');
fprintf('PRESSURE SENSOR BASELINES\n');
fprintf('====================================================\n');

fprintf('p1 baseline = %.6f hPa\n',p1_zero);
fprintf('p2 baseline = %.6f hPa\n',p2_zero);

%% ========================================================================
% VALID DATA MASK
% =========================================================================

valid_mask = ...
    isfinite(time) & ...
    isfinite(p1) & ...
    isfinite(p2) & ...
    isfinite(q_deg) & ...
    isfinite(dq_rad) & ...
    isfinite(u1) & ...
    isfinite(u2);

% Allow small deviations outside nominal pressure range
valid_mask = valid_mask & ...
    p1 >= (p_min-pressure_tol) & ...
    p1 <= (p_max+pressure_tol) & ...
    p2 >= (p_min-pressure_tol) & ...
    p2 <= (p_max+pressure_tol);

% All valid experimental samples, including transient motion
P1_all = p1(valid_mask);
P2_all = p2(valid_mask);
Q_all = q_deg(valid_mask);

%% ========================================================================
% IDENTIFY PRESSURE-COMMAND PLATEAUS
% =========================================================================

% The controller holds each pressure combination for a variable duration.
%
% Each change in commanded pressure defines a new plateau.
%
% Cooling intervals are also detected as zero-command plateaus.

command_change = [true; ...
    abs(diff(u1)) > command_tol | ...
    abs(diff(u2)) > command_tol];

plateau_id = cumsum(command_change);

plateau_start_idx = find(command_change);

plateau_start_time = time(plateau_start_idx);

time_since_change = ...
    time-plateau_start_time(plateau_id);

%% ========================================================================
% EXTRACT ONE EQUILIBRIUM POINT PER PRESSURE PLATEAU
% =========================================================================

% For every pressure plateau:
%
% 1. Exclude initial settling time.
% 2. Select the final equilibrium_window seconds.
% 3. Require all samples to satisfy velocity threshold.
% 4. Average measured pressures and configuration.
% 5. Store one equilibrium point.
%
% This prevents long-duration plateaus from receiving greater weight.

unique_plateaus = unique(plateau_id);

num_plateaus = length(unique_plateaus);

% Preallocate
P1_eq = nan(num_plateaus,1);
P2_eq = nan(num_plateaus,1);
Q_eq = nan(num_plateaus,1);

P1_cmd_eq = nan(num_plateaus,1);
P2_cmd_eq = nan(num_plateaus,1);

plateau_valid = false(num_plateaus,1);

%% Process plateaus

for k = 1:num_plateaus

    % Current plateau sample indices
    idx = find(plateau_id == unique_plateaus(k));

    if isempty(idx)
        continue;
    end

    % Commanded pressures
    p1_cmd = u1(idx(1));
    p2_cmd = u2(idx(1));

    % Exclude cooling and initial waiting intervals
    if exclude_zero_command && ...
            abs(p1_cmd) < command_tol && ...
            abs(p2_cmd) < command_tol

        continue;

    end

    % Plateau start and end times
    t_start = time(idx(1));
    t_end = time(idx(end));

    % Require enough time for settling and averaging
    if (t_end-t_start) < ...
            (min_hold_time+equilibrium_window)

        continue;

    end

    % Select final equilibrium window
    window_idx = idx( ...
        time(idx) >= (t_end-equilibrium_window) & ...
        time(idx) >= (t_start+min_hold_time));

    if length(window_idx) < min_static_samples
        continue;
    end

    % Require valid measurements throughout window
    if ~all(valid_mask(window_idx))
        continue;
    end

    % Require low angular velocity throughout window
    if ~all(abs(dq_rad(window_idx)) <= dq_static_tol)
        continue;
    end

    % Average measured equilibrium values
    P1_eq(k) = mean(p1(window_idx));
    P2_eq(k) = mean(p2(window_idx));
    Q_eq(k) = mean(q_deg(window_idx));

    % Retain commanded pressures for reference
    P1_cmd_eq(k) = p1_cmd;
    P2_cmd_eq(k) = p2_cmd;

    plateau_valid(k) = true;

end

%% ========================================================================
% EXTRACT VALID EQUILIBRIUM DATA
% =========================================================================

P1_static = P1_eq(plateau_valid);
P2_static = P2_eq(plateau_valid);
Q_static = Q_eq(plateau_valid);

P1_cmd_static = P1_cmd_eq(plateau_valid);
P2_cmd_static = P2_cmd_eq(plateau_valid);

fprintf('\n');
fprintf('====================================================\n');
fprintf('EQUILIBRIUM DATA EXTRACTION\n');
fprintf('====================================================\n');

fprintf('Total command plateaus: %d\n',num_plateaus);
fprintf('Valid equilibrium points: %d\n',length(Q_static));
fprintf('Rejected plateaus: %d\n', ...
    num_plateaus-length(Q_static));

if length(Q_static) < 6

    error('Insufficient equilibrium points for quadratic fitting.');

end

%% ========================================================================
% EQUILIBRIUM DATA TABLE
% =========================================================================

equilibrium_table = table( ...
    P1_cmd_static, ...
    P2_cmd_static, ...
    P1_static, ...
    P2_static, ...
    Q_static, ...
    'VariableNames', ...
    {'p1_command','p2_command', ...
     'p1_measured','p2_measured','q_eq_deg'});

fprintf('\n');
disp(equilibrium_table);

%% ========================================================================
% LINEAR LEAST-SQUARES REGRESSION
% =========================================================================

% q = beta0 + beta1*p1 + beta2*p2

X_linear = [ ...
    ones(length(Q_static),1), ...
    P1_static, ...
    P2_static];

beta_linear = X_linear\Q_static;

Q_linear_pred = X_linear*beta_linear;

SS_res_linear = sum((Q_static-Q_linear_pred).^2);
SS_tot = sum((Q_static-mean(Q_static)).^2);

if SS_tot <= 0
    error('Equilibrium configuration has zero variance.');
end

R2_linear = 1-SS_res_linear/SS_tot;

RMSE_linear = sqrt(mean((Q_static-Q_linear_pred).^2));

%% ========================================================================
% QUADRATIC LEAST-SQUARES REGRESSION
% =========================================================================

% q = beta0 + beta1*p1 + beta2*p2
%   + beta3*p1^2 + beta4*p1*p2 + beta5*p2^2

X_quad = [ ...
    ones(length(Q_static),1), ...
    P1_static, ...
    P2_static, ...
    P1_static.^2, ...
    P1_static.*P2_static, ...
    P2_static.^2];

beta_quad = X_quad\Q_static;

Q_quad_pred = X_quad*beta_quad;

SS_res_quad = sum((Q_static-Q_quad_pred).^2);

R2_quad = 1-SS_res_quad/SS_tot;

RMSE_quad = sqrt(mean((Q_static-Q_quad_pred).^2));

%% ========================================================================
% PRINT LINEAR MODEL
% =========================================================================

fprintf('\n');
fprintf('====================================================\n');
fprintf('LINEAR MODEL\n');
fprintf('====================================================\n');

fprintf('q = beta0 + beta1*p1 + beta2*p2\n\n');

fprintf('beta0 = %.9f\n',beta_linear(1));
fprintf('beta1 = %.9f\n',beta_linear(2));
fprintf('beta2 = %.9f\n',beta_linear(3));

fprintf('\n');

fprintf('q = %.9f %+.9f*p1 %+.9f*p2\n', ...
    beta_linear(1), ...
    beta_linear(2), ...
    beta_linear(3));

fprintf('\nR^2 = %.8f\n',R2_linear);
fprintf('RMSE = %.6f deg\n',RMSE_linear);

%% ========================================================================
% PRINT QUADRATIC MODEL
% =========================================================================

fprintf('\n');
fprintf('====================================================\n');
fprintf('QUADRATIC MODEL\n');
fprintf('====================================================\n');

fprintf('q = beta0 + beta1*p1 + beta2*p2\n');
fprintf('  + beta3*p1^2 + beta4*p1*p2 + beta5*p2^2\n\n');

for i = 1:6
    fprintf('beta%d = %.9f\n',i-1,beta_quad(i));
end

fprintf('\n');

fprintf('q = %.9f %+.9f*p1 %+.9f*p2\n', ...
    beta_quad(1), ...
    beta_quad(2), ...
    beta_quad(3));

fprintf('  %+.9f*p1^2 %+.9f*p1*p2 %+.9f*p2^2\n', ...
    beta_quad(4), ...
    beta_quad(5), ...
    beta_quad(6));

fprintf('\nR^2 = %.8f\n',R2_quad);
fprintf('RMSE = %.6f deg\n',RMSE_quad);

%% ========================================================================
% GENERATE FITTED SURFACES
% =========================================================================

p1_range = linspace(p_min,p_max,grid_resolution);
p2_range = linspace(p_min,p_max,grid_resolution);

[P1_grid,P2_grid] = meshgrid(p1_range,p2_range);

% Linear plane
Q_linear_grid = ...
    beta_linear(1) ...
    + beta_linear(2)*P1_grid ...
    + beta_linear(3)*P2_grid;

% Quadratic surface
Q_quad_grid = ...
    beta_quad(1) ...
    + beta_quad(2)*P1_grid ...
    + beta_quad(3)*P2_grid ...
    + beta_quad(4)*P1_grid.^2 ...
    + beta_quad(5)*P1_grid.*P2_grid ...
    + beta_quad(6)*P2_grid.^2;

%% ========================================================================
% FIGURE 1: ALL EXPERIMENTAL DATA
% =========================================================================

figure( ...
    'Name','All Experimental Data', ...
    'Color','w');

scatter3( ...
    P1_all, ...
    P2_all, ...
    Q_all, ...
    8, ...
    Q_all, ...
    'filled', ...
    'MarkerFaceAlpha',0.15, ...
    'MarkerEdgeAlpha',0.15);

grid on;
box off;
view(3);

xlabel('$p_1$ (hPa)', ...
    'Interpreter','latex', ...
    'FontSize',14);

ylabel('$p_2$ (hPa)', ...
    'Interpreter','latex', ...
    'FontSize',14);

zlabel('$q_1$ (deg)', ...
    'Interpreter','latex', ...
    'FontSize',14);

title('All Pressure--Configuration Measurements', ...
    'Interpreter','latex', ...
    'FontSize',16);

colorbar;

set(gca, ...
    'TickLabelInterpreter','latex', ...
    'FontSize',12);

%% ========================================================================
% FIGURE 2: EQUILIBRIUM DATA + LINEAR FIT
% =========================================================================

figure( ...
    'Name','Linear Static Model', ...
    'Color','w');

hold on;

h_data = scatter3( ...
    P1_static, ...
    P2_static, ...
    Q_static, ...
    20, ...
    'b', ...
    'filled', ...
    'MarkerFaceAlpha',0.5, ...
    'MarkerEdgeAlpha',0.5);

h_fit = surf( ...
    P1_grid, ...
    P2_grid, ...
    Q_linear_grid, ...
    'FaceAlpha',0.35, ...
    'EdgeColor','none');

hold off;

grid on;
box off;
view(3);

xlabel('$p_1$ (hPa)', ...
    'Interpreter','latex', ...
    'FontSize',14);

ylabel('$p_2$ (hPa)', ...
    'Interpreter','latex', ...
    'FontSize',14);

zlabel('$q_1$ (deg)', ...
    'Interpreter','latex', ...
    'FontSize',14);

title( ...
    sprintf('Linear Static Model, $R^2 = %.4f$',R2_linear), ...
    'Interpreter','latex', ...
    'FontSize',16);

legend([h_data,h_fit], ...
    {'Equilibrium measurements','Linear fit'}, ...
    'Interpreter','latex', ...
    'Location','best');

set(gca, ...
    'TickLabelInterpreter','latex', ...
    'FontSize',12);

%% ========================================================================
% FIGURE 3: EQUILIBRIUM DATA + QUADRATIC FIT
% =========================================================================

figure( ...
    'Name','Quadratic Static Model', ...
    'Color','w');

hold on;

h_data = scatter3( ...
    P1_static, ...
    P2_static, ...
    Q_static, ...
    20, ...
    'b', ...
    'filled', ...
    'MarkerFaceAlpha',0.5, ...
    'MarkerEdgeAlpha',0.5);

h_fit = surf( ...
    P1_grid, ...
    P2_grid, ...
    Q_quad_grid, ...
    'FaceAlpha',0.35, ...
    'EdgeColor','none');

hold off;

grid on;
box off;
view(3);

xlabel('$p_1$ (hPa)', ...
    'Interpreter','latex', ...
    'FontSize',14);

ylabel('$p_2$ (hPa)', ...
    'Interpreter','latex', ...
    'FontSize',14);

zlabel('$q_1$ (deg)', ...
    'Interpreter','latex', ...
    'FontSize',14);

title( ...
    sprintf('Quadratic Static Model, $R^2 = %.4f$',R2_quad), ...
    'Interpreter','latex', ...
    'FontSize',16);

legend([h_data,h_fit], ...
    {'Equilibrium measurements','Quadratic fit'}, ...
    'Interpreter','latex', ...
    'Location','best');

set(gca, ...
    'TickLabelInterpreter','latex', ...
    'FontSize',12);

%% ========================================================================
% MODEL COMPARISON
% =========================================================================

fprintf('\n');
fprintf('====================================================\n');
fprintf('MODEL COMPARISON\n');
fprintf('====================================================\n');

model_names = {'Linear';'Quadratic'};

R_squared = [R2_linear;R2_quad];
RMSE_deg = [RMSE_linear;RMSE_quad];
num_coefficients = [3;6];

comparison_table = table( ...
    model_names, ...
    num_coefficients, ...
    R_squared, ...
    RMSE_deg, ...
    'VariableNames', ...
    {'Model','Coefficients','R_squared','RMSE_deg'});

disp(comparison_table);

%% ========================================================================
% OPTIONAL: SAVE CALIBRATION RESULTS
% =========================================================================

% Uncomment to save extracted equilibrium data:
%
% writetable(equilibrium_table, ...
%     'quadratic_calibration_equilibrium_points.csv');

% Uncomment to save identified models:
%
% save('quadratic_calibration_models.mat', ...
%     'beta_linear','beta_quad', ...
%     'R2_linear','R2_quad', ...
%     'RMSE_linear','RMSE_quad', ...
%     'equilibrium_table');


