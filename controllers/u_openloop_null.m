
function u = u_openloop_null(x,c,t)

%% ========================================================================
% EXPERIMENT SETTINGS
% =========================================================================

% Desired equilibrium configuration [deg]
q_des = 40;

% Null-space sweep settings
z_step = 10;          % Increment between z values [hPa]
num_z_values = 10;     % Number of distinct z values
repetitions = 2;      % Repetitions of each z value

% Reset configuration [deg]
q_reset = 0;

% Null-space input during reset [hPa]
z_reset = 30;

% Timing [s]
initial_wait = 5;
hold_time    = 60;
reset_time   = 20;

% Pressure limits [hPa gauge]
p_min = 0;
p_max = 250;

%% ========================================================================
% IDENTIFIED STATIC MODEL
% =========================================================================

% q_deg = beta0 + beta1*p1 + beta2*p2

beta0 = -1.264908;   % [deg]
beta1 = 0.423700;    % [deg/hPa]
beta2 = -0.268104;   % [deg/hPa]

B = [beta1 beta2];

if norm(B) < 1e-12
    error('Identified B matrix is approximately zero.');
end

% Unit null-space vector
N = [-B(2); B(1)] / norm(B);

% This sweep increases both chamber pressures with z
if all(N < 0)
    N = -N;
end

if any(N <= 0)
    error('Expected a positive null-space direction.');
end

%% ========================================================================
% PRESSURE ALLOCATION
% =========================================================================

% Nominal pressure combination for desired configuration
p_nominal = pinv(B)*(q_des-beta0);

% Minimum z required to satisfy lower pressure limits
z_min = max((p_min-p_nominal)./N);

% Round UP to nearest multiple of z_step
z_start = ceil(z_min/z_step)*z_step;

% Generate distinct z values
z_unique = z_start + (0:num_z_values-1)*z_step;

% Repeat each z value consecutively
z_values = repelem(z_unique,repetitions);

% Each column represents one experimental pressure combination
P = p_nominal + N*z_values;

% Check feasibility of each trial
valid = all(P >= p_min-1e-9 & P <= p_max+1e-9,1);

% Keep only feasible pressure combinations
P = P(:,valid);
z_valid = z_values(valid);

num_trials = length(z_valid);

if num_trials == 0
    error('No feasible null-space pressure combinations.');
end

%% ========================================================================
% RESET PRESSURE ALLOCATION
% =========================================================================

% Pressure combination corresponding to q_reset
p_reset_nominal = pinv(B)*(q_reset-beta0);

% Add the prescribed reset null-space input
p_reset = p_reset_nominal + N*z_reset;

% Verify reset pressure limits
if any(p_reset < p_min) || any(p_reset > p_max)
    error('Reset pressure combination exceeds limits.');
end

%% ========================================================================
% EXPERIMENT TIMING
% =========================================================================

u = zeros(4,1);

% Initial waiting period
if t < initial_wait
    u(1:2) = p_reset;
    return;
end

% Time since initial waiting period
t_exp = t - initial_wait;

% Each trial includes a hold and reset interval
trial_duration = hold_time + reset_time;

% Current trial index
trial_idx = floor(t_exp/trial_duration) + 1;

% Experiment completed
if trial_idx > num_trials
    u(1:2) = p_reset;
    return;
end

% Time within current trial
t_trial = mod(t_exp,trial_duration);

if t_trial < hold_time

    % Apply experimental pressure combination
    u(1:2) = P(:,trial_idx);

else

    % Apply nonzero reset pressure combination
    u(1:2) = p_reset;

end

end
