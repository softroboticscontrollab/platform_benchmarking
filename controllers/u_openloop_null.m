function u = u_openloop_null(x,c,t)
% NULL_SPACE_STATIC
%
% Output:
%   u = [p1; p2; 0; 0]  [hPa gauge]
%
% Each feasible null-space pressure combination is:
%   1. Applied for hold_time seconds
%   2. Released for reset_time seconds

%% ========================================================================
% EXPERIMENT SETTINGS
% =========================================================================

% Desired equilibrium configuration [deg]
q_des = 20;

% Null-space inputs to test [hPa]
z_values = [0 10 20 30 40 50 60 70 80 90 100];

% Timing [s]
initial_wait = 5;
hold_time    = 30;
reset_time   = 10;

% Pressure limits [hPa gauge]
p_min = 0;
p_max = 250;

%% ========================================================================
% IDENTIFIED STATIC MODEL
% =========================================================================

% Replace with experimental regression coefficients
% q_deg = beta0 + beta1*p1 + beta2*p2

beta0 = -1.264908;       % [deg] 
beta1 = 0.423700;     % [deg/hPa] 
beta2 = -0.268104;    % [deg/hPa]

B = [beta1 beta2];

if norm(B) < 1e-12
    error('Identified B matrix is approximately zero.');
end

% Explicit unit null-space vector for a 1x2 matrix.
% This avoids numerical sign ambiguity between runs.
N = [-B(2); B(1)] / norm(B);

%% ========================================================================
% PRESSURE ALLOCATION
% =========================================================================

p_nominal = pinv(B)*(q_des-beta0);

% Each column is one pressure combination
P = p_nominal + N*z_values;

% Check feasibility of each trial
valid = all(P >= p_min & P <= p_max,1);

% Keep only feasible pressure combinations
P = P(:,valid);
z_valid = z_values(valid);

num_trials = length(z_valid);

if num_trials == 0
    error('No feasible null-space pressure combinations.');
end

%% ========================================================================
% EXPERIMENT TIMING
% =========================================================================

u = zeros(4,1);

if t < initial_wait
    return;
end

% Time since initial waiting period
t_exp = t - initial_wait;

% Each trial includes a pressure hold and a reset
trial_duration = hold_time + reset_time;

% Current trial index
trial_idx = floor(t_exp/trial_duration) + 1;

% Experiment completed
if trial_idx > num_trials
    return;
end

% Time within current trial
t_trial = mod(t_exp,trial_duration);

if t_trial < hold_time

    % Apply the selected pressure combination
    u(1:2) = P(:,trial_idx);

else

    % Reset all chamber pressures
    u(:) = 0;

end

end
