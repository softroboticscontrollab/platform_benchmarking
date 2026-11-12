
function u = u_openloop_quad_calib(x,c,t)

%% ========================================================================
% QUADRATIC MODEL PRESSURE SWEEP
% =========================================================================
%
% Sweeps all combinations of p1 and p2 from 0 to 200 hPa.
%
% The controller only determines when to advance to the next pressure
% combination. All data is recorded by the external logger.
%
% Output:
%   u = [p1; p2; 0; 0]
%
% Restart:
%   clear u_openloop_quadratic_calibration
%
% =========================================================================

%% EXPERIMENT SETTINGS

p_min = 0;
p_max = 200;
p_step = 10;

q_idx = 1;

dq_tol = 0.002;       % rad/s
static_time = 2.0;    % s

min_hold_time = 5;    % s
max_hold_time = 20;   % s

initial_wait = 5;     % s

rest_every = 20;      % Number of pressure combinations
rest_time = 60;       % s

repetitions = 1;

%% ROBOT STATE

dq = x(3:4);

%% PERSISTENT VARIABLES

persistent initialized
persistent state
persistent P
persistent num_trials
persistent trial_idx
persistent state_start_time
persistent trial_start_time
persistent static_start_time
persistent timeout_count

%% INITIALIZATION

if isempty(initialized)

    p_values = p_min:p_step:p_max;
    num_p = length(p_values);

    P_unique = zeros(2,num_p^2);

    idx = 1;

    % Serpentine grid sweep
    for j = 1:num_p

        p2 = p_values(j);

        if mod(j,2) == 1
            p1_sequence = p_values;
        else
            p1_sequence = fliplr(p_values);
        end

        for i = 1:num_p

            P_unique(:,idx) = [p1_sequence(i);p2];
            idx = idx+1;

        end

    end

    P = repmat(P_unique,1,repetitions);

    num_trials = size(P,2);

    % States:
    % 1 = INITIAL_WAIT
    % 2 = APPLY_PRESSURE
    % 3 = WAIT_FOR_EQUILIBRIUM
    % 4 = ADVANCE_TRIAL
    % 5 = REST
    % 6 = COMPLETE

    state = 1;

    trial_idx = 1;

    state_start_time = t;
    trial_start_time = NaN;
    static_start_time = NaN;

    timeout_count = 0;

    initialized = true;

    fprintf('\n');
    fprintf('QUADRATIC MODEL PRESSURE SWEEP\n');
    fprintf('----------------------------------------\n');
    fprintf('Pressure range: %.1f to %.1f hPa\n',p_min,p_max);
    fprintf('Pressure step: %.1f hPa\n',p_step);
    fprintf('Total combinations: %d\n',num_trials);
    fprintf('Minimum hold: %.1f s\n',min_hold_time);
    fprintf('Maximum hold: %.1f s\n',max_hold_time);
    fprintf('Static verification: %.1f s\n',static_time);
    fprintf('Rest every: %d trials\n',rest_every);
    fprintf('Rest duration: %.1f s\n',rest_time);
    fprintf('----------------------------------------\n');

end

%% DEFAULT OUTPUT

u = zeros(4,1);

%% STATE MACHINE

switch state

    %% STATE 1: INITIAL WAIT
    case 1

        u(:) = 0;

        if (t-state_start_time) >= initial_wait

            state = 2;
            state_start_time = t;

        end

    %% STATE 2: APPLY PRESSURES
    case 2

        u(1:2) = P(:,trial_idx);

        trial_start_time = t;
        static_start_time = NaN;

        fprintf('\nTrial %d/%d\n',trial_idx,num_trials);
        fprintf('p1 = %.1f hPa, p2 = %.1f hPa\n', ...
            P(1,trial_idx),P(2,trial_idx));

        state = 3;
        state_start_time = t;

    %% STATE 3: WAIT FOR EQUILIBRIUM
    case 3

        u(1:2) = P(:,trial_idx);

        elapsed = t-trial_start_time;

        % Minimum settling time
        if elapsed < min_hold_time
            static_start_time = NaN;
            return;
        end

        % Check velocity criterion
        velocity_ok = isfinite(dq(q_idx)) && ...
            abs(dq(q_idx)) <= dq_tol;

        if velocity_ok

            if isnan(static_start_time)
                static_start_time = t;
            end

            if (t-static_start_time) >= static_time

                fprintf('Equilibrium detected at %.2f s\n',elapsed);

                state = 4;
                state_start_time = t;

            end

        else

            static_start_time = NaN;

        end

        % Timeout protection
        if state == 3 && elapsed >= max_hold_time

            fprintf('WARNING: Trial %d timed out.\n',trial_idx);

            timeout_count = timeout_count+1;

            state = 4;
            state_start_time = t;

        end

    %% STATE 4: ADVANCE TRIAL
    case 4

        % Maintain previous command until next state
        u(1:2) = P(:,trial_idx);

        if trial_idx >= num_trials

            state = 6;
            u(:) = 0;

            fprintf('\n');
            fprintf('PRESSURE SWEEP COMPLETE\n');
            fprintf('Total trials: %d\n',num_trials);
            fprintf('Timed-out trials: %d\n',timeout_count);

        elseif mod(trial_idx,rest_every) == 0

            fprintf('\nCooling period: %.1f seconds\n',rest_time);

            state = 5;
            state_start_time = t;
            u(:) = 0;

        else

            trial_idx = trial_idx+1;

            state = 2;
            state_start_time = t;

        end

    %% STATE 5: REST
    case 5

        u(:) = 0;

        if (t-state_start_time) >= rest_time

            fprintf('Cooling period complete.\n');

            trial_idx = trial_idx+1;

            state = 2;
            state_start_time = t;

        end

    %% STATE 6: COMPLETE
    case 6

        u(:) = 0;

end

end
