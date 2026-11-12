
function u = u_openloop_nullQM(x,c,t)

%% ========================================================================
% EXPERIMENT SETTINGS
% =========================================================================

% Desired equilibrium configuration [deg]
q_des = 40;

% Null-space sweep settings
z_step = 40; % 10          % Arc-length spacing [hPa]
num_z_values = 5; % 10     % Number of distinct pressure combinations
repetitions = 2;       % Repetitions of each combination

% Reset configuration [deg]
q_reset = 0;

% Distance along reset contour [hPa]
z_reset = 50;

% Timing [s]
initial_wait = 5;
hold_time    = 60;
reset_time   = 20;

% Pressure limits [hPa gauge]
p_min = 0;
p_max = 250;

% Numerical settings
integration_rel_tol = 1e-9;
integration_abs_tol = 1e-10;

% Maximum allowable model configuration error [deg]
configuration_tolerance = 1e-5;

% Initial contour search tolerance [deg]
max_seed_error = 1e-6;

% Jacobian singularity threshold [deg/hPa]
jacobian_tolerance = 1e-8;

%% ========================================================================
% IDENTIFIED QUADRATIC STATIC MODEL
% =========================================================================

% q_deg = beta0 + beta1*p1 + beta2*p2 ...
%       + beta3*p1^2 + beta4*p1*p2 + beta5*p2^2

beta0 = 0.506263;
beta1 = 0.347200;
beta2 = -0.223037;
beta3 = 0.000184;
beta4 = 0.002320;
beta5 = -0.002814;

beta = [beta0 beta1 beta2 beta3 beta4 beta5];

if any(~isfinite(beta))
    error('Invalid quadratic model coefficients.');
end

%% ========================================================================
% PERSISTENT EXPERIMENTAL PRESSURE SCHEDULE
% =========================================================================

% The pressure schedule is calculated once at the beginning
% of the experiment.
%
% ode45 internally updates the pressure-dependent null-space
% direction throughout numerical integration.
%
% The experimental controller then applies the calculated
% pressure combinations according to the timing schedule.

persistent P p_reset num_trials

if isempty(P)

    %% ====================================================================
    % INITIAL PRESSURE COMBINATION
    % =====================================================================

    % Find a feasible starting point satisfying:
    %
    % q(p_start) = q_des
    %
    % and permitting both pressures to increase along
    % the selected null-space direction.

    p_start = find_contour_start( ...
        q_des,beta,p_min,p_max,max_seed_error);

    %% ====================================================================
    % NULL-SPACE PRESSURE ALLOCATION USING ODE45
    % =====================================================================

    % Experimental null-space coordinates [hPa]
    z_unique = (0:num_z_values-1)*z_step;

    % Integration options
    options = odeset( ...
        'RelTol',integration_rel_tol, ...
        'AbsTol',integration_abs_tol, ...
        'Events',@(z,p) contour_events( ...
            z,p,beta,p_min,p_max,jacobian_tolerance));

    % Integrate:
    %
    % dp/dz = N(p)
    %
    % The solver automatically selects intermediate integration
    % steps and evaluates the pressure-dependent Jacobian.

    [z_sol,P_sol,ze,~,~] = ode45( ...
        @(z,p) null_direction(p,beta), ...
        z_unique, ...
        p_start, ...
        options);

    % Verify that integration reached the requested final z
    if ~isempty(ze) || ...
       numel(z_sol) ~= num_z_values || ...
       abs(z_sol(end)-z_unique(end)) > 1e-7

        error(['Null-space integration terminated before ', ...
               'reaching the requested final z.']);

    end

    % Store pressure combinations
    %
    % P_unique(:,k) corresponds to z_unique(k)

    P_unique = P_sol.';

    %% ====================================================================
    % VERIFY PRESSURE ALLOCATION
    % =====================================================================

    q_pred = zeros(1,num_z_values);

    for k = 1:num_z_values

        q_pred(k) = quadratic_model( ...
            P_unique(:,k),beta);

    end

    % Maximum configuration error
    q_error = max(abs(q_pred-q_des));

    fprintf('\n');
    fprintf('QUADRATIC NULL-SPACE ALLOCATION\n');
    fprintf('----------------------------------------\n');
    fprintf('Desired configuration: %.3f deg\n',q_des);
    fprintf('Maximum model error: %.10f deg\n',q_error);

    if q_error > configuration_tolerance

        error(['ODE45 integration did not maintain ', ...
               'the desired model configuration.']);

    end

    % Verify pressure bounds
    if any(P_unique(:) < p_min-1e-8) || ...
       any(P_unique(:) > p_max+1e-8)

        error('Calculated pressure combinations exceed limits.');

    end

    % Verify both chamber pressures increase
    if any(diff(P_unique,1,2) <= 0,'all')

        error(['Calculated pressure combinations do not ', ...
               'increase both chamber pressures.']);

    end

    % Verify selected null-space branch at each output
    for k = 1:num_z_values

        J = quadratic_jacobian(P_unique(:,k),beta);

        if J(1) <= 0 || J(2) >= 0
            error('Invalid null-space branch at z index %d.',k);
        end

    end

    %% ====================================================================
    % DISPLAY PRESSURE SCHEDULE
    % =====================================================================

    pressure_schedule = table( ...
        z_unique(:), ...
        P_unique(1,:)', ...
        P_unique(2,:)', ...
        q_pred', ...
        'VariableNames', ...
        {'z_hPa','p1_hPa','p2_hPa','q_pred_deg'});

    disp(pressure_schedule);

    %% ====================================================================
    % REPEAT PRESSURE COMBINATIONS
    % =====================================================================

    % Repeat each pressure combination consecutively:
    %
    % Trial 1: z1, unloaded
    % Trial 2: z1, loaded
    % Trial 3: z2, unloaded
    % Trial 4: z2, loaded
    % ...
    %
    % The weight is applied/removed manually during
    % the reset interval.

    P = repelem(P_unique,1,repetitions);

    num_trials = size(P,2);

    %% ====================================================================
    % RESET PRESSURE ALLOCATION
    % =====================================================================

    % Find initial point on the q_reset contour

    p_reset_start = find_contour_start( ...
        q_reset,beta,p_min,p_max,max_seed_error);

    % Integrate along reset contour to z_reset

    if z_reset > 0

        [z_reset_sol,P_reset_sol,ze_reset,~,~] = ode45( ...
            @(z,p) null_direction(p,beta), ...
            [0 z_reset], ...
            p_reset_start, ...
            options);

        if ~isempty(ze_reset) || ...
           abs(z_reset_sol(end)-z_reset) > 1e-7

            error('Reset contour integration terminated early.');

        end

        p_reset = P_reset_sol(end,:).';

    else

        p_reset = p_reset_start;

    end

    %% ====================================================================
    % VERIFY RESET CONFIGURATION
    % =====================================================================

    q_reset_pred = quadratic_model(p_reset,beta);

    reset_error = abs(q_reset_pred-q_reset);

    if reset_error > configuration_tolerance

        error('Reset pressure allocation configuration error.');

    end

    if any(p_reset < p_min-1e-8) || ...
       any(p_reset > p_max+1e-8)

        error('Reset pressures exceed physical limits.');

    end

    fprintf('\nRESET PRESSURE ALLOCATION\n');
    fprintf('----------------------------------------\n');
    fprintf('p1_reset = %.4f hPa\n',p_reset(1));
    fprintf('p2_reset = %.4f hPa\n',p_reset(2));
    fprintf('Predicted reset angle = %.6f deg\n',q_reset_pred);

    %% ====================================================================
    % EXPERIMENT INFORMATION
    % =====================================================================

    trial_duration = hold_time+reset_time;

    total_duration = initial_wait+num_trials*trial_duration;

    fprintf('\nEXPERIMENT SETTINGS\n');
    fprintf('----------------------------------------\n');
    fprintf('Unique z values: %d\n',num_z_values);
    fprintf('Repetitions per z: %d\n',repetitions);
    fprintf('Total trials: %d\n',num_trials);
    fprintf('Hold duration: %.1f s\n',hold_time);
    fprintf('Reset duration: %.1f s\n',reset_time);
    fprintf('Total duration: %.1f s\n',total_duration);
    fprintf('Total duration: %.2f min\n',total_duration/60);

    fprintf('\nTrial schedule:\n');

    for k = 1:num_trials

        z_idx = ceil(k/repetitions);
        rep_idx = mod(k-1,repetitions)+1;

        fprintf(['Trial %2d | z = %5.1f hPa | ', ...
                 'Repetition %d | p = [%.3f %.3f]\n'], ...
                 k,z_unique(z_idx),rep_idx, ...
                 P(1,k),P(2,k));

    end

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
t_exp = t-initial_wait;

% Each trial includes a hold and reset interval
trial_duration = hold_time+reset_time;

% Current trial index
trial_idx = floor(t_exp/trial_duration)+1;

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

    % Apply reset pressure combination
    u(1:2) = p_reset;

end

end

%% ========================================================================
% LOCAL FUNCTION: QUADRATIC MODEL
% =========================================================================

function q = quadratic_model(p,beta)

p1 = p(1);
p2 = p(2);

q = beta(1) ...
  + beta(2)*p1 ...
  + beta(3)*p2 ...
  + beta(4)*p1^2 ...
  + beta(5)*p1*p2 ...
  + beta(6)*p2^2;

end

%% ========================================================================
% LOCAL FUNCTION: QUADRATIC JACOBIAN
% =========================================================================

function J = quadratic_jacobian(p,beta)

p1 = p(1);
p2 = p(2);

dq_dp1 = beta(2) ...
       + 2*beta(4)*p1 ...
       + beta(5)*p2;

dq_dp2 = beta(3) ...
       + beta(5)*p1 ...
       + 2*beta(6)*p2;

J = [dq_dp1 dq_dp2];

end

%% ========================================================================
% LOCAL FUNCTION: NULL-SPACE DIRECTION
% =========================================================================

function N = null_direction(p,beta)

% Calculate the pressure-dependent Jacobian
J = quadratic_jacobian(p,beta);

J_norm = norm(J);

if J_norm < 1e-8

    error('Quadratic Jacobian is nearly singular.');

end

% Normalized null-space direction
%
% J*N = 0
%
% For the selected branch:
% j1 > 0 and j2 < 0
%
% Both pressures increase as z increases.

N = [-J(2);J(1)]/J_norm;

end

%% ========================================================================
% LOCAL FUNCTION: ODE45 TERMINATION EVENTS
% =========================================================================

function [value,isterminal,direction] = contour_events( ...
    ~,p,beta,p_min,p_max,jacobian_tolerance)

J = quadratic_jacobian(p,beta);

% Terminate when:
% 1. Either pressure reaches its upper limit
% 2. The increasing-pressure branch reaches a turning point
% 3. The Jacobian approaches singularity
%
% Lower pressure limits are verified during initialization.
% Both pressures increase along the selected branch.

value = [ ...
    p_max-p(1); ...
    p_max-p(2); ...
    J(1); ...
    -J(2); ...
    norm(J)-jacobian_tolerance];

isterminal = ones(5,1);

% Detect crossings from positive to zero
direction = -ones(5,1);

end

%% ========================================================================
% LOCAL FUNCTION: FIND INITIAL CONTOUR POINT
% =========================================================================

function p_start = find_contour_start( ...
    q_des,beta,p_min,p_max,tolerance)

% Find feasible pressure combinations satisfying
%
% q(p1,p2) = q_des
%
% Search the branch where:
%
% dq/dp1 > 0
% dq/dp2 < 0
%
% This permits both pressures to increase while
% maintaining constant configuration.
%
% Select the feasible point with the smallest
% total pressure p1+p2.

p1_grid = linspace(p_min,p_max,5001);

candidates = [];

for k = 1:length(p1_grid)

    p1 = p1_grid(k);

    % Quadratic equation in p2:
    %
    % a*p2^2 + b*p2 + c = 0

    a = beta(6);

    b = beta(3)+beta(5)*p1;

    c = beta(1)+beta(2)*p1 ...
      + beta(4)*p1^2-q_des;

    %% Solve for p2

    if abs(a) < 1e-12

        if abs(b) < 1e-12
            continue;
        end

        roots_p2 = -c/b;

    else

        discriminant = b^2-4*a*c;

        if discriminant < 0
            continue;
        end

        roots_p2 = [ ...
            (-b+sqrt(discriminant))/(2*a), ...
            (-b-sqrt(discriminant))/(2*a)];

    end

    %% Check candidate pressure combinations

    for j = 1:length(roots_p2)

        p2 = roots_p2(j);

        if ~isreal(p2) || ~isfinite(p2)
            continue;
        end

        if p2 < p_min || p2 > p_max
            continue;
        end

        p_candidate = [p1;p2];

        % Verify configuration
        q_candidate = quadratic_model(p_candidate,beta);

        if abs(q_candidate-q_des) > tolerance
            continue;
        end

        % Evaluate Jacobian
        J = quadratic_jacobian(p_candidate,beta);

        % Require both pressures to increase along
        % the selected null-space direction.
        if J(1) <= 0 || J(2) >= 0
            continue;
        end

        candidates = [candidates p_candidate];

    end

end

%% ========================================================================
% SELECT INITIAL PRESSURE COMBINATION
% =========================================================================

if isempty(candidates)

    error(['No feasible initial pressure combination ', ...
           'found on the increasing-pressure branch.']);

end

% Minimize total initial chamber pressure
[~,idx] = min(sum(candidates,1));

p_start = candidates(:,idx);

% Final verification
if any(p_start < p_min) || any(p_start > p_max)

    error('Initial pressure combination exceeds limits.');

end

if abs(quadratic_model(p_start,beta)-q_des) > tolerance

    error('Initial pressure combination does not satisfy q_des.');

end

end
