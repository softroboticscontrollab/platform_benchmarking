function w = u_v_static_sweep(x, c, t)
% U_V_STATIC_SWEEP_NO_C
%
% Automated static-response sweep.
%
% For every combination of:
%       u_values
%       base_values
%
% the controller performs:
%
%   1) RESET
%        Set all control inputs to zero.
%        Wait until uncontrolled static equilibrium is reached.
%
%   2) APPLY U
%        Apply one selected component of u.
%        Wait until static equilibrium is reached.
%        Save the resulting position as q_ref.
%
%   3) APPLY BASE OFFSET
%        Keep u constant.
%        Apply the selected base offset to one component of v.
%        Keep the compensating component of v at zero.
%
%   4) WAIT FOR BASE-OFFSET STATIC RESPONSE
%        Allow the robot to reach its new static position after the base
%        offset has been introduced.
%
%   5) COMPENSATE
%        Adjust ONLY v(v_adjust_idx) until q returns to q_ref.
%
%   6) VERIFY FINAL STATIC RESPONSE
%        Freeze all inputs and verify that:
%
%             |q - q_ref| <= q_tol
%             |dq|        <= dq_tol
%
%        continuously for static_time seconds.
%
%   7) RECORD RESULTS
%
%   8) RESET
%        Set all inputs back to zero.
%        Wait for uncontrolled static equilibrium.
%        Automatically move to the next (u,base) combination.
%
%
% Output:
%
%       w = [u1; u2; v1; v2]
%
%
% To restart the entire sweep:
%
%       clear u_v_static_sweep_no_c
%
% -------------------------------------------------------------------------


%% ========================================================================
%  EXPERIMENT SETTINGS
%  Edit these values directly.
% =========================================================================

% Values of u to test
u_values = [40 60 80 100];

% Base-offset values to test
base_values = [20 40 60 80];


% -------------------------------------------------------------------------
% Which coordinate / input components are being used?
% -------------------------------------------------------------------------

% q coordinate whose position is monitored
q_idx = 1;

% Component of u being actuated
u_command_idx = 1;

% Component of v receiving the prescribed base offset
v_base_idx = 1;

% ONLY this component of v is adjusted during compensation
v_adjust_idx = 2;


% -------------------------------------------------------------------------
% Static-response criteria
% -------------------------------------------------------------------------

% Allowed position error relative to q_ref [rad]
q_tol = 0.01;

% Velocity threshold used to determine static response [rad/s]
dq_tol = 0.002;

% Amount of continuous time that the velocity criterion must be satisfied
% before the system is considered static [s]
static_time = 2.0;


% -------------------------------------------------------------------------
% Compensation settings
% -------------------------------------------------------------------------

% Amount by which the adjustable v input changes each update
adjust_step = 0.1;

% Minimum time between changes in the adjustable v input [s]
%
% This prevents changing pressure every single controller iteration if the
% control loop is very fast.
adjust_period = 0.1;

% Adjustable pressure limits
%
% Pressure is always additive, so v must remain nonnegative.
v_min = 0;
v_max = 300;


% -------------------------------------------------------------------------
% Compensation direction
% -------------------------------------------------------------------------
%
% err = q_ref - q
%
% With adjust_sign = -1:
%
%       err > 0  --> increase v_adjust
%       err < 0  --> decrease v_adjust
%
% This is appropriate if increasing the compensating pressure moves the
% robot in the direction required when q < q_ref.
%
% If the physical direction is opposite, change this to +1.
%
adjust_sign = 1;


%% ========================================================================
%  ROBOT STATE
% =========================================================================

q  = x(1:2);
dq = x(3:4);


%% ========================================================================
%  PERSISTENT EXPERIMENT MEMORY
% =========================================================================

persistent state

persistent test_u_idx
persistent test_base_idx
persistent trial_idx

persistent q_ref
persistent q_base_static

persistent v_adjust

persistent static_start_time
persistent last_adjust_time

persistent initialized

persistent results


%% ========================================================================
%  INITIALIZATION
% =========================================================================

if isempty(initialized)

    % State definitions:
    %
    % 1 = RESET_INPUTS
    % 2 = WAIT_RESET_STATIC
    %
    % 3 = APPLY_U
    % 4 = WAIT_U_STATIC
    %
    % 5 = APPLY_BASE
    % 6 = WAIT_BASE_STATIC
    %
    % 7 = ADJUST_V
    % 8 = WAIT_FINAL_STATIC
    %
    % 9 = ADVANCE_TEST
    %
    % 10 = COMPLETE

    state = 1;


    % Start with first u/base combination
    test_u_idx = 1;
    test_base_idx = 1;

    trial_idx = 1;


    % Stored positions
    q_ref = NaN;
    q_base_static = NaN;


    % Adjustable pressure command
    v_adjust = 0;


    % Timers
    static_start_time = NaN;
    last_adjust_time = -inf;


    % Results structure
    results = struct( ...
        'trial',{}, ...
        'u_value',{}, ...
        'base_value',{}, ...
        'v_adjust',{}, ...
        'q_ref',{}, ...
        'q_base_static',{}, ...
        'q_final',{}, ...
        'base_displacement',{}, ...
        'time_complete',{});


    initialized = true;


    fprintf('\n====================================================\n');
    fprintf('Static u/base-offset sweep initialized\n');
    fprintf('====================================================\n');

    fprintf('Monitoring q(%d)\n',q_idx);

    fprintf('Actuating u(%d)\n',u_command_idx);

    fprintf('Base offset applied to v(%d)\n',v_base_idx);

    fprintf('Compensation performed ONLY with v(%d)\n', ...
        v_adjust_idx);

    fprintf('Total combinations: %d\n', ...
        length(u_values)*length(base_values));

    fprintf('====================================================\n');

end


%% ========================================================================
%  DEFAULT OUTPUT
% =========================================================================

u = zeros(2,1);
v = zeros(2,1);


%% ========================================================================
%  STATE MACHINE
% =========================================================================

switch state


    %% ====================================================================
    %  STATE 1
    %  RESET ALL INPUTS
    % =====================================================================

    case 1

        u(:) = 0;
        v(:) = 0;


        % Clear values from previous trial
        q_ref = NaN;
        q_base_static = NaN;

        v_adjust = 0;


        % Reset timers
        static_start_time = NaN;
        last_adjust_time = -inf;


        fprintf('\n----------------------------------------------------\n');

        fprintf('Trial %d of %d\n', ...
            trial_idx, ...
            length(u_values)*length(base_values));

        fprintf('Resetting all control inputs to zero.\n');


        state = 2;



    %% ====================================================================
    %  STATE 2
    %  WAIT FOR UNCONTROLLED STATIC EQUILIBRIUM
    % =====================================================================

    case 2

        % Everything remains zero
        u(:) = 0;
        v(:) = 0;


        % Since the whole robot is uncontrolled, wait until both measured
        % velocities are sufficiently small.
        reset_is_static = all(abs(dq) <= dq_tol);


        if reset_is_static

            if isnan(static_start_time)

                static_start_time = t;

            end


            if (t - static_start_time) >= static_time

                fprintf('Uncontrolled static equilibrium reached.\n');

                static_start_time = NaN;

                state = 3;

            end

        else

            % Motion restarted, so restart the static dwell timer
            static_start_time = NaN;

        end



    %% ====================================================================
    %  STATE 3
    %  APPLY SELECTED U
    % =====================================================================

    case 3

        current_u = u_values(test_u_idx);


        % Start from zero
        u(:) = 0;
        v(:) = 0;


        % Apply only the selected u component
        u(u_command_idx) = current_u;


        fprintf('\nCurrent combination:\n');

        fprintf('u = %.4f\n',current_u);

        fprintf('base offset = %.4f\n', ...
            base_values(test_base_idx));

        fprintf('Applying u(%d) = %.4f with v = 0.\n', ...
            u_command_idx,current_u);


        static_start_time = NaN;


        state = 4;



    %% ====================================================================
    %  STATE 4
    %  WAIT FOR STATIC RESPONSE UNDER U
    % =====================================================================

    case 4

        current_u = u_values(test_u_idx);


        % Hold exact same input
        u(:) = 0;
        v(:) = 0;

        u(u_command_idx) = current_u;


        % Check monitored coordinate
        initial_is_static = abs(dq(q_idx)) <= dq_tol;


        if initial_is_static

            if isnan(static_start_time)

                static_start_time = t;

            end


            if (t - static_start_time) >= static_time

                % Save the equilibrium position produced by u alone
                q_ref = q(q_idx);


                fprintf('\nInitial static response reached.\n');

                fprintf('Saved q_ref = %.6f rad.\n',q_ref);


                static_start_time = NaN;


                state = 5;

            end

        else

            static_start_time = NaN;

        end



    %% ====================================================================
    %  STATE 5
    %  APPLY BASE OFFSET
    % =====================================================================

    case 5

        current_u = u_values(test_u_idx);

        current_base = base_values(test_base_idx);


        % Reconstruct held inputs
        u(:) = 0;
        v(:) = 0;


        % Keep u CONSTANT
        u(u_command_idx) = current_u;


        % Apply the prescribed base offset
        v(v_base_idx) = current_base;


        % Compensating v remains zero initially
        v_adjust = 0;

        v(v_adjust_idx) = v_adjust;


        fprintf('\nApplying base offset.\n');

        fprintf('v(%d) = %.4f\n', ...
            v_base_idx,current_base);

        fprintf('v(%d) remains at zero for now.\n', ...
            v_adjust_idx);

        fprintf('Waiting for new static position before compensation.\n');


        static_start_time = NaN;


        state = 6;



    %% ====================================================================
    %  STATE 6
    %  WAIT FOR NEW STATIC RESPONSE AFTER BASE OFFSET
    % =====================================================================

    case 6

        current_u = u_values(test_u_idx);

        current_base = base_values(test_base_idx);


        % Hold every input constant
        u(:) = 0;
        v(:) = 0;


        u(u_command_idx) = current_u;

        v(v_base_idx) = current_base;

        v(v_adjust_idx) = 0;


        % Wait for the monitored coordinate to stop moving
        base_is_static = abs(dq(q_idx)) <= dq_tol;


        if base_is_static

            if isnan(static_start_time)

                static_start_time = t;

            end


            if (t - static_start_time) >= static_time

                % Save the displaced equilibrium position
                q_base_static = q(q_idx);


                fprintf('\nBase-offset static response reached.\n');

                fprintf('q_ref         = %.6f rad\n',q_ref);

                fprintf('q_base_static = %.6f rad\n',q_base_static);

                fprintf('Displacement  = %.6f rad\n', ...
                    q_base_static - q_ref);


                % Begin compensation from zero pressure
                v_adjust = 0;

                last_adjust_time = -inf;

                static_start_time = NaN;


                state = 7;

            end

        else

            static_start_time = NaN;

        end



    %% ====================================================================
    %  STATE 7
    %  ADJUST ONLY ONE V COMPONENT UNTIL q_ref IS RECOVERED
    % =====================================================================

    case 7

        current_u = u_values(test_u_idx);

        current_base = base_values(test_base_idx);


        % Reconstruct all inputs explicitly.
        %
        % This guarantees that:
        %
        %   u remains constant
        %   base offset remains constant
        %   ONLY v_adjust changes
        %
        u(:) = 0;
        v(:) = 0;


        u(u_command_idx) = current_u;

        v(v_base_idx) = current_base;


        % Position error
        err = q_ref - q(q_idx);


        % -------------------------------------------------------------
        % Modify ONLY the compensating pressure
        % -------------------------------------------------------------

        if abs(err) > q_tol && ...
                (t - last_adjust_time) >= adjust_period


            v_adjust = v_adjust ...
                - adjust_sign*adjust_step*sign(err);


            % Pressure must remain positive / additive
            v_adjust = max(v_min, ...
                min(v_max,v_adjust));


            last_adjust_time = t;

        end


        % Apply compensating input
        v(v_adjust_idx) = v_adjust;


        % -------------------------------------------------------------
        % If q_ref has been recovered, stop changing pressure
        % -------------------------------------------------------------

        if abs(err) <= q_tol

            fprintf('\nq_ref recovered within tolerance.\n');

            fprintf('Holding v(%d) = %.4f\n', ...
                v_adjust_idx,v_adjust);

            fprintf('Checking final static response.\n');


            static_start_time = NaN;


            state = 8;

        end



    %% ====================================================================
    %  STATE 8
    %  VERIFY FINAL STATIC RESPONSE
    % =====================================================================

    case 8

        current_u = u_values(test_u_idx);

        current_base = base_values(test_base_idx);


        % Freeze ALL experimental inputs
        u(:) = 0;
        v(:) = 0;


        u(u_command_idx) = current_u;

        v(v_base_idx) = current_base;

        v(v_adjust_idx) = v_adjust;


        % Current position error
        err = q_ref - q(q_idx);


        % Need both correct position and static response
        position_ok = abs(err) <= q_tol;

        velocity_ok = abs(dq(q_idx)) <= dq_tol;


        if position_ok && velocity_ok

            if isnan(static_start_time)

                static_start_time = t;

            end


            if (t - static_start_time) >= static_time


                %% --------------------------------------------------------
                %  RECORD COMPLETED TRIAL
                % ---------------------------------------------------------

                k = length(results) + 1;


                results(k).trial = trial_idx;

                results(k).u_value = current_u;

                results(k).base_value = current_base;

                results(k).v_adjust = v_adjust;

                results(k).q_ref = q_ref;

                results(k).q_base_static = q_base_static;

                results(k).q_final = q(q_idx);

                results(k).base_displacement = ...
                    q_base_static - q_ref;

                results(k).time_complete = t;



                fprintf('\n====================================================\n');

                fprintf('Completed trial %d\n',trial_idx);

                fprintf('====================================================\n');

                fprintf('u(%d)             = %.4f\n', ...
                    u_command_idx,current_u);

                fprintf('v_base(%d)        = %.4f\n', ...
                    v_base_idx,current_base);

                fprintf('v_adjust(%d)      = %.4f\n', ...
                    v_adjust_idx,v_adjust);

                fprintf('q_ref             = %.6f rad\n', ...
                    q_ref);

                fprintf('q_base_static     = %.6f rad\n', ...
                    q_base_static);

                fprintf('q_final           = %.6f rad\n', ...
                    q(q_idx));

                fprintf('base displacement = %.6f rad\n', ...
                    q_base_static-q_ref);

                fprintf('====================================================\n');


                static_start_time = NaN;


                state = 9;

            end


        else

            % If the robot moves away from q_ref before final static
            % equilibrium is confirmed, resume compensation.
            static_start_time = NaN;


            if ~position_ok

                state = 7;

            end

        end



    %% ====================================================================
    %  STATE 9
    %  ADVANCE AUTOMATICALLY TO NEXT COMBINATION
    % =====================================================================

    case 9

        % Immediately reset every control input
        u(:) = 0;
        v(:) = 0;


        % -------------------------------------------------------------
        % First sweep through all base values for current u
        % -------------------------------------------------------------

        if test_base_idx < length(base_values)

            test_base_idx = test_base_idx + 1;

            trial_idx = trial_idx + 1;


            % Return to complete reset state
            state = 1;


        % -------------------------------------------------------------
        % Then move to next u value
        % -------------------------------------------------------------

        elseif test_u_idx < length(u_values)

            test_u_idx = test_u_idx + 1;

            test_base_idx = 1;

            trial_idx = trial_idx + 1;


            % Return to complete reset state
            state = 1;


        % -------------------------------------------------------------
        % Entire sweep complete
        % -------------------------------------------------------------

        else

            state = 10;


            fprintf('\n====================================================\n');

            fprintf('STATIC SWEEP COMPLETE\n');

            fprintf('%d trials completed.\n', ...
                length(results));

            fprintf('All control inputs commanded to zero.\n');

            fprintf('====================================================\n');


            % Export completed data to MATLAB base workspace
            assignin( ...
                'base', ...
                'static_sweep_results', ...
                results);

        end



    %% ====================================================================
    %  STATE 10
    %  COMPLETE
    % =====================================================================

    case 10

        % Keep all inputs at zero after experiment completes
        u(:) = 0;
        v(:) = 0;

end


%% ========================================================================
%  OUTPUT
% =========================================================================

w = [u; abs(v)];


% Optional debugging:
%
% Uncomment this if you want to continuously see the output vector.

fprintf('w = [%.2f %.2f %.2f %.2f]\n', ...
    w(1),w(2),w(3),w(4));


end