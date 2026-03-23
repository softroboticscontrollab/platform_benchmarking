function ezloophwROS2BendingAngleCallback(message, handles)
    % Persistent filters and state history
    persistent asymGauss1 asymGauss2 symGauss1 symGauss2 butterLp1 butterLp2 isInitialized
    persistent q_prev dq_prev % Store [q1, q2] and [dq1, dq2] for backward diff
    persistent dtHistory      % Buffer for dt sliding average (print only)
    
    global bendingVec dBendingVec ddBendingVec
    global prevRxTime q_storage dq_storage ddq_storage

    % --- Settings ---
    sigma = 0.6; 
    hw_sigma = 3;
    dt_win = 5; 
    fc = 0.2;
    
    % 1. Calculate Instantaneous dt
    tnow = double(ros2time(handles.node,'now').sec) + double(ros2time(handles.node,'now').nanosec) * 1e-9;
    
    if isempty(prevRxTime) || prevRxTime == 0
        prevRxTime = tnow;
        return;
    end
    
    inst_dt = tnow - prevRxTime;
    
    % --- Timestamp Sanity Check ---
    if inst_dt <= 0 %|| inst_dt > 0.5
        warning('Invalid dt detected (%.4f). Discarding frame.', inst_dt);
        % Dont update prevRxTime since tnow is wrong; 
        return;
    end
    
    prevRxTime = tnow;

    % 2. Update dt Sliding Average (For monitoring only)
    if isempty(dtHistory)
        dtHistory = repmat(inst_dt, dt_win, 1);
    else
        dtHistory = [dtHistory(2:end); inst_dt];
    end
    dt_avg = mean(dtHistory);

    % --- 3. Process Message ---
    if ~isempty(message.data)
        % Convert raw data to radians (English comments as requested)
        rawBending = 2 * message.data(:)' * (pi/180);
        
        % Step 1: Smoothing to get current Q
        % Initialization check happens here to capture the first valid filtered q
        if isempty(isInitialized)
            % Initialize filters with nominal dt
            dt_fixed = 0.025;
            asymGauss1 = RealTimeFilters.CausalAsymGaussFilter(dt_fixed, sigma, hw_sigma);
            asymGauss2 = RealTimeFilters.CausalAsymGaussFilter(dt_fixed, sigma, hw_sigma);
            symGauss1  = RealTimeFilters.CausalSymGaussFilter(dt_fixed, sigma, hw_sigma);
            symGauss2  = RealTimeFilters.CausalSymGaussFilter(dt_fixed, sigma, hw_sigma);
            butterLp1  = RealTimeFilters.RealTimeButterFilter(dt_fixed, fc);
            butterLp2  = RealTimeFilters.RealTimeButterFilter(dt_fixed, fc);
            % Get the very first filtered sample
            [q1_init, ~] = asymGauss1.step(rawBending(1));
            [q2_init, ~] = asymGauss2.step(rawBending(2));
            
            % Set q_prev to the current filtered value to avoid huge initial dq
            q_prev = [q1_init, q2_init];
            dq_prev = [0, 0];
            
            % Update global current state for the first frame
            bendingVec = q_prev;
            dBendingVec = [0, 0];
            ddBendingVec = [0, 0];
            
            isInitialized = true;
            return; % Skip differentiation for the very first frame
        end

        % Normal Operation: Step Filters
%         [q1, ~] = asymGauss1.step(rawBending(1));
%         [q2, ~] = asymGauss2.step(rawBending(2));
        % [q1, ~] = symGauss1.step(rawBending(1));
        % [q2, ~] = symGauss2.step(rawBending(2));
        [q1, ~] = butterLp1.step(rawBending(1));
        [q2, ~] = butterLp2.step(rawBending(2));
        q_now = [q1, q2];

        % Step 2: Backward Difference for dq and ddq
        dq_now = (q_now - q_prev) / dt_avg;
        ddq_now = (dq_now - dq_prev) / dt_avg;

        % Update Globals
        bendingVec = q_now;
        dBendingVec = dq_now;
        ddBendingVec = ddq_now;

        % Update Persistent History
        q_prev = q_now;
        dq_prev = dq_now;

        % 4. Storage and Debugging
        q_storage = [q_storage; bendingVec];
        dq_storage = [dq_storage; dBendingVec];
        ddq_storage = [ddq_storage; ddBendingVec];
        
        fprintf('dt: %.4f | dt_avg: %.4f | Q: [%.2f %.2f] | dQ: [%.2f %.2f] | ddQ: [%.2f %.2f]\n', ...
            inst_dt, dt_avg, bendingVec(1), bendingVec(2), dBendingVec(1), dBendingVec(2), ddBendingVec(1), ddBendingVec(2));
    end
end