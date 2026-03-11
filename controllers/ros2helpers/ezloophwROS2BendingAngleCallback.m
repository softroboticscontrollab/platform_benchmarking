function ezloophwROS2BendingAngleCallback(message, handles)
    %exampleHelperROS2PoseCallback Subscriber callback function for pose data    
    %   exampleHelperROS2PoseCallback(MESSAGE) returns no arguments - it instead sets 
    %   global variables to the values of position and orientation that are
    %   received in the ROS 2 message MESSAGE.
    %   
    %   See also ROSPublishAndSubscribeExample.
    
    %   Copyright 2019 The MathWorks, Inc.
    
    % Declare global variables to store bending angle
    global bendingVec
    global n_smooth_prev_velocity % for smoothing the bending angle: we have this may rows (previous samples)
    global prevBendingVec % now stores previous 10
    global dBendingVec
    global prevRxTime
    global q_storage      
    global dq_storage

    % get the time since last message received
    % --- Settings ---
    sigma = 0.06;        % Smoothing factor
    hw_sigma = 3;        % This results in halfWidth = 2 (0.08s lag)

    tnow = double(ros2time(handles.node,'now').sec) + double(ros2time(handles.node,'now').nanosec) * 1e-9;
    dt = tnow - prevRxTime;
    prevRxTime = tnow;

    % --- Initialization ---
    myFilter = RobotStateFilter(dt, sigma, hw_sigma, 2);

    % Extract position and orientation from the ROS message and assign the
    % data to the global variables.

    if ~isempty(message.data)
        % Cycle through the bending vector list
%         a = prevBendingVec;
        prevBendingVec(1:end-1, :) = prevBendingVec(2:end, :); % move the data up in the stack
        bendingVec = message.data(:)';  % Row vector
        % convert angle from theta to state q
        bendingVec = 2*bendingVec;
        % Convert to radians
        bendingVec = bendingVec*(pi/180);
        % insert new sample
        prevBendingVec(end,:) = bendingVec;
        % finite difference
        % dBendingVec = (prevBendingVec(end,:)- prevBendingVec(end-1,:))/dt;
        % Assuming ‘prevBendingVec’ is your 10x2 matrix
        [bendingVec, dBendingVec] = myFilter.process(prevBendingVec);
        % debugging
        q_storage = [q_storage; bendingVec];
        dq_storage = [dq_storage; dBendingVec];
        disp("Received " + string(bendingVec) + ", velocities " + string(dBendingVec));
    end

end