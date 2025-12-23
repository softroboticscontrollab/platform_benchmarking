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

    % get the time since last message received
    tnow = double(ros2time(handles.node,'now').sec) + double(ros2time(handles.node,'now').nanosec) * 1e-9;
    dt = tnow - prevRxTime;
    prevRxTime = tnow;

    % Extract position and orientation from the ROS message and assign the
    % data to the global variables.

    if ~isempty(message.data)
        % Cycle through the bending vector list
        prevBendingVec(1:end-1, :) = prevBendingVec(2:end, :);
        bendingVec = message.data(:)';  % Row vector
        % Convert to radians
        bendingVec = bendingVec*(pi/180);
        % convert angle from theta to state q
        bendingVec = 2*bendingVec;
        % insert new sample
        prevBendingVec(end,:) = bendingVec;
        % finite difference
        % dBendingVec = (bendingVec - prevBendingVec)/dt;
        % averaging filter
        dBendingVec = sum(prevBendingVec,1)/n_smooth_prev_velocity;
        % debugging
        disp("Received " + string(bendingVec) + ", velocities " + string(dBendingVec));
    end

end