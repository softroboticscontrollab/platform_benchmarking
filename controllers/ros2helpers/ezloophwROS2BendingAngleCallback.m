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
    tnow = double(ros2time(handles.node,'now').sec) + double(ros2time(handles.node,'now').nanosec) * 1e-9;
    dt = tnow - prevRxTime;
    prevRxTime = tnow;

    % Extract position and orientation from the ROS message and assign the
    % data to the global variables.

    if ~isempty(message.data)
        % Cycle through the bending vector list
        a = prevBendingVec;
        prevBendingVec(1:end-1, :) = prevBendingVec(2:end, :);
        bendingVec = message.data(:)';  % Row vector
        % convert angle from theta to state q
        bendingVec = 2*bendingVec;
        % Convert to radians
        bendingVec = bendingVec*(pi/180);
        % insert new sample
        prevBendingVec(end,:) = bendingVec;
        % finite difference
        q_storage = [q_storage; bendingVec];
        % dBendingVec = (bendingVec - prevBendingVec)/dt; % wrong dimensional
        % averaging filter
        
        % average velocity of the last ten
%         dBendingVec = sum(prevBendingVec-a,1)/n_smooth_prev_velocity/dt;
        % no average velocity, just current velocity
        dBendingVec = (prevBendingVec(end,:)- prevBendingVec(end-1,:))/dt;
        % debugging
        dq_storage = [dq_storage; dBendingVec];
        disp("Received " + string(bendingVec) + ", velocities " + string(dBendingVec));
    end

end