function ezloophwROS2ControlTimerV2(~, ~, handles)
    %ezloophwROS2ControlTimer - Timer update function that sends whatever
    % control signal is calculated by handles.ctrlr. Starts at t=0 when the
    %timer is created.
    % (C) Soft Robotics Control Lab, 2025  

    % it's cleaner if we get the measurement here and then pass it in, that
    % way the controller function doesn't have to know about ROS.
    global bendingVec;
    global dBendingVec;
    global ddBendingVec;

    % these are both stored as row vectors for ROS2 compatibility
    x_t = [bendingVec'; dBendingVec';ddBendingVec'];

    % Update the control message values
    if isvalid(handles.controlPub)
        % net time since start, in seconds
        tnow = double(ros2time(handles.node,'now').sec) + double(ros2time(handles.node,'now').nanosec) * 1e-9;
        t = tnow - handles.timeatstart;
        u = handles.ctrlr(x_t, handles.c, t);
        handles.controlPubmsg.data = u;
        % Debugging:
        % disp(handles.controlPubmsg.data)
        % Publish the control input message
        send(handles.controlPub, handles.controlPubmsg);
    end
    
   % Publish extra telemetry topics for logging in Python
    
    % Publish filtered/smoothed joint angles
    if isfield(handles.publishers, 'smoothed_q')
        smoothed_q_msg = handles.publishers.smoothed_q.msg;
        smoothed_q_msg.data = bendingVec'; % Data passed as an array
        send(handles.publishers.smoothed_q.pub, smoothed_q_msg);
    end
    
    % Publish filtered/smoothed joint velocities
    if isfield(handles.publishers, 'smoothed_dq')
        smoothed_dq_msg = handles.publishers.smoothed_dq.msg;
        smoothed_dq_msg.data = dBendingVec'; 
        send(handles.publishers.smoothed_dq.pub, smoothed_dq_msg);
    end
    
    % Publish joint accelerations or secondary metrics
    if isfield(handles.publishers, 'smoothed_ddq')
        smoothed_ddq_msg = handles.publishers.smoothed_ddq.msg;
        smoothed_ddq_msg.data = ddBendingVec'; % Placeholder for acceleration values
        send(handles.publishers.smoothed_ddq.pub, smoothed_ddq_msg);
    end

end