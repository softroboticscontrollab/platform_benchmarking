function ezloophwROS2ControlTimer(~, ~, handles)
    %ezloophwROS2ControlTimer - Timer update function that sends whatever
    % control signal is calculated by handles.ctrlr. Starts at t=0 when the
    %timer is created.
    % (C) Soft Robotics Control Lab, 2025  

    % it's cleaner if we get the measurement here and then pass it in, that
    % way the controller function doesn't have to know about ROS.
    global bendingVec;
    global dBendingVec;

    % these are both stored as row vectors for ROS2 compatibility
    x_t = [bendingVec'; dBendingVec'];

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
end