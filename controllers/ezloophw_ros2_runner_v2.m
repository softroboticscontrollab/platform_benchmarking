% % ezloophw_ros2_softcbf.m
% (C) Soft Robotics Control Lab 2025
% Closed loop control of a 2-segment soft manipulator over ROS2 with safety
% via a CBF-based QP supervisory control system.

%%% Setup just to make sure we start at the same state each time. 
clear all
close all
clc;

disp('Closed loop controller for ezloophw over ROS2, soft manipulator, 2 segments, with CBF.');
disp('(C) Soft Robotics Control Lab at Boston University, 2025');
disp('creating the nodes / starting ros2...');

%% Setup and path management

roshelperspath = "ros2helpers";
addpath(roshelperspath);

% From the CBF simulation code:
% add paths to the dynamics functions, etc., in other folders in this
% repository.
basedir = '..'; % assuming we are in a subdirectory
pathstoadd = genpath(basedir);
% this includes .git folders, which is bad, so add everything then manually
% remove the .git
pathstoremove = genpath(strcat(basedir,'\.git'));
addpath(pathstoadd);
rmpath(pathstoremove);

% also load the plotting functions and helpers.
helpdir = '../../helpers';
plotdir = '../../plotting_soft';
addpath(genpath(helpdir));
addpath(genpath(plotdir));
constraintsdir = '../../polyhedron_constraints';
addpath(genpath(constraintsdir));

%% Setup and constants for the controllers and dynamics model

% constants for the dynamics + robot geometry

m2 = 0.13; % kg
m6 = 0.13;
l1 = 0.122; % m
l2 = 0.122;

%%%% new calibration 2026 mar 25 - post butterworth online smoothing
k1 = 143.20623; k2 = 245.96992;

damping = 5; 

tol = 10^4; % numerical tolerance for instability: if f(x) is greater than this, in any element, assume our simulation has encountered a big issue and stop integrating

c.m2 = m2;
c.m6 = m6;
c.damping = damping;
c.k1 = k1;
c.k2 = k2;
c.tol = tol;
c.l1 = l1;
c.l2 = l2;
c.g = 9.81; 

% constants for the nominal controller: sinusoids
c.amp1 = 100; 
c.amp2 = 100; 
per1 = 200; 
per2 = 200; 
c.freq1 = 1/per1;
c.freq2 = 1/per2;
c.shift1 = 0;
c.shift2 = 0;
c.u_limit = 250;

% constants for the CBF-based supervisor
F_max = 11.16*1.6/1000;
k_env = 11.16;

aE = 0.1; bE = 0.1; gam = 0.05;
% Use for medium conservativeness aE = 2; bE = 2; gam = 0.05
% Use for high conservativeness aE = 0.1; bE = 0.1; gam = 0.05

p_des = [deg2rad(20); -1*deg2rad(20)];  

c.k_env = k_env;
c.F_max = F_max;
c.aE = aE;
c.bE = bE;
c.gam = gam;

%% PID Controller Setups 
Kp = [212; 280]; Kd = [20; 22]; 
Kp_ct = [5; 5]; Kd_ct = [1;1];
c.p_des = p_des; c.Kp = Kp; 
c.Kd = Kd; c.Kp_ct = Kp_ct; 
c.Kd_ct = Kd_ct; 
%%
% which controller to choose. This is the combined controller, both nom and
% safe supervisor.

% ctrlr = @u_Pressuretunner;

% ctrlr = @u_pd_control;

ctrlr = @u_pd_control_trajectory;

% ctrlr = @u_computed_torque_control;

% ctrlr = @u_softcbf_combined;

%% Initialize ROS2 nodes for the MATLAB side
% we are this node for sending control commands
matlabPubNodeName = "matlab_cmder";
matlabPubNode = ros2node(matlabPubNodeName);
% we are this node for receiving joint angles
matlabSubNodeName = "matlab_receiver";
matlabSubNode = ros2node(matlabSubNodeName);

%% --- Setup: Publisher to controller commands topic ---

pubWait = 2;
cmdtopic = '/u_t';

disp("Starting the publisher, waiting " + string(pubWait) + " seconds...");

controlPub = ros2publisher(matlabPubNode, cmdtopic, 'std_msgs/Float64MultiArray','History', 'keeplast', 'Depth', 1);
controlMsg = ros2message(controlPub);
pause(pubWait); %wait for some time to register publisher on the network

% Define the extra topics we want to send to python script e.g. filtered
% bending angles and its derivatives
extraTopics = {'smoothed_q', 'smoothed_dq', 'smoothed_ddq'};
timerHandles.publishers = struct(); % Sub-struct to hold publishers

%% --- Setup: Subscriber to /bending_angles ---

subStarted = 0;
substartwait = 5;
subtopic = 'ezloophw_sensors';

% currently unused, but this variable would contain the data received from
% ros
global bendingVec;
global prevBendingVec; % for velocities finite difference
global prevdBendingVec; 
global n_smooth_prev_velocity; % velocity smoothing
global dBendingVec; % the calculated velocity finite difference
global ddBendingVec; % the calculated acceleration finite difference 

global q_storage; % storing values for debugging 
global dq_storage; % storing values for debugging

rxCallbackHandles.node = matlabSubNode;
% for calculating dq/dt
timeatstart = double(ros2time(matlabSubNode,'now').sec) + double(ros2time(matlabSubNode,'now').nanosec) * 1e-9;
global prevRxTime;
prevRxTime = timeatstart;

% initialize the variables that will be assigned in the callback
% stored as row vectors for ROS2
bendingVec = zeros(1,2);
n_smooth_prev_velocity = 10;
dBendingVec = zeros(1,2);
ddBendingVec = zeros(1,2);

q_storage = zeros(1,2);
dq_storage = zeros(1,2);

disp('Attempting to start the subscriber...')
while ~subStarted
    try
        bendingSub = ros2subscriber(matlabSubNode, subtopic, {@ezloophwROS2BendingAngleCallback, rxCallbackHandles},'History', 'keeplast', 'Depth', 1);
        subStarted = 1;
    catch
        disp("ERROR! You need to start the publisher in python for topic:");
        disp(subtopic)
        disp("Waiting another " + string(substartwait) + " seconds, trying again...");
        pause(substartwait);
    end
end

%% --- Start the publisher ---

% pubRate = 1;    % 1 Hz publish
% pubRate = 5.0;    % this is period not frequency
pubRate = 0.05;     % this is half one control period, dt=0.1 sec


% Create a timer for publishing messages and assign appropriate handles
% The timer will call exampleHelperROSSimTimer at a rate of pubRate.
timerHandles.controlPub = controlPub;
timerHandles.controlPubmsg = controlMsg;
timerHandles.node = matlabPubNode;

timerHandles.c = c;
timerHandles.ctrlr = ctrlr;

% timerHandles.timeatstart = ros2time(matlabPubNode, 'now');
for i = 1:length(extraTopics)
    topicName = extraTopics{i};
    % Store publisher and message in the handles struct
    timerHandles.publishers.(topicName).pub = ros2publisher(matlabPubNode, topicName, 'std_msgs/Float64MultiArray','History', 'keeplast', 'Depth', 1);
    timerHandles.publishers.(topicName).msg = ros2message(timerHandles.publishers.(topicName).pub);
end

timerHandles.timeatstart = double(ros2time(matlabPubNode,'now').sec) + double(ros2time(matlabPubNode,'now').nanosec) * 1e-9;
simTimer = ezloophwROS2Timer(pubRate, {@ezloophwROS2ControlTimerV2,timerHandles});

%% Main loop. Just waits until user clicks q in window,

disp('Press q in the open MATLAB figure  window to stop ROS2 from the MATLAB side.');
disp('If you close the window by accident, run clear all from the matlab command prompt.')

% so that the window can stop ros nodes
global stopFlag
stopFlag = false;

fig = figure('Name', 'Press "q" to quit');
set(fig, 'KeyPressFcn', @keyPressCallback);

while ~stopFlag
    % Allow figure window and callbacks to process
    drawnow;
end

close all; % close the window so we know that we're done


disp('Shutting down MATLAB ROS nodes...');
clear bendingSub
clear simTimer
clear matlabPubNode
clear matlabSubNode


function keyPressCallback(~, event)
    global stopFlag
    if strcmp(event.Key, 'q')
        stopFlag = true;
    end
end
