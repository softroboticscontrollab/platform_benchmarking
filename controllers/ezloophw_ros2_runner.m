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

% % Open loop nominal controller reads from this file:
% ol_hw_datafile = 'ezloophw_closedloop_ros2_pneumatics_2025-4-9_110543.csv';
% 
% % read and preprocess
% ol_hw_data = readtable(ol_hw_datafile, 'HeaderLines', 2, 'VariableNamingRule', 'preserve');
% 
% time     = ol_hw_data.("Test time");
% theta_0
% 
% time_data = ow_hw_data.("Test time");
% smoothed_theta_0 = theta_0;
% smoothed_theta_1 = theta_1;  = ol_hw_data.theta_0;
% theta_1  = ol_hw_data.theta_1;
% mpr_0    = ol_hw_data.MPRpressure_0;
% mpr_1    = ol_hw_data.MPRpressure_1;
% mpr_2    = ol_hw_data.MPRpressure_2;
% mpr_3    = ol_hw_data.MPRpressure_3;
% 
% % Convert angles from degrees to radians
% theta_0 = deg2rad(theta_0);
% theta_1 = deg2rad(theta_1);
% smoothed_mpr_0 = smoothdata(mpr_0, 'sgolay', 12);
% smoothed_mpr_1 = smoothdata(mpr_1, 'sgolay', 12);
% smoothed_mpr_2 = smoothdata(mpr_2, 'sgolay', 12);
% smoothed_mpr_3 = smoothdata(mpr_3, 'sgolay', 12);
% 
% % Get the ...


% constants for the dynamics + robot geometry

m2 = 0.13; % kg
m6 = 0.13;
l1 = 0.122;
l2 = 0.122;

%%% from Juan/Akua as of 2025-07-24
% damping = 10.0;
% k1 = 130.85249; 
% k2 = 165.12965;
% damping = 3.5; % 3.5
% k1 = 75.41558; 
% k2 = 112.84726; 

% most recent?
% k1 = 69.48618;
% k2 = 89.03760;

%%% from Drew recalibration 2025-07-26+
% k1 = 75.5; k2 = 128;

%%% from Akua relacibration 2025-10-27
%k1 = 147.55762; k2 = 253.56177;

%%% from Akua recalibration with big t ags 2026-02-13
k1 = 137.20584; k2 = 239.44695;

damping = 5.0; %used to be 3
% damping = 5.0;
% damping = 10.0; % A larger damping constant means that the simulation has less motion under a change in u.
% damping = 20.0; %...but a larger damping for the model means that the
% CBF-based QP predicts less motion than actually occurs.

tol = 10^4; % numerical tolerance for instability: if f(x) is greater than this, in any element, assume our simulation has encountered a big issue and stop integrating

c.m2 = m2;
c.m6 = m6;
c.damping = damping;
c.k1 = k1;
c.k2 = k2;
c.tol = tol;
c.l1 = l1;
c.l2 = l2;
c.g = 9.81; % not used -- to do, clean up constants

% constants for the nominal controller: sinusoids
c.amp1 = 160; % amp1 = 45 for CBF paper 80
c.amp2 = 160; % amp2 = 45 for CBF paper 80
% c.amp1 = 45; % April 2025 used 45
% c.amp2 = 45; % April 2025 used 45
per1 = 140; % per1 = 60 for CBF paper 100
per2 = 140; % per2 = 60 for CBF paper 100
c.freq1 = 1/per1;
c.freq2 = 1/per2;
c.shift1 = 0;
c.shift2 = 0;
c.u_limit = 250;

% constants for the CBF-based supervisor
F_max = 0.16; % shouldn't this be 0.1786? 11.16*1.6/1000
k_env = 11.16;

%%% Juan/Akua constants as of 2025-07-24
% aE = 5; % 0.2 1 2 20
% bE = 5; % 0.2 1 2 20
% gam = 5; % 0.2 1 2 20

% aE = 10; % even more conservative
% aE = 1050;
% bE = 1050;
% gam = 2000;
% k_env = 10;
% F_max = 20;

%%% Comparison with April 2025 constants
% aE = 2.0; bE = 2.0; gam = 2.0; % medium from the April 2025 paper
% aE = 0.2; bE = 0.2; gam = 0.2; % high from the April 2025 paper

% aE = 20.0; bE = 20.0; gam = 20.0; % low conservative. Fixed as of 2025-07-26
% aE = 5.0; bE = 5.0; gam = 5.0; % medium
% aE = 1.0; bE = 1.0; gam = 1.0; % high

aE = 0.1; bE = 0.1; gam = 0.1;
p_des = [deg2rad(45); deg2rad(45)];  

c.k_env = k_env;
c.F_max = F_max;
c.aE = aE;
c.bE = bE;
c.gam = gam;

%% PID Controller Setups 
Kp = [10; 10]; Kd = [1; 1]; 
Kp_ct = [5;5]; Kd_ct = [1;1];
c.p_des = p_des; c.Kp = Kp; 
c.Kd = Kd; c.Kp_ct = Kp_ct; 
c.Kd_ct = Kd_ct; 
%%
% which controller to choose. This is the combined controller, both nom and
% safe supervisor.

ctrlr = @u_computed_torque_control;

%@u_computed_torque_control
%@u_sim_traj

%% Initialize ROS2 nodes for the MATLAB side
% we are this node for sending control commands
matlabPubNodeName = "matlab_cmder";
matlabPubNode = ros2node(matlabPubNodeName);
% we are this node for receiving joint angles
matlabSubNodeName = "matlab_receiver";
matlabSubNode = ros2node(matlabSubNodeName);

%% --- Setup: Publisher to controller commands topic ---

pubWait = 2;
cmdtopic = '/cmd_u_t';

disp("Starting the publisher, waiting " + string(pubWait) + " seconds...");

controlPub = ros2publisher(matlabPubNode, cmdtopic, 'std_msgs/Float64MultiArray');
controlMsg = ros2message(controlPub);
pause(pubWait); %wait for some time to register publisher on the network

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
prevBendingVec = zeros(n_smooth_prev_velocity, 2); % now storing the last ten samples.
dBendingVec = zeros(1,2);
prevdBendingVec = zeros(n_smooth_prev_velocity, 2);

q_storage = zeros(1,2);
dq_storage = zeros(1,2);

disp('Attempting to start the subscriber...')
while ~subStarted
    try
        bendingSub = ros2subscriber(matlabSubNode, subtopic, {@ezloophwROS2BendingAngleCallback, rxCallbackHandles});
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
timerHandles.timeatstart = double(ros2time(matlabPubNode,'now').sec) + double(ros2time(matlabPubNode,'now').nanosec) * 1e-9;
simTimer = ezloophwROS2Timer(pubRate, {@ezloophwROS2ControlTimer,timerHandles});

%% Main loop. Just waits until user clicks q in window,

disp('Press q in the open MATLAB figure window to stop ROS2 from the MATLAB side.');
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
