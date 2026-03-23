
function [time_vec, q, dq, ddq, smooth_q, smooth_dq, smooth_ddq, indexs, rho0] = ezloopdata_compare(data, doPlot, stepResponse, sinResponse, indexs)
% processEzLoopData  Load and process EZ-Loop ROS2 dataset
%
% INPUT
%   dat5   : string or char
%            Path to CSV data file
%   doPlot : logical (optional)
%            If true, generate comparison plots (default = false)
%
% OUTPUT
%   time_vec   : processed time vector
%   q          : raw joint positions [Nx2]
%   dq         : numerically differentiated velocities [Nx2]
%   ddq        : numerically differentiated accelerations [Nx2]
%   smooth_q   : filtered joint positions from log [Nx2]
%   smooth_dq  : filtered velocities from log [Nx2]
%   smooth_ddq : filtered accelerations from log [Nx2]
%   indexs     : start time of base trajectory
%   rho0         : safety measurement for trajectroy 

% length of limb 1
l1 = 0.122; 
% length of limb 2
l2 = 0.122; 
epsilon = 1e-6; 
% calculated spring constant of deformable force plate   
k = 11.16; 
% allowable max force on force plate - used for force-critical tasks
Fmax = 11.16 * 1.6 / 100; 

if nargin < 2
    doPlot = false;
    stepResponse = false; 
    indexs = 0;
    sinResponse = false;
end

if nargin < 3
    stepResponse = false;
    indexs = 0;
    sinResponse = false;
end

if nargin < 4
    indexs = 0; % used to align respose from Dynamics Controller to calibration results
    sinResponse = false;
end

if nargin < 5
    sinResponse = false;
end 

%% Load data
data = readtable(data, ...
    'HeaderLines',2,'VariableNamingRule','preserve');

time_vec = data.("Test time");

%% Find start index

startidx = find(data.("u_t(0)") ~= 0, 1, 'first');

% If limb 0 never activates, try limb 1
if isempty(startidx)
    startidx = find(data.("u_t(1)") ~= 0, 1, 'first');
end

% If still empty, fall back to start of dataset
if isempty(startidx)
    startidx = 1;
end

% Apply manual alignment offset
startidx = startidx + indexs;

% Prevent invalid indexing
startidx = max(startidx,1);

if stepResponse
    period = 20;

elseif sinResponse
    period = 200;

else
    lastidx = find(data.("u_t(0)") ~= 0, 1, 'last');

    if isempty(lastidx)
        period = time_vec(end) - time_vec(startidx);
    else
        period = time_vec(lastidx) - time_vec(startidx);
    end
end

period_idx = find(time_vec - time_vec(startidx) < period, 1, 'last');

if isempty(period_idx)
    period_idx = length(time_vec) - startidx;
end

indexs = startidx;

endidx = min(startidx + period_idx, length(time_vec));

time_vec = time_vec(startidx:endidx) - time_vec(startidx);

%% Extract angles
theta_0 = data.theta_0(startidx:endidx);
theta_1 = data.theta_1(startidx:endidx);

%% Extract Pressures
pressure_0 = data.MPRpressure_0;
pressure_1 = data.MPRpressure_1;
pressure_2 = data.MPRpressure_2;
pressure_3 = data.MPRpressure_3;

%% Estimated ATM Pressure 
ATM = (pressure_0(1)+pressure_1(1)+pressure_2(1)+pressure_3(1))/4 - 1;

%% Extract Pressures
pressure_0 = pressure_0(startidx:endidx);
pressure_1 = pressure_1(startidx:endidx);
pressure_2 = pressure_2(startidx:endidx);
pressure_3 = pressure_3(startidx:endidx);

%% Solve for absolute pressure for comparison with control input
abs_0 = pressure_0 - ATM;
abs_1 = pressure_1 - ATM;
abs_2 = pressure_2 - ATM;
abs_3 = pressure_3 - ATM;

%% Extract Valve Aperture data 
val0 = data.ValveOpening_0(startidx:endidx);
val1 = data.ValveOpening_1(startidx:endidx);
val2 = data.ValveOpening_2(startidx:endidx);
val3 = data.ValveOpening_3(startidx:endidx);

%% Extract Command Inputs
u0 = data.("u_t(0)")(startidx:endidx);
u1 = data.("u_t(1)")(startidx:endidx);

%% Extract Force Measurements
f0 = data.("ForcePlate_0"); % (forceplate in positive direction)
f1 = data.("ForcePlate_1"); % only needed if using two forceplates (forceplate in negative direction)

f0   = f0(startidx:endidx)/1000*9.8;
f1   = f1(startidx:endidx)/1000*9.8;

rho0 = (Fmax - f0)/Fmax;
rho1 = (Fmax - f1)/Fmax;

%% Solve for Error 
error0 = zeros(1,length(u0)); pos0 = u0 > 0; neg0 = u0 < 0;
error1 = zeros(1,length(u1)); pos1 = u1 > 0; neg1 = u1 < 0;

error0(pos0) = -abs_0(pos0) + u0(pos0); % chamber 0
error0(neg0) = abs_1(neg0) + u0(neg0); % chamber 1
error0(u0==0) = abs_0(u0==0);
error1(pos1) = -abs_2(pos1) + u1(pos1); % chamber 2
error1(neg1) = abs_3(neg1) + u1(neg1); % chamber 3
error1(u1==0) = abs_2(u1==0);

int_error0 = zeros(1,length(u0));
int_error1 = zeros(1,length(u0));

for k = 2:length(error0)
    dt = time_vec(k) - time_vec(k-1);
    int_error0(k) = sum(error0(1:k-1))*dt;
    int_error1(k) = sum(error1(1:k-1))*dt;
end

%% Convert to state coordinates (radians)
q0 = deg2rad(theta_0 * 2);
q1 = deg2rad(theta_1 * 2);

%% Compute derivatives
[dq0, ddq0] = computeDerivatives(q0, time_vec); 
[dq1, ddq1] = computeDerivatives(q1, time_vec);

q   = [q0 q1];
dq  = [dq0 dq1];
ddq = [ddq0 ddq1];

%% Load smoothed signals from log
smooth_q = [
    data.smoothed_q_0(startidx:endidx), ...
    data.smoothed_q_1(startidx:endidx)
];

smooth_dq = [
    data.smoothed_dq_0(startidx:endidx), ...
    data.smoothed_dq_1(startidx:endidx)
];

try
    smooth_ddq = [
        data.ddq_0(startidx:endidx), ...
        data.ddq_1(startidx:endidx)
    ];
catch
    disp('ddq data not under name data.ddq');
end
try
    smooth_ddq = [
        data.smoothed_ddq_0(startidx:endidx), ...
        data.smoothed_ddq_1(startidx:endidx)
    ];
catch
    disp('ddq data not under name data.smoothed_ddq');
end

%% Plot comparison (optional)

if doPlot
    plot1 = figure;

    subplot(2,2,1)
    plot(time_vec,q(:,1),LineWidth=2,Color='k'); hold on
    plot(time_vec,smooth_q(:,1))
    grid on
    hold off
    title('q_0')

    subplot(2,2,3)
    plot(time_vec,q(:,2),LineWidth=2,Color='k'); hold on
    plot(time_vec,smooth_q(:,2))
    grid on
    hold off
    title('q_1')

    subplot(2,2,2)
    plot(time_vec,dq(:,1),LineWidth=2,Color='k'); hold on
    plot(time_vec,smooth_dq(:,1))
    grid on
    hold off
    title('dq_0')

    subplot(2,2,4)
    plot(time_vec,dq(:,2),LineWidth=2,Color='k'); hold on
    plot(time_vec,smooth_dq(:,2))
    grid on
    hold off
    title('dq_1')

    %% Control Comparison
    % when viewing these plots it is important to not what the control
    % input corresponds to in the data. For the case of the current
    % infrastructure, control input corresponds to pressure increasing from
    % atmospheric pressure per chamber. The sign of the control input
    % corresponds to which side of the chamber is being inflated.

    plot2 = figure;
    subplot(2,4,1)
    plot(time_vec,u0); hold on
    plot(time_vec,abs_0)
    grid on 
    hold off 
    xlim([0 time_vec(floor(length(time_vec)/2))])
    title('Chamber0 Step Response')

    subplot(2,4,2)
    plot(time_vec,u0*-1); hold on
    plot(time_vec,abs_1)
    grid on 
    hold off 
    if stepResponse
        xlim([0 time_vec(floor(length(time_vec)/2))])
    else
        xlim([time_vec(floor(length(time_vec)/2)) time_vec(end)])
    end
    title('Chamber1 Step Response')
                
    subplot(2,4,3)
    plot(time_vec,u1); hold on
    plot(time_vec,abs_2)
    grid on 
    hold off 
    xlim([0 time_vec(floor(length(time_vec)/2))])
    title('Chamber2 Step Response')

    subplot(2,4,4)
    plot(time_vec,u1*-1); hold on
    plot(time_vec,abs_3)
    grid on 
    hold off 
    if stepResponse
        xlim([0 time_vec(floor(length(time_vec)/2))])
    else
        xlim([time_vec(floor(length(time_vec)/2)) time_vec(end)])
    end
    title('Chamber3 Step Response')

    subplot(2,4,5)
    plot(time_vec,val0);
    grid on 
    xlim([0 time_vec(floor(length(time_vec)/2))])
    title('Valve Aperture Response')

    subplot(2,4,6)
    plot(time_vec,val1)
    grid on 
    if stepResponse
        xlim([0 time_vec(floor(length(time_vec)/2))])
    else
        xlim([time_vec(floor(length(time_vec)/2)) time_vec(end)])
    end
    title('Valve Aperture Response')
                
    subplot(2,4,7)
    plot(time_vec,val2)
    grid on 
    xlim([0 time_vec(floor(length(time_vec)/2))])
    title('Valve Aperture Response')

    subplot(2,4,8)
    plot(time_vec,val3)
    grid on 
    if stepResponse
        xlim([0 time_vec(floor(length(time_vec)/2))])
    else
        xlim([time_vec(floor(length(time_vec)/2)) time_vec(end)])
    end
    title('Valve Aperture Response')

    %% Plotting associated with errors

    plot3 = figure; 
    subplot(2,4,1)
    plot(time_vec(pos0),error0(pos0))
    grid on 
    xlim([0 time_vec(floor(length(time_vec)/2))])
    title('Pressure Error in chamber 1')

    subplot(2,4,2)
    plot(time_vec(neg0),error0(neg0))
    grid on 
    if stepResponse
        xlim([0 time_vec(floor(length(time_vec)/2))])
    else
        xlim([time_vec(floor(length(time_vec)/2)) time_vec(end)])
    end
    title('Pressure Error in chamber 2')

    subplot(2,4,3)
    plot(time_vec(pos1),error1(pos1))
    grid on 
    xlim([0 time_vec(floor(length(time_vec)/2))])
    title('Pressure Error in chamber 3')

    subplot(2,4,4)
    plot(time_vec(neg1),error1(neg1))
    grid on 
    if stepResponse
        xlim([0 time_vec(floor(length(time_vec)/2))])
    else
        xlim([time_vec(floor(length(time_vec)/2)) time_vec(end)])
    end
    title('Pressure Error in chamber 4')

    subplot(2,4,5)
    plot(time_vec(pos0),int_error0(pos0))
    grid on 
    xlim([0 time_vec(floor(length(time_vec)/2))])
    title('Pressure Integral Error in chamber 1')

    subplot(2,4,6)
    plot(time_vec(neg0),int_error0(neg0))
    grid on 
    if stepResponse
        xlim([0 time_vec(floor(length(time_vec)/2))])
    else
        xlim([time_vec(floor(length(time_vec)/2)) time_vec(end)])
    end
    title('Pressure Integral Error in chamber 2')

    subplot(2,4,7)
    plot(time_vec(pos1),int_error1(pos1))
    grid on 
    xlim([0 time_vec(floor(length(time_vec)/2))])
    title('Pressure Integral Error in chamber 3')

    subplot(2,4,8)
    plot(time_vec(neg1),int_error1(neg1))
    grid on 
    if stepResponse
        xlim([0 time_vec(floor(length(time_vec)/2))])
    else
        xlim([time_vec(floor(length(time_vec)/2)) time_vec(end)])
    end
    title('Pressure Integral Error in chamber 4')     
end

end

function [dq, ddq] = computeDerivatives(q, t)
% computeDerivatives  Compute first and second derivatives of a signal
% using finite differences with non-uniform time steps.
%
% Inputs:
%   q : Nx1 signal vector
%   t : Nx1 time vector
%
% Outputs:
%   dq  : Nx1 first derivative
%   ddq : Nx1 second derivative

N = length(q);

dq  = zeros(N,1);
ddq = zeros(N,1);

% First derivative
for i = 1:N-1
    dt = t(i+1) - t(i);
    dq(i) = (q(i+1) - q(i)) / dt;
end
dq(N) = dq(N-1);   % pad final element

% Second derivative
for i = 1:N-1
    dt = t(i+1) - t(i);
    ddq(i) = (dq(i+1) - dq(i)) / dt;
end
ddq(N) = ddq(N-1); % pad final element

end