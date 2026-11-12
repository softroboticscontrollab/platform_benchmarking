%% Plotting Code used for plotting of data in Paper
% clear all; close all; clc;

%% Setup
% values here are chosen based on physical hardware

% length of limb 1
l1 = 0.122; 
% length of limb 2
l2 = 0.122; 
epsilon = 1e-6; 
% calculated spring constant of deformable force plate   
k = 11.16; 
% allowable max force on force plate - used for force-critical tasks
Fmax = 11.16 * 1.6 / 100; 

initial_wait = 5;
hold_time = 60;
reset_time = 20;

q_des = 40;
num_z_values = 10;
repetitions = 2;

steady_window = 15;  % Final 10 seconds of each hold

beta0 = -1.264908;
beta1 = 0.423700;
beta2 = -0.268104;

%% Example Code

%RTJ = 'data/ezloophw_self_stiffening_2026-10-26_130010.csv'; % first time with qd = 20 degrees
%RTJ = 'data/ezloophw_self_stiffening_2026-10-27_20.csv'; % second time with qd = 20 degrees
%RTJ = 'data/ezloophw_self_stiffening_2026-10-27_40.csv'; % first time with qd = 40 degrees
%RTJ = 'data/ezloophw_self_stiffening_2026-10-28_40f50.csv'; % first time with qd = 40 degrees and added torque 50g mass
RTJ = 'data/ezloophw_self_stiffening_2026-11-3_40f50v2.csv'; % first time with qd = 40 degrees and added torque 50g mass version 2

RTJ = readtable(RTJ, ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');

time     = RTJ.("Test time");
time     = time(1:end);
theta_0  = RTJ.theta_0;
theta_0  = theta_0(1:end);
theta_1  = RTJ.theta_1;
theta_1  = theta_1(1:end);

startidx = find(RTJ.("u_t(0)"), 1, 'first');

endidx   = find(RTJ.("u_t(0)"), 1, 'last');

u0 = RTJ.("u_t(0)")(startidx:endidx);
u1 = RTJ.("u_t(1)")(startidx:endidx);
u2 = RTJ.("u_t(2)")(startidx:endidx);
u3 = RTJ.("u_t(3)")(startidx:endidx);

last_time = endidx - startidx;

time = time - time(startidx);
time = time(startidx:endidx);

theta_0 = theta_0(startidx:endidx);
theta_1 = theta_1(startidx:endidx);

% Convert angles from degrees to radians
q0_rad  = deg2rad(theta_0*2);
q1_rad  = deg2rad(theta_1*2);

smooth_q0 = rad2deg(RTJ.smoothed_q_0(startidx:endidx));
smooth_q1 = rad2deg(RTJ.smoothed_q_1(startidx:endidx));

%% Plot

% plot(time,theta_0*2)

%% Troubleshooting

p0 = RTJ.MPRpressure_0(startidx:endidx);
p1 = RTJ.MPRpressure_1(startidx:endidx);
p2 = RTJ.MPRpressure_2(startidx:endidx);
p3 = RTJ.MPRpressure_3(startidx:endidx);

p0 = p0 - p0(1);
p1 = p1 - p1(1);
p2 = p2 - p2(1);
p3 = p3 - p3(1);

figure()
subplot(3,1,1)
plot(time,u0)
hold on
plot(time,p0)
title('u0 vs p0: first40')
hold off


subplot(3,1,2)
plot(time,u1)
hold on
plot(time,p1)
title('u1 vs p1: first40')
hold off


Predict_angle = beta0 + beta1*p0 + beta2*p1;

subplot(3,1,3)
plot(time,Predict_angle)
hold on
plot(time,smooth_q0)
title('Predicted Position vs Actual: first40')
hold off

%% Calculating increase in stiffness ...

flag = true;

if flag
    
    mass = 0.050; % kilogram 
    g = 9.81; % m/s^2
    moment_arm = l1; 

    tau_ext = mass*g*moment_arm;

    % RECONSTRUCT NULL-SPACE INPUTS

    B = [beta1 beta2];
    N = [-B(2); B(1)]/norm(B);

    p_nominal = pinv(B)*(q_des-beta0);

    p_min = 0;
    z_step = 10;

    z_min = max((p_min-p_nominal)./N);
    z_start = ceil(z_min/z_step)*z_step;

    z_unique = z_start + (0:num_z_values-1)*z_step;
    z_values = repelem(z_unique,repetitions);

    % EXTRACT STEADY-STATE ANGLES

    % Use your existing measured angle and time arrays.
    % Replace these variable names if necessary.
    time = time(:);
    q_measured = smooth_q0(:);

    num_trials = num_z_values*repetitions;
    trial_duration = hold_time+reset_time;

    q_steady = nan(num_trials,1);

    for i = 1:num_trials

        t_start = initial_wait+(i-1)*trial_duration;
        t_end = t_start+hold_time;

        idx = time >= (t_end-steady_window) & ...
              time < t_end;

        if any(idx)
            q_steady(i) = mean(q_measured(idx),'omitnan');
        end

    end

    % ANGULAR DEFLECTION BETWEEN REPETITIONS

    % First repetition: unloaded
    q_unloaded = q_steady(1:2:end);

    % Second repetition: loaded
    q_loaded = q_steady(2:2:end);

    delta_q_deg = q_loaded-q_unloaded;
    delta_q_rad = deg2rad(delta_q_deg);

    % EFFECTIVE ROTATIONAL STIFFNESS

    K_eff = tau_ext./abs(delta_q_rad);

    % Exclude nearly zero deflections
    K_eff(abs(delta_q_deg)<0.1) = NaN;

    % RESULTS TABLE

    results = table( ...
        z_unique(:), ...
        q_unloaded(:), ...
        q_loaded(:), ...
        delta_q_deg(:), ...
        K_eff(:), ...
        'VariableNames', ...
        {'z_hPa','q_unloaded_deg','q_loaded_deg', ...
         'delta_q_deg','K_eff_Nm_rad'});

    disp(results);

    % PLOT STIFFNESS VS NULL-SPACE INPUT

    figure('Color','w');
    subplot(2,1,1)

    plot(z_unique,K_eff,'o-', ...
        'LineWidth',1.5, ...
        'MarkerSize',7);

    xlabel('$z$ (hPa)','Interpreter','latex');
    ylabel('$K_{\mathrm{eff}}$ (N m/rad)', ...
        'Interpreter','latex');

    title('Effective Rotational Stiffness vs Null-Space Input', ...
        'Interpreter','latex');

    grid on;
    box off;

    % PLOT ANGULAR DEFLECTION VS NULL-SPACE INPUT

    subplot(2,1,2)

    plot(z_unique,delta_q_deg,'o-', ...
        'LineWidth',1.5, ...
        'MarkerSize',7);

    xlabel('$z$ (hPa)','Interpreter','latex');
    ylabel('$\Delta q$ (deg)','Interpreter','latex');

    title('Load-Induced Angular Deflection vs Null-Space Input', ...
        'Interpreter','latex');

    grid on;
    box off;

end 