%% Example Code for plotting of data 
clear all; close all; clc;
l1 = 0.122; 
% length of limb 2
l2 = 0.122; 
epsilon = 1e-6; 
% calculated spring constant of deformable force plate   
k = 11.16; 
%% Using the new function 

% Format of data extractor and plotter. 
% data : .csv file location 
% doPlot : plots generated for general debugging 
% stepResponse : useful flag for visualizing step resposne behavior 
% sinResponse : useful when looking at sine data. Do not use for
% teach-and-repeat
% indexs : time values useful for plotting multiple data files on the same
% plot
% ezloopdata_compare(data,doplot,stepResponse,indexs,sinResponse)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% final tuning response for chamber 1
% dat = 'tuning/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-16_tuningC1.csv'; 
% ezloopdata_compare(dat, false, true);
% 
% % final tuning resposne for chamber 2
% dat = 'tuning/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-16_tuningC2.csv'; 
% ezloopdata_compare(dat, false, true);
% 
% % final tuning resposne for chamber 3
% dat = 'tuning/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-16_tuningC3.csv'; 
% ezloopdata_compare(dat, false, true);
% 
% % final tuning resposne for chamber 4
% dat = 'tuning/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-16_tuningC4.csv'; 
% ezloopdata_compare(dat, false, true);

% Open-loop motion for dynamics calibration with amplitude of 100 and period of 200
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_CalibrationRunV1.csv';
[time_cal, q_cal, dq_cal, ddq_cal, smooth_q_cal, smooth_dq_cal, smooth_ddq_cal, calib_start,~,calib_cont] = ezloopdata_compare(dat, false, false, true, 0);

% PD Tuning Results 
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PD1.csv'; 
[time_PD1, q_PD1, dq_PD1, ddq_PD1, smooth_q_PD1, smooth_dq_PD1, smooth_ddq_PD1,~,~,cont_PD1] = ezloopdata_compare(dat, false, false, true, 0);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PD2.csv'; 
[time_PD2, q_PD2, dq_PD2, ddq_PD2, smooth_q_PD2, smooth_dq_PD2, smooth_ddq_PD2,~,~,cont_PD2] = ezloopdata_compare(dat, false, false, true, 0);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PD3.csv'; 
[time_PD3, q_PD3, dq_PD3, ddq_PD3, smooth_q_PD3, smooth_dq_PD3, smooth_ddq_PD3,~,~,cont_PD3] = ezloopdata_compare(dat, false, false, true, 0);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PD4.csv'; 
[time_PD4, q_PD4, dq_PD4, ddq_PD4, smooth_q_PD4, smooth_dq_PD4, smooth_ddq_PD4,~,~,cont_PD4] = ezloopdata_compare(dat, false, false, true, 0);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PD5.csv'; 
[time_PD5, q_PD5, dq_PD5, ddq_PD5, smooth_q_PD5, smooth_dq_PD5, smooth_ddq_PD5,~,~,cont_PD5] = ezloopdata_compare(dat, false, false, true, 0);

smooth_q_cells = {smooth_q_PD1, smooth_q_PD2, smooth_q_PD3, ...
                  smooth_q_PD4, smooth_q_PD5};

smooth_dq_cells = {smooth_dq_PD1, smooth_dq_PD2, smooth_dq_PD3, ...
                  smooth_dq_PD4, smooth_dq_PD5};

u_cells = {cont_PD1(:,5:6), cont_PD2(:,5:6), cont_PD3(:,5:6), ...
                  cont_PD4(:,5:6), cont_PD5(:,5:6)};

time_cells = {time_PD1, time_PD2, time_PD3, time_PD4, time_PD5};

rho_cells = {};

time_step = time_PD1;

p_des = [deg2rad(20); -1*deg2rad(20)]; 

smooth_q_step = [p_des(1)*ones(length(time_PD1),1) p_des(2)*ones(length(time_PD1),1)];

TunePD = ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_step, smooth_q_step, false, rho_cells,false,smooth_dq_cells,u_cells);

% PD Attempt at Tracking Calibration 
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PDCal1.csv'; 
[time_PDCal1, q_PDCal1, dq_PDCal1, ddq_PDCal1, smooth_q_PDCal1, smooth_dq_PDCal1, smooth_ddq_PDCal1,~,~,cont_PDCal1] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PDCal2.csv'; 
[time_PDCal2, q_PDCal2, dq_PDCal2, ddq_PDCal2, smooth_q_PDCal2, smooth_dq_PDCal2, smooth_ddq_PDCal2,~,~,cont_PDCal2] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PDCal3.csv'; 
[time_PDCal3, q_PDCal3, dq_PDCal3, ddq_PDCal3, smooth_q_PDCal3, smooth_dq_PDCal3, smooth_ddq_PDCal3,~,~,cont_PDCal3] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PDCal4.csv'; 
[time_PDCal4, q_PDCal4, dq_PDCal4, ddq_PDCal4, smooth_q_PDCal4, smooth_dq_PDCal4, smooth_ddq_PDCal4,~,~,cont_PDCal4] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PDCal5.csv'; 
[time_PDCal5, q_PDCal5, dq_PDCal5, ddq_PDCal5, smooth_q_PDCal5, smooth_dq_PDCal5, smooth_ddq_PDCal5,~,~,cont_PDCal5] = ezloopdata_compare(dat, false, false, true, calib_start);

smooth_q_cells = {smooth_q_PDCal1, smooth_q_PDCal2, smooth_q_PDCal3, ...
                  smooth_q_PDCal4, smooth_q_PDCal5};

smooth_dq_cells = {smooth_dq_PDCal1, smooth_dq_PDCal2, smooth_dq_PDCal3, ...
                  smooth_dq_PDCal4, smooth_dq_PDCal5};

u_cells = {cont_PDCal1(:,5:6), cont_PDCal2(:,5:6), cont_PDCal3(:,5:6), ...
                  cont_PDCal4(:,5:6), cont_PDCal5(:,5:6)};

time_cells = {time_PDCal1, time_PDCal2, time_PDCal3, time_PDCal4, time_PDCal5};

rho_cells = {};

CalibPD = ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_cal, smooth_q_cal, false, rho_cells,false,smooth_dq_cells,u_cells);

% CD Results tracking calib trajectory
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_TrackCalib1.csv'; 
[time_tra1, q_tra1, dq_tra1, ddq_tra1, smooth_q_tra1, smooth_dq_tra1, smooth_ddq_tra1,~,~,cont_tra1] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_TrackCalib2.csv'; 
[time_tra2, q_tra2, dq_tra2, ddq_tra2, smooth_q_tra2, smooth_dq_tra2, smooth_ddq_tra2,~,~,cont_tra2] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_TrackCalib3.csv'; 
[time_tra3, q_tra3, dq_tra3, ddq_tra3, smooth_q_tra3, smooth_dq_tra3, smooth_ddq_tra3,~,~,cont_tra3] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_TrackCalib4.csv'; 
[time_tra4, q_tra4, dq_tra4, ddq_tra4, smooth_q_tra4, smooth_dq_tra4, smooth_ddq_tra4,~,~,cont_tra4] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_TrackCalib5.csv'; 
[time_tra5, q_tra5, dq_tra5, ddq_tra5, smooth_q_tra5, smooth_dq_tra5, smooth_ddq_tra5,~,~,cont_tra5] = ezloopdata_compare(dat, false, false, true, calib_start);

smooth_q_cells = {smooth_q_tra1, smooth_q_tra2, smooth_q_tra3, ...
                  smooth_q_tra4, smooth_q_tra5};

smooth_dq_cells = {smooth_dq_tra1, smooth_dq_tra2, smooth_dq_tra3, ...
                  smooth_dq_tra4, smooth_dq_tra5};

u_cells = {cont_tra1(:,5:6), cont_tra2(:,5:6), cont_tra3(:,5:6), ...
                  cont_tra4(:,5:6), cont_tra5(:,5:6)};

time_cells = {time_tra1, time_tra2, time_tra3, time_tra4, time_tra5};

rho_cells = {};

CalibCD = ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_cal, smooth_q_cal, false, rho_cells, false,smooth_dq_cells,u_cells);

%% To visualize how the filter works, below are two examples
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % AsymGaussian Filter Results 
% dat = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-19_122338.csv';
% [time_asym, q_asym, dq_asym, ddq_asym, smooth_q_asym, smooth_dq_asym, smooth_ddq_asym, ~, ~] = ezloopdata_compare(dat, false, false, true, 0);
% 
% % Butterworth Filter Results
% dat = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-19_123309.csv';
% [time_butter, q_butter, dq_butter, ddq_butter, smooth_q_butter, smooth_dq_butter, smooth_ddq_butter, ~, ~] = ezloopdata_compare(dat, false, false, true, 0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Dynamics Controller response to Teach-and-Repeat 4
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Teach-and-Repeat 4 trajectory
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4.csv'; 
[time_TnR4, q_TnR4, dq_TnR4, ddq_TnR4, smooth_q_TnR4,smooth_dq_TnR4,smooth_ddq_TnR4, index_TnR4, ~,~] = ezloopdata_compare(dat, false, false, false, 0);

% Inverse Dynamics Controller on TnR4 w/o forceplate
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CD1.csv'; 
[time_CD1, q_CD1, dq_CD1, ddq_CD1, smooth_q_CD1, smooth_dq_CD1, smooth_ddq_CD1, ~, ~,cont_CD1] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CD2.csv'; 
[time_CD2, q_CD2, dq_CD2, ddq_CD2, smooth_q_CD2, smooth_dq_CD2, smooth_ddq_CD2, ~, ~,cont_CD2] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CD3.csv'; 
[time_CD3, q_CD3, dq_CD3, ddq_CD3, smooth_q_CD3, smooth_dq_CD3, smooth_ddq_CD3, ~, ~,cont_CD3] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CD4.csv'; 
[time_CD4, q_CD4, dq_CD4, ddq_CD4, smooth_q_CD4, smooth_dq_CD4, smooth_ddq_CD4, ~, ~,cont_CD4] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CD5.csv'; 
[time_CD5, q_CD5, dq_CD5, ddq_CD5, smooth_q_CD5, smooth_dq_CD5, smooth_ddq_CD5, ~, ~,cont_CD5] = ezloopdata_compare(dat, false, false, false, index_TnR4);

smooth_q_cells = {smooth_q_CD1, smooth_q_CD2, smooth_q_CD3, ...
                  smooth_q_CD4, smooth_q_CD5};

smooth_dq_cells = {smooth_dq_CD1, smooth_dq_CD2, smooth_dq_CD3, ...
                  smooth_dq_CD4, smooth_dq_CD5};

u_cells = {cont_CD1(:,5:6), cont_CD2(:,5:6), cont_CD3(:,5:6), ...
                  cont_CD4(:,5:6), cont_CD5(:,5:6)};

time_cells = {time_CD1, time_CD2, time_CD3, time_CD4, time_CD5};

rho_cells = {};

TnR = ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_TnR4, smooth_q_TnR4, false, rho_cells, false,smooth_dq_cells,u_cells);

% Inverse Dynamics Controller on TnR4 w/ forceplate
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_CD4FP1.csv'; 
[time_CDFP1, q_CDFP1, dq_CDFP1, ddq_CDFP1, smooth_q_CDFP1, smooth_dq_CDFP1, smooth_ddq_CDFP1, ~, rho0_CDFP1,cont_CDFP1] = ezloopdata_compare(dat, false, false, false, index_TnR4);
 
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_CD4FP2.csv'; 
[time_CDFP2, q_CDFP2, dq_CDFP2, ddq_CDFP2, smooth_q_CDFP2, smooth_dq_CDFP2, smooth_ddq_CDFP2, ~, rho0_CDFP2,cont_CDFP2] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_CD4FP3.csv'; 
[time_CDFP3, q_CDFP3, dq_CDFP3, ddq_CDFP3, smooth_q_CDFP3, smooth_dq_CDFP3, smooth_ddq_CDFP3, ~, rho0_CDFP3,cont_CDFP3] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_CD4FP4.csv'; 
[time_CDFP4, q_CDFP4, dq_CDFP4, ddq_CDFP4, smooth_q_CDFP4, smooth_dq_CDFP4, smooth_ddq_CDFP4, ~, rho0_CDFP4,cont_CDFP4] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_CD4FP5.csv'; 
[time_CDFP5, q_CDFP5, dq_CDFP5, ddq_CDFP5, smooth_q_CDFP5, smooth_dq_CDFP5, smooth_ddq_CDFP5, ~, rho0_CDFP5,cont_CDFP5] = ezloopdata_compare(dat, false, false, false, index_TnR4);

smooth_q_cells = {smooth_q_CDFP1, smooth_q_CDFP2, smooth_q_CDFP3, ...
                  smooth_q_CDFP4, smooth_q_CDFP5};

smooth_dq_cells = {smooth_dq_CDFP1, smooth_dq_CDFP2, smooth_dq_CDFP3, ...
                  smooth_dq_CDFP4, smooth_dq_CDFP5};

u_cells = {cont_CDFP1(:,5:6), cont_CDFP2(:,5:6), cont_CDFP3(:,5:6), ...
                  cont_CDFP4(:,5:6), cont_CDFP5(:,5:6)};

rho_cells = {rho0_CDFP1, rho0_CDFP2, rho0_CDFP3, rho0_CDFP4, rho0_CDFP5};

time_cells = {time_CDFP1, time_CDFP2, time_CDFP3, time_CDFP4, time_CDFP5};

TnRCDFP = ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_TnR4, smooth_q_TnR4, true, rho_cells, true,smooth_dq_cells,u_cells);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% CBF Controller Response for High Conservative Case for Teach and Repeat 4

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CBFH1.csv'; 
[time_H1, q_H1, dq_H1, ddq_H1, smooth_q_H1, smooth_dq_H1, smooth_ddq_H1, ~, rho0_H1,cont_H1] = ezloopdata_compare(dat, false, false, false, index_TnR4);
 
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CBFH2.csv'; 
[time_H2, q_H2, dq_H2, ddq_H2, smooth_q_H2, smooth_dq_H2, smooth_ddq_H2, ~, rho0_H2,cont_H2] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CBFH3.csv'; 
[time_H3, q_H3, dq_H3, ddq_H3, smooth_q_H3, smooth_dq_H3, smooth_ddq_H3, ~, rho0_H3,cont_H3] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CBFH4.csv'; 
[time_H4, q_H4, dq_H4, ddq_H4, smooth_q_H4, smooth_dq_H4, smooth_ddq_H4, ~, rho0_H4,cont_H4] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CBFH5.csv'; 
[time_H5, q_H5, dq_H5, ddq_H5, smooth_q_H5, smooth_dq_H5, smooth_ddq_H5, ~, rho0_H5,cont_H5] = ezloopdata_compare(dat, false, false, false, index_TnR4);

smooth_q_cells = {smooth_q_H1, smooth_q_H2, smooth_q_H3, ...
                  smooth_q_H4, smooth_q_H5};

smooth_dq_cells = {smooth_dq_H1, smooth_dq_H2, smooth_dq_H3, ...
                  smooth_dq_H4, smooth_dq_H5};

u_cells = {cont_H1(:,5:6), cont_H2(:,5:6), cont_H3(:,5:6), ...
                  cont_H4(:,5:6), cont_H5(:,5:6)};

rho_cells = {rho0_H1, rho0_H2, rho0_H3, rho0_H4, rho0_H5};

time_cells = {time_H1, time_H2, time_H3, time_H4, time_H5};

TnRHigh = ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_TnR4, smooth_q_TnR4, true, rho_cells, true,smooth_dq_cells,u_cells);

%% Example Plotting code used for Robosoft poster
%%%%%%%%%%%%%%%%%%%

sim_calib_data = load('callibration_data.mat');
sim_time = sim_calib_data.export_data(:,1);
sim_q1 = sim_calib_data.export_data(:,2);
sim_q2 = sim_calib_data.export_data(:,3);
sim_q1_inter = sim_calib_data.export_data(:,4);
sim_q2_inter = sim_calib_data.export_data(:,5);

sim_q1 = deg2rad(sim_q1);
sim_q2 = deg2rad(sim_q2);

sim_q1_inter = deg2rad(sim_q1_inter);
sim_q2_inter = deg2rad(sim_q2_inter);

axisFont   = 10;
labelFont  = 12;
legendFont = 10;

picturewidth_singlecolumn = 8.9;   % cm, typical single-column width
hw_ratio = 1.4;                    % height / width for 6 stacked plots

sim_plot = figure('Units','centimeters', ...
    'Position',[2.3 1 picturewidth_singlecolumn hw_ratio*picturewidth_singlecolumn]);

t = tiledlayout(sim_plot,2,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');

xLimits = [0 sim_time(end)];

% Plot 1
ax1 = nexttile;
p_r1 = plot(sim_time, smooth_q_cal(2:end,1), '-.', 'Color', [0.2 0.2 0.2], 'LineWidth', 2); hold on
p1 = plot(sim_time, sim_q1_inter, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);

ylabel('$q_1$ (rad)','FontSize',labelFont)
grid on
hold off

lgd = legend([p1, p_r1], {'Sim.','Ref.'}, 'Numcolumns',2,'Location','northeast','FontSize',legendFont);
xlim([0 sim_time(end)])
grid on;
hold off

% Plot 2
ax2 = nexttile;
p_r1 = plot(sim_time, smooth_q_cal(2:end,2), '-.', 'Color', [0.2 0.2 0.2], 'LineWidth', 2); hold on
p1 = plot(sim_time, sim_q2_inter, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);

xlabel('Time (s)','FontSize',12)
ylabel('$q_2$ (rad)','FontSize',12)
grid on;
hold off

set([ax1 ax2], ...
    'FontSize',axisFont, ...
    'XLim',xLimits, ...
    'Box','off', ...
    'TickLabelInterpreter','latex');

ax1.XTickLabel = [];

linkaxes([ax1 ax2],'x')

set(findall(sim_plot,'-property','Interpreter'), 'Interpreter', 'latex')
set(findall(sim_plot,'-property','TickLabelInterpreter'), 'TickLabelInterpreter', 'latex')
set(findall(sim_plot,'-property','Box'), 'Box', 'off')

fname = 'fin_plots/SimTuning';
exportgraphics(sim_plot, strcat(fname, '.pdf'), 'ContentType', 'vector');

%% FIGURES for Paper 

single_column = true;
picturewidth_singlecolumn = 8.6 ; % cm

Fmax = 11.16 * 1.6 / 100;  

out1 = TnRCDFP;

    % Back-calculate raw force for every run
    out1.raw.force_all = Fmax * (1 - out1.raw.rho_all);

    % Best way: compute force statistics from the raw transformed data
    out1.force.mu    = mean(out1.raw.force_all, 2);
    out1.force.sigma = std(out1.raw.force_all, 0, 2);
    out1.force.upper = out1.force.mu + 2*out1.force.sigma;
    out1.force.lower = out1.force.mu - 2*out1.force.sigma;

    % Force fill data
    out1.force.fillX = [out1.time; flipud(out1.time)];
    out1.force.fillY = [out1.force.upper; flipud(out1.force.lower)];
    out1.force.fillColor = [0.850 0.325 0.098];
    out1.force.fillAlpha = 0.2;

out2 = TnRHigh;

    % Back-calculate raw force for every run
    out2.raw.force_all = Fmax * (1 - out2.raw.rho_all);

    % Best way: compute force statistics from the raw transformed data
    out2.force.mu    = mean(out2.raw.force_all, 2);
    out2.force.sigma = std(out2.raw.force_all, 0, 2);
    out2.force.upper = out2.force.mu + 2*out2.force.sigma;
    out2.force.lower = out2.force.mu - 2*out2.force.sigma;

    % Force fill data
    out2.force.fillX = [out2.time; flipud(out2.time)];
    out2.force.fillY = [out2.force.upper; flipud(out2.force.lower)];
    out2.force.fillColor = [0.850 0.325 0.098];
    out2.force.fillAlpha = 0.2;

F_max = Fmax*ones(length(out1.time),1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% rho plot
rho_plot = figure;
fill(out1.rho.fillX, out1.rho.fillY, [0 0.447 0.741], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.15);
hold on

fill(out2.rho.fillX, out2.rho.fillY, [0.850 0.325 0.098], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.15);

p1 = plot(out1.rho.time, out1.rho.mu, 'Color', [0 0.447 0.741], 'LineWidth', 1.5);
p2 = plot(out2.rho.time, out2.rho.mu, 'Color', [0.850 0.325 0.098], 'LineWidth', 1.5);
p3 = plot(out1.rho.time, out1.rho.max, '--r', 'LineWidth', 1.5);

ylabel('Rho','FontSize',15);xlabel('Time ($s$)','FontSize',15)
lgd = legend([p1,p2,p3],{'CD','CBF','Max Rho'},'Location','best','FontSize',12);
ax = gca;
ax.FontSize = 14;

lgd.Units = 'normalized';       % Make position relative to axes
pos1 = lgd.Position;
pos1(1) = pos1(1) - 0.26;         % Move right
pos1(2) = pos1(2) + 0.3;         % Move up
lgd.Position = pos1;

hold off

hw_ratio = 1.1;

set(findall(rho_plot,'-property','Interpreter'), 'Interpreter', 'latex')
set(findall(rho_plot,'-property','TickLabelInterpreter'), 'TickLabelInterpreter', 'latex')
set(findall(rho_plot,'-property','Box'), 'Box', 'off')
set(rho_plot, 'Units', 'centimeters', ...
        'Position', [2.3 1 picturewidth_singlecolumn hw_ratio * picturewidth_singlecolumn]);
grid on;

fname = 'fin_plots/Rho_Comparison';
exportgraphics(rho_plot, strcat(fname, '.pdf'), 'ContentType', 'vector');

%%%%%%%%%%%%%% force plot

axisFont   = 10;
labelFont  = 12;
legendFont = 10;

picturewidth_singlecolumn = 8.9;   % cm, typical single-column width
hw_ratio = 1.4;                    % height / width for 6 stacked plots

force_plot = figure('Units','centimeters', ...
    'Position',[2.3 1 picturewidth_singlecolumn hw_ratio*picturewidth_singlecolumn]);

t = tiledlayout(force_plot,1,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');

xLimits = [0 out1.time(end)];

ax1 = nexttile;
fill(out1.force.fillX, out1.force.fillY, [0 0.447 0.741], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.15);
hold on

fill(out2.force.fillX, out2.force.fillY, [0.850 0.325 0.098], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.15);

p1 = plot(out1.time, out1.force.mu, 'Color', [0 0.447 0.741], 'LineWidth', 1.5);
p2 = plot(out2.time, out2.force.mu, 'Color', [0.850 0.325 0.098], 'LineWidth', 1.5);
p3 = plot(out1.time, F_max, '--r', 'LineWidth', 1.5);

ylabel('Force ($N$)','FontSize',15);xlabel('Time ($s$)','FontSize',labelFont)
lgd = legend([p1,p2,p3], {'CD','CBF','Max Force'}, 'Numcolumns',2,'Location','northeast','FontSize',legendFont,'Box','off');
grid on;
hold off

set(ax1, ...
    'FontSize',axisFont, ...
    'XLim',xLimits, ...
    'Box','off', ...
    'TickLabelInterpreter','latex');

set(findall(force_plot,'-property','Interpreter'), ...
    'Interpreter','latex')

fname = 'fin_plots/Force_Comparison';
exportgraphics(force_plot, strcat(fname, '.pdf'), 'ContentType', 'vector');

%% force plot, just CD

axisFont   = 10;
labelFont  = 12;
legendFont = 10;

picturewidth_singlecolumn = 8.9;   % cm, typical single-column width
hw_ratio = 1.4;                    % height / width for 6 stacked plots

forceCD_plot = figure('Units','centimeters', ...
    'Position',[2.3 1 picturewidth_singlecolumn hw_ratio*picturewidth_singlecolumn]);

t = tiledlayout(forceCD_plot,1,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');

xLimits = [0 out1.time(end)];

% Plot 1
ax1 = nexttile;
fill(out1.force.fillX, out1.force.fillY, [0 0.447 0.741], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.15);
hold on

p1 = plot(out1.time, out1.force.mu, 'Color', [0 0.447 0.741], 'LineWidth', 1.5);
p3 = plot(out1.time, F_max, '--r', 'LineWidth', 1.5);

ylabel('Force ($N$)','FontSize',15);xlabel('Time ($s$)','FontSize',labelFont)
lgd = legend([p1,p3], {'CD','Max Force'}, 'Numcolumns',2,'Location','northeast','FontSize',legendFont,'Box','off');
grid on;
hold off

set(ax1, ...
    'FontSize',axisFont, ...
    'XLim',xLimits, ...
    'Box','off', ...
    'TickLabelInterpreter','latex');

set(findall(forceCD_plot,'-property','Interpreter'), ...
    'Interpreter','latex')

fname = 'fin_plots/ForceCD_Comparison';
exportgraphics(forceCD_plot, strcat(fname, '.pdf'), 'ContentType', 'vector');

%% force plot, just CBF

forceCBF_plot = figure;
hold on

fill(out2.force.fillX, out2.force.fillY, [0.850 0.325 0.098], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.15);

p2 = plot(out2.time, out2.force.mu, 'Color', [0.850 0.325 0.098], 'LineWidth', 1.5);
p3 = plot(out1.time, F_max, '--r', 'LineWidth', 1.5);

ylabel('Force ($N$)','FontSize',15);xlabel('Time ($s$)','FontSize',15)
lgd = legend([p2,p3],{'CBF','Max Force'},'Location','best','FontSize',12);
ax = gca;
ax.FontSize = 14;

hold off

hw_ratio = 1.1;

set(findall(forceCBF_plot,'-property','Interpreter'), 'Interpreter', 'latex')
set(findall(forceCBF_plot,'-property','TickLabelInterpreter'), 'TickLabelInterpreter', 'latex')
set(findall(forceCBF_plot,'-property','Box'), 'Box', 'off')
set(forceCBF_plot, 'Units', 'centimeters', ...
        'Position', [2.3 1 picturewidth_singlecolumn hw_ratio * picturewidth_singlecolumn]);
grid on;

fname = 'fin_plots/ForceCBF_Comparison';
exportgraphics(forceCBF_plot, strcat(fname, '.png'), 'ContentType', 'vector');

%% comparison of q0 and q1 for CD vs CBF

axisFont   = 10;
labelFont  = 12;
legendFont = 10;

picturewidth_singlecolumn = 8.9;   % cm, typical single-column width
hw_ratio = 1.4;                    % height / width for 6 stacked plots

traj_plot = figure('Units','centimeters', ...
    'Position',[2.3 1 picturewidth_singlecolumn hw_ratio*picturewidth_singlecolumn]);

t = tiledlayout(traj_plot,8,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');

xLimits = [0 out1.time_ref(end)];
blue = [0 0.4470 0.7410];
gray = [0.2 0.2 0.2];

% Plot 1: q1
ax1 = nexttile([2 1]);
fill(out1.q0.fillX, out1.q0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
fill(out2.q0.fillX, out2.q0.fillY, [0.8500 0.3250 0.0980], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
p_r1 = plot(out1.time_ref, out1.reference.q(:,1), '-.', ...
    'Color',gray,'LineWidth',1.6);
p1 = plot(out1.time, out1.q0.mu, ...
    'Color',blue,'LineWidth',1.4);
p2 = plot(out2.time, out2.q0.mu, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);

ylabel('$q_1$ (rad)','FontSize',labelFont)
lgd = legend([p1,p2,p_r1], {'CD','CBF','Ref.'}, 'NumColumns',2, ...
    'Location','northwest','FontSize',legendFont,'Box','off');

lgd.Position = lgd.Position + [-0.01 0 0 0.04];

grid on
hold off

% Plot 2: q2
ax2 = nexttile([2 1]);
fill(out1.q1.fillX, out1.q1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
fill(out2.q1.fillX, out2.q1.fillY, [0.8500 0.3250 0.0980], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
p_r1 = plot(out1.time_ref, out1.reference.q(:,2), '-.', ...
    'Color',gray,'LineWidth',1.6);
p1 = plot(out1.time, out1.q1.mu, ...
    'Color',blue,'LineWidth',1.4);
p2 = plot(out2.time, out2.q1.mu, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);

ylabel('$q_2$ (rad)','FontSize',labelFont)

grid on
hold off

% Plot 3: dq1
ax3 = nexttile;
fill(out1.dq0.fillX, out1.dq0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
fill(out2.dq0.fillX, out2.dq0.fillY, [0.8500 0.3250 0.0980], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
plot(out1.time, out1.dq0.mu, ...
    'Color',blue,'LineWidth',1.2);
plot(out2.time, out2.dq0.mu, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);

ylabel('$\dot{q}_1$','FontSize',labelFont)
grid on
hold off

% Plot 4: dq2
ax4 = nexttile;
fill(out1.dq1.fillX, out1.dq1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
fill(out2.dq1.fillX, out2.dq1.fillY, [0.8500 0.3250 0.0980], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
plot(out1.time, out1.dq1.mu, ...
    'Color',blue,'LineWidth',1.2);
plot(out2.time, out2.dq1.mu, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);
ylabel('$\dot{q}_2$','FontSize',labelFont)
grid on
hold off

% Plot 5: u1
ax5 = nexttile;
fill(out1.u0.fillX, out1.u0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
fill(out2.u0.fillX, out2.u0.fillY, [0.8500 0.3250 0.0980], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
plot(out1.time, out1.u0.mu, ...
    'Color',blue,'LineWidth',1.2);
plot(out2.time, out2.u0.mu, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);

ylabel('$u_1$','FontSize',labelFont)
grid on
hold off

% Plot 6: u2
ax6 = nexttile;
fill(out1.u1.fillX, out1.u1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
fill(out2.u1.fillX, out2.u1.fillY, [0.8500 0.3250 0.0980], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
plot(out1.time, out1.u1.mu, ...
    'Color',blue,'LineWidth',1.2);
plot(out2.time, out2.u1.mu, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);

xlabel('Time (s)','FontSize',labelFont)
ylabel('$u_2$','FontSize',labelFont)
grid on
hold off

set([ax1 ax2 ax3 ax4 ax5 ax6], ...
    'FontSize',axisFont, ...
    'XLim',xLimits, ...
    'Box','off', ...
    'TickLabelInterpreter','latex');

ax1.XTickLabel = [];
ax1.YTick = [-0.5 0 0.9];
ax2.XTickLabel = [];
ax3.XTickLabel = [];
ax4.XTickLabel = [];
ax4.YTick = [-0.4 0 0.6];
ax5.XTickLabel = [];
ax5.YLim = [-120 140];
ax5.YTick = [-120 0 140];
ax6.YTick = [-120 0 200];

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6],'x')

set(findall(traj_plot,'-property','Interpreter'), ...
    'Interpreter','latex')

fname = 'fin_plots/TrajectoryTracking_Comparison';
exportgraphics(traj_plot, strcat(fname, '.pdf'), 'ContentType', 'vector');

%% Tracking CD

axisFont   = 10;
labelFont  = 12;
legendFont = 10;

picturewidth_singlecolumn = 8.9;   % cm, typical single-column width
hw_ratio = 1.4;                    % height / width for 6 stacked plots

trajCD_plot = figure('Units','centimeters', ...
    'Position',[2.3 1 picturewidth_singlecolumn hw_ratio*picturewidth_singlecolumn]);

t = tiledlayout(trajCD_plot,8,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');

xLimits = [0 out1.time_ref(end)];
blue = [0 0.4470 0.7410];
gray = [0.2 0.2 0.2];

% Plot 1: q1
ax1 = nexttile([2 1]);
fill(out1.q0.fillX, out1.q0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
p_r1 = plot(out1.time_ref, out1.reference.q(:,1), '-.', ...
    'Color',gray,'LineWidth',1.6);
p1 = plot(out1.time, out1.q0.mu, ...
    'Color',blue,'LineWidth',1.4);

ylabel('$q_1$ (rad)','FontSize',labelFont)
lgd = legend([p1, p_r1], {'CD','Ref.'}, 'NumColumns',2, ...
    'Location','northwest','FontSize',legendFont,'Box','off');

lgd.Position = lgd.Position + [-0.01 0 0 0.04];

grid on
hold off

% Plot 2: q2
ax2 = nexttile([2 1]);
fill(out1.q1.fillX, out1.q1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(out1.time_ref, out1.reference.q(:,2), '-.', ...
    'Color',gray,'LineWidth',1.6);
plot(out1.time, out1.q1.mu, ...
    'Color',blue,'LineWidth',1.4);

ylabel('$q_2$ (rad)','FontSize',labelFont)
grid on
hold off

% Plot 3: dq1
ax3 = nexttile;
fill(out1.dq0.fillX, out1.dq0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(out1.time, out1.dq0.mu, ...
    'Color',blue,'LineWidth',1.2);

ylabel('$\dot{q}_1$','FontSize',labelFont)
grid on
hold off

% Plot 4: dq2
ax4 = nexttile;
fill(out1.dq1.fillX, out1.dq1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(out1.time, out1.dq1.mu, ...
    'Color',blue,'LineWidth',1.2);

ylabel('$\dot{q}_2$','FontSize',labelFont)
grid on
hold off

% Plot 5: u1
ax5 = nexttile;
fill(out1.u0.fillX, out1.u0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(out1.time, out1.u0.mu, ...
    'Color',blue,'LineWidth',1.2);

ylabel('$u_1$','FontSize',labelFont)
grid on
hold off

% Plot 6: u2
ax6 = nexttile;
fill(out1.u1.fillX, out1.u1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(out1.time, out1.u1.mu, ...
    'Color',blue,'LineWidth',1.2);

xlabel('Time (s)','FontSize',labelFont)
ylabel('$u_2$','FontSize',labelFont)
grid on
hold off

set([ax1 ax2 ax3 ax4 ax5 ax6], ...
    'FontSize',axisFont, ...
    'XLim',xLimits, ...
    'Box','off', ...
    'TickLabelInterpreter','latex');

ax1.XTickLabel = [];
ax1.YTick = [-0.5 0 0.9];
ax2.XTickLabel = [];
ax3.XTickLabel = [];
ax3.YTick = [-0.11 0 0.12];
ax4.XTickLabel = [];
ax4.YTick = [-0.3 0 0.4];
ax5.XTickLabel = [];
ax6.YTick = [-120 0 200];

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6],'x')

set(findall(trajCD_plot,'-property','Interpreter'), ...
    'Interpreter','latex')

fname = 'fin_plots/CDTrackingFP';
exportgraphics(trajCD_plot, strcat(fname, '.pdf'), 'ContentType', 'vector');

%% PD tuning results

axisFont   = 10;
labelFont  = 12;
legendFont = 10;

picturewidth_singlecolumn = 8.9;   % cm, typical single-column width
hw_ratio = 1.4;                    % height / width for 6 stacked plots

pdtuning_plot = figure('Units','centimeters', ...
    'Position',[2.3 1 picturewidth_singlecolumn hw_ratio*picturewidth_singlecolumn]);

t = tiledlayout(pdtuning_plot,8,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');

xLimits = [0 TunePD.time_ref(end)];
blue = [0 0.4470 0.7410];
gray = [0.2 0.2 0.2];

% Plot 1: q1
ax1 = nexttile([2 1]);
fill(TunePD.q0.fillX, TunePD.q0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
p_r1 = plot(TunePD.time_ref, TunePD.reference.q(:,1), '-.', ...
    'Color',gray,'LineWidth',1.6);
p1 = plot(TunePD.time, TunePD.q0.mu, ...
    'Color',blue,'LineWidth',1.4);

ylabel('$q_1$ (rad)','FontSize',labelFont)
lgd = legend([p1, p_r1], {'PD','Ref.'}, 'NumColumns',2, ...
    'Location','northeast','FontSize',legendFont,'Box','off');

lgd.Position = lgd.Position + [0.04 0 0 0.04];

grid on
hold off

% Plot 2: q2
ax2 = nexttile([2 1]);
fill(TunePD.q1.fillX, TunePD.q1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(TunePD.time_ref, TunePD.reference.q(:,2), '-.', ...
    'Color',gray,'LineWidth',1.6);
plot(TunePD.time, TunePD.q1.mu, ...
    'Color',blue,'LineWidth',1.4);

ylabel('$q_2$ (rad)','FontSize',labelFont)
grid on
hold off

% Plot 3: dq1
ax3 = nexttile;
fill(TunePD.dq0.fillX, TunePD.dq0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(TunePD.time, TunePD.dq0.mu, ...
    'Color',blue,'LineWidth',1.2);

ylabel('$\dot{q}_1$','FontSize',labelFont)
grid on
hold off

% Plot 4: dq2
ax4 = nexttile;
fill(TunePD.dq1.fillX, TunePD.dq1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(TunePD.time, TunePD.dq1.mu, ...
    'Color',blue,'LineWidth',1.2);

ylabel('$\dot{q}_2$','FontSize',labelFont)
grid on
hold off

% Plot 5: u1
ax5 = nexttile;
fill(TunePD.u0.fillX, TunePD.u0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(TunePD.time, TunePD.u0.mu, ...
    'Color',blue,'LineWidth',1.2);

ylabel('$u_1$','FontSize',labelFont)
grid on
hold off

% Plot 6: u2
ax6 = nexttile;
fill(TunePD.u1.fillX, TunePD.u1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(TunePD.time, TunePD.u1.mu, ...
    'Color',blue,'LineWidth',1.2);

xlabel('Time (s)','FontSize',labelFont)
ylabel('$u_2$','FontSize',labelFont)
grid on
hold off

set([ax1 ax2 ax3 ax4 ax5 ax6], ...
    'FontSize',axisFont, ...
    'XLim',xLimits, ...
    'Box','off', ...
    'TickLabelInterpreter','latex');

ax1.XTickLabel = [];
ax2.XTickLabel = [];
ax3.XTickLabel = [];
ax4.XTickLabel = [];
ax5.XTickLabel = [];

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6],'x')

set(findall(pdtuning_plot,'-property','Interpreter'), ...
    'Interpreter','latex')

fname = 'fin_plots/PDTuning';
exportgraphics(pdtuning_plot, strcat(fname, '.pdf'), ...
    'ContentType','vector');

%% PD tracking results

pdtracking_plot = figure('Units','centimeters', ...
    'Position',[2.3 1 picturewidth_singlecolumn hw_ratio*picturewidth_singlecolumn]);

t = tiledlayout(pdtracking_plot,8,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');

xLimits = [0 CalibPD.time_ref(end)];
blue = [0 0.4470 0.7410];
gray = [0.2 0.2 0.2];

% Plot 1: q1
ax1 = nexttile([2 1]);
fill(CalibPD.q0.fillX, CalibPD.q0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
p_r1 = plot(CalibPD.time_ref, CalibPD.reference.q(:,1), '-.', ...
    'Color',gray,'LineWidth',1.6);
p1 = plot(CalibPD.time, CalibPD.q0.mu, ...
    'Color',blue,'LineWidth',1.4);

ylabel('$q_1$ (rad)','FontSize',labelFont)
lgd = legend([p1, p_r1], {'PD','Ref.'}, 'NumColumns',2, ...
    'Location','northeast','FontSize',legendFont,'Box','off');

lgd.Position = lgd.Position + [0.04 0 0 0.04];

grid on
hold off

% Plot 2: q2
ax2 = nexttile([2 1]);
fill(CalibPD.q1.fillX, CalibPD.q1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(CalibPD.time_ref, CalibPD.reference.q(:,2), '-.', ...
    'Color',gray,'LineWidth',1.6);
plot(CalibPD.time, CalibPD.q1.mu, ...
    'Color',blue,'LineWidth',1.4);

ylabel('$q_2$ (rad)','FontSize',labelFont)
grid on
hold off

% Plot 3: dq1
ax3 = nexttile;
fill(CalibPD.dq0.fillX, CalibPD.dq0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(CalibPD.time, CalibPD.dq0.mu, ...
    'Color',blue,'LineWidth',1.2);

ylabel('$\dot{q}_1$','FontSize',labelFont)
grid on
hold off

% Plot 4: dq2
ax4 = nexttile;
fill(CalibPD.dq1.fillX, CalibPD.dq1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(CalibPD.time, CalibPD.dq1.mu, ...
    'Color',blue,'LineWidth',1.2);

ylabel('$\dot{q}_2$','FontSize',labelFont)
grid on
hold off

% Plot 5: u1
ax5 = nexttile;
fill(CalibPD.u0.fillX, CalibPD.u0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(CalibPD.time, CalibPD.u0.mu, ...
    'Color',blue,'LineWidth',1.2);

ylabel('$u_1$','FontSize',labelFont)
grid on
hold off

% Plot 6: u2
ax6 = nexttile;
fill(CalibPD.u1.fillX, CalibPD.u1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(CalibPD.time, CalibPD.u1.mu, ...
    'Color',blue,'LineWidth',1.2);

xlabel('Time (s)','FontSize',labelFont)
ylabel('$u_2$','FontSize',labelFont)
grid on
hold off

set([ax1 ax2 ax3 ax4 ax5 ax6], ...
    'FontSize',axisFont, ...
    'XLim',xLimits, ...
    'Box','off', ...
    'TickLabelInterpreter','latex');

ax1.XTickLabel = [];
ax2.XTickLabel = [];
ax3.XTickLabel = [];
ax4.XTickLabel = [];
ax5.XTickLabel = [];
ax5.YTick = [-60 0 50];

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6],'x')

set(findall(pdtracking_plot,'-property','Interpreter'), ...
    'Interpreter','latex')

fname = 'fin_plots/PDTracking';
exportgraphics(pdtracking_plot, strcat(fname, '.pdf'), 'ContentType', 'vector');

%% CD Tracking results 

cdtracking_plot = figure('Units','centimeters', ...
    'Position',[2.3 1 picturewidth_singlecolumn hw_ratio*picturewidth_singlecolumn]);

t = tiledlayout(cdtracking_plot,8,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');

xLimits = [0 CalibCD.time_ref(end)];
blue = [0 0.4470 0.7410];
gray = [0.2 0.2 0.2];

% Plot 1: q1
ax1 = nexttile([2 1]);
fill(CalibCD.q0.fillX, CalibCD.q0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
p_r1 = plot(CalibCD.time_ref, CalibCD.reference.q(:,1), '-.', ...
    'Color',gray,'LineWidth',1.6);
p1 = plot(CalibCD.time, CalibCD.q0.mu, ...
    'Color',blue,'LineWidth',1.4);

ylabel('$q_1$ (rad)','FontSize',labelFont)
lgd = legend([p1, p_r1], {'CD','Ref.'}, 'NumColumns',2, ...
    'Location','northeast','FontSize',legendFont,'Box','off');

lgd.Position = lgd.Position + [0.04 0 0 0.04];

grid on
hold off

% Plot 2: q2
ax2 = nexttile([2 1]);
fill(CalibCD.q1.fillX, CalibCD.q1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(CalibCD.time_ref, CalibCD.reference.q(:,2), '-.', ...
    'Color',gray,'LineWidth',1.6);
plot(CalibCD.time, CalibCD.q1.mu, ...
    'Color',blue,'LineWidth',1.4);

ylabel('$q_2$ (rad)','FontSize',labelFont)
grid on
hold off

% Plot 3: dq1
ax3 = nexttile;
fill(CalibCD.dq0.fillX, CalibCD.dq0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(CalibCD.time, CalibCD.dq0.mu, ...
    'Color',blue,'LineWidth',1.2);

ylabel('$\dot{q}_1$','FontSize',labelFont)
grid on
hold off

% Plot 4: dq2
ax4 = nexttile;
fill(CalibCD.dq1.fillX, CalibCD.dq1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(CalibCD.time, CalibCD.dq1.mu, ...
    'Color',blue,'LineWidth',1.2);

ylabel('$\dot{q}_2$','FontSize',labelFont)
grid on
hold off

% Plot 5: u1
ax5 = nexttile;
fill(CalibCD.u0.fillX, CalibCD.u0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(CalibCD.time, CalibCD.u0.mu, ...
    'Color',blue,'LineWidth',1.2);

ylabel('$u_1$','FontSize',labelFont)
grid on
hold off

% Plot 6: u2
ax6 = nexttile;
fill(CalibCD.u1.fillX, CalibCD.u1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(CalibCD.time, CalibCD.u1.mu, ...
    'Color',blue,'LineWidth',1.2);

xlabel('Time (s)','FontSize',labelFont)
ylabel('$u_2$','FontSize',labelFont)
grid on
hold off

set([ax1 ax2 ax3 ax4 ax5 ax6], ...
    'FontSize',axisFont, ...
    'XLim',xLimits, ...
    'Box','off', ...
    'TickLabelInterpreter','latex');

ax1.XTickLabel = [];
ax2.XTickLabel = [];
ax3.XTickLabel = [];
ax4.XTickLabel = [];
ax4.YTick = [-0.05 0 0.03];
ax5.XTickLabel = [];
ax6.YTick = [-100 0 80];

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6],'x')

set(findall(cdtracking_plot,'-property','Interpreter'), ...
    'Interpreter','latex')

fname = 'fin_plots/CDTracking';
exportgraphics(cdtracking_plot, strcat(fname, '.pdf'), 'ContentType', 'vector');

%% CD tracking of TnR

cdtnr_plot = figure('Units','centimeters', ...
    'Position',[2.3 1 picturewidth_singlecolumn hw_ratio*picturewidth_singlecolumn]);

t = tiledlayout(cdtnr_plot,8,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');

xLimits = [0 TnR.time_ref(end)];
blue = [0 0.4470 0.7410];
gray = [0.2 0.2 0.2];

% Plot 1: q1
ax1 = nexttile([2 1]);
fill(TnR.q0.fillX, TnR.q0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
p_r1 = plot(TnR.time_ref, TnR.reference.q(:,1), '-.', ...
    'Color',gray,'LineWidth',1.6);
p1 = plot(TnR.time, TnR.q0.mu, ...
    'Color',blue,'LineWidth',1.4);

ylabel('$q_1$ (rad)','FontSize',labelFont)
lgd = legend([p1, p_r1], {'CD','Ref.'}, 'NumColumns',2, ...
    'Location','northwest','FontSize',legendFont,'Box','off');

lgd.Position = lgd.Position + [-0.01 0 0 0.04];

grid on
hold off

% Plot 2: q2
ax2 = nexttile([2 1]);
fill(TnR.q1.fillX, TnR.q1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(TnR.time_ref, TnR.reference.q(:,2), '-.', ...
    'Color',gray,'LineWidth',1.6);
plot(TnR.time, TnR.q1.mu, ...
    'Color',blue,'LineWidth',1.4);

ylabel('$q_2$ (rad)','FontSize',labelFont)
grid on
hold off

% Plot 3: dq1
ax3 = nexttile;
fill(TnR.dq0.fillX, TnR.dq0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(TnR.time, TnR.dq0.mu, ...
    'Color',blue,'LineWidth',1.2);

ylabel('$\dot{q}_1$','FontSize',labelFont)
grid on
hold off

% Plot 4: dq2
ax4 = nexttile;
fill(TnR.dq1.fillX, TnR.dq1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(TnR.time, TnR.dq1.mu, ...
    'Color',blue,'LineWidth',1.2);

ylabel('$\dot{q}_2$','FontSize',labelFont)
grid on
hold off

% Plot 5: u1
ax5 = nexttile;
fill(TnR.u0.fillX, TnR.u0.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(TnR.time, TnR.u0.mu, ...
    'Color',blue,'LineWidth',1.2);

ylabel('$u_1$','FontSize',labelFont)
grid on
hold off

% Plot 6: u2
ax6 = nexttile;
fill(TnR.u1.fillX, TnR.u1.fillY, blue, ...
    'FaceAlpha',0.2,'EdgeColor','none'); 
hold on
plot(TnR.time, TnR.u1.mu, ...
    'Color',blue,'LineWidth',1.2);

xlabel('Time (s)','FontSize',labelFont)
ylabel('$u_2$','FontSize',labelFont)
grid on
hold off

set([ax1 ax2 ax3 ax4 ax5 ax6], ...
    'FontSize',axisFont, ...
    'XLim',xLimits, ...
    'Box','off', ...
    'TickLabelInterpreter','latex');

ax1.XTickLabel = [];
ax2.XTickLabel = [];
ax3.XTickLabel = [];
ax3.YTick = [-0.15 0 0.25];
ax4.XTickLabel = [];
ax4.YTick = [-0.25 0 0.45];
ax5.XTickLabel = [];
ax5.YTick = [-120 0 200];
ax6.YTick = [-120 0 200];

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6],'x')

set(findall(cdtnr_plot,'-property','Interpreter'), ...
    'Interpreter','latex')

fname = 'fin_plots/CDTnRTracking';
exportgraphics(cdtnr_plot, strcat(fname, '.pdf'), 'ContentType', 'vector');

%% Results from Low level tunning, how close can we follow control input

cont_plot = figure;

frs_limb = [calib_cont((1:length(time_cal)/2),1) ; -1*calib_cont((length(time_cal)/2+1):end,2)];
sec_limb = [calib_cont((1:length(time_cal)/2),3) ; -1*calib_cont((length(time_cal)/2+1):end,4)];

subplot(2,1,1)
p1 = plot(time_cal,frs_limb,'Color', [1 0 0], 'LineWidth', 1.5); hold on 
p3 = plot(time_cal,calib_cont(:,5), '-.', 'Color', [0.2 0.2 0.2], 'LineWidth', 2);

xlabel('Time (s)','FontSize',15)
ylabel('$u_1$ (hPa)','FontSize',15)
lgd = legend([p1, p3], {'$u$ measured','$u$ commanded'}, 'Location','best','FontSize',12);
ax = gca;
ax.FontSize = 14;
xlim([0 time_cal(end)])
grid on;
hold off

subplot(2,1,2)
p1 = plot(time_cal,sec_limb,'Color', [1 0 0], 'LineWidth', 1.5); hold on 
p3 = plot(time_cal,calib_cont(:,6), '-.', 'Color', [0.2 0.2 0.2], 'LineWidth', 2);

xlabel('Time (s)','FontSize',15)
ylabel('$u_2$ (hPa)','FontSize',15)
lgd = legend([p1, p3], {'$u$ measured','$u$ commanded'}, 'Location','best','FontSize',12);
ax = gca;
ax.FontSize = 14;
xlim([0 time_cal(end)])
grid on;
hold off

hw_ratio = 1.1;

set(findall(cont_plot,'-property','Interpreter'), 'Interpreter', 'latex')
set(findall(cont_plot,'-property','TickLabelInterpreter'), 'TickLabelInterpreter', 'latex')
set(findall(cont_plot,'-property','Box'), 'Box', 'off')
set(cont_plot, 'Units', 'centimeters', ...
        'Position', [2.3 1 picturewidth_singlecolumn hw_ratio * picturewidth_singlecolumn]);

fname = 'fin_plots/LowLevelTuningCon';
exportgraphics(cont_plot, strcat(fname, '.png'), 'ContentType', 'vector');