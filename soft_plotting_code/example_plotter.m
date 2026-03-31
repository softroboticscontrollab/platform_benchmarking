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
[time_cal, q_cal, dq_cal, ddq_cal, smooth_q_cal, smooth_dq_cal, smooth_ddq_cal, calib_start,~] = ezloopdata_compare(dat, false, false, true, 0);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_TrackCalib1.csv'; 
[time_tra1, q_tra1, dq_tra1, ddq_tra1, smooth_q_tra1, smooth_dq_tra1, smooth_ddq_tra1,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_TrackCalib2.csv'; 
[time_tra2, q_tra2, dq_tra2, ddq_tra2, smooth_q_tra2, smooth_dq_tra2, smooth_ddq_tra2,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_TrackCalib3.csv'; 
[time_tra3, q_tra3, dq_tra3, ddq_tra3, smooth_q_tra3, smooth_dq_tra3, smooth_ddq_tra3,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_TrackCalib4.csv'; 
[time_tra4, q_tra4, dq_tra4, ddq_tra4, smooth_q_tra4, smooth_dq_tra4, smooth_ddq_tra4,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_TrackCalib5.csv'; 
[time_tra5, q_tra5, dq_tra5, ddq_tra5, smooth_q_tra5, smooth_dq_tra5, smooth_ddq_tra5,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

smooth_q_cells = {smooth_q_tra1, smooth_q_tra2, smooth_q_tra3, ...
                  smooth_q_tra4, smooth_q_tra5};

time_cells = {time_tra1, time_tra2, time_tra3, time_tra4, time_tra5};

rho_cells = {};

CalibCD = ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_cal, smooth_q_cal, false, rho_cells, false);

% PD Attempt at Tracking Calibration 
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PDCal1.csv'; 
[time_PDCal1, q_PDCal1, dq_PDCal1, ddq_PDCal1, smooth_q_PDCal1, smooth_dq_PDCal1, smooth_ddq_PDCal1,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PDCal2.csv'; 
[time_PDCal2, q_PDCal2, dq_PDCal2, ddq_PDCal2, smooth_q_PDCal2, smooth_dq_PDCal2, smooth_ddq_PDCal2,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PDCal3.csv'; 
[time_PDCal3, q_PDCal3, dq_PDCal3, ddq_PDCal3, smooth_q_PDCal3, smooth_dq_PDCal3, smooth_ddq_PDCal3,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PDCal4.csv'; 
[time_PDCal4, q_PDCal4, dq_PDCal4, ddq_PDCal4, smooth_q_PDCal4, smooth_dq_PDCal4, smooth_ddq_PDCal4,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PDCal5.csv'; 
[time_PDCal5, q_PDCal5, dq_PDCal5, ddq_PDCal5, smooth_q_PDCal5, smooth_dq_PDCal5, smooth_ddq_PDCal5,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

smooth_q_cells = {smooth_q_PDCal1, smooth_q_PDCal2, smooth_q_PDCal3, ...
                  smooth_q_PDCal4, smooth_q_PDCal5};

time_cells = {time_PDCal1, time_PDCal2, time_PDCal3, time_PDCal4, time_PDCal5};

rho_cells = {};

CalibPD = ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_cal, smooth_q_cal, false, rho_cells,false);

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
[time_TnR4, q_TnR4, dq_TnR4, ddq_TnR4, smooth_q_TnR4,smooth_dq_TnR4,smooth_ddq_TnR4, index_TnR4, ~] = ezloopdata_compare(dat, false, false, false, 0);

% Inverse Dynamics Controller on TnR4 w/o forceplate
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CD1.csv'; 
[time_CD1, q_CD1, dq_CD1, ddq_CD1, smooth_q_CD1, smooth_dq_CD1, smooth_ddq_CD1, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CD2.csv'; 
[time_CD2, q_CD2, dq_CD2, ddq_CD2, smooth_q_CD2, smooth_dq_CD2, smooth_ddq_CD2, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CD3.csv'; 
[time_CD3, q_CD3, dq_CD3, ddq_CD3, smooth_q_CD3, smooth_dq_CD3, smooth_ddq_CD3, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CD4.csv'; 
[time_CD4, q_CD4, dq_CD4, ddq_CD4, smooth_q_CD4, smooth_dq_CD4, smooth_ddq_CD4, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CD5.csv'; 
[time_CD5, q_CD5, dq_CD5, ddq_CD5, smooth_q_CD5, smooth_dq_CD5, smooth_ddq_CD5, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR4);

smooth_q_cells = {smooth_q_CD1, smooth_q_CD2, smooth_q_CD3, ...
                  smooth_q_CD4, smooth_q_CD5};

time_cells = {time_CD1, time_CD2, time_CD3, time_CD4, time_CD5};

rho_cells = {};

TnR = ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_TnR4, smooth_q_TnR4, false, rho_cells, false);

% Inverse Dynamics Controller on TnR4 w/ forceplate
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_CD4FP1.csv'; 
[time_CDFP1, q_CDFP1, dq_CDFP1, ddq_CDFP1, smooth_q_CDFP1, smooth_dq_CDFP1, smooth_ddq_CDFP1, ~, rho0_CDFP1] = ezloopdata_compare(dat, false, false, false, index_TnR4);
 
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_CD4FP2.csv'; 
[time_CDFP2, q_CDFP2, dq_CDFP2, ddq_CDFP2, smooth_q_CDFP2, smooth_dq_CDFP2, smooth_ddq_CDFP2, ~, rho0_CDFP2] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_CD4FP3.csv'; 
[time_CDFP3, q_CDFP3, dq_CDFP3, ddq_CDFP3, smooth_q_CDFP3, smooth_dq_CDFP3, smooth_ddq_CDFP3, ~, rho0_CDFP3] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_CD4FP4.csv'; 
[time_CDFP4, q_CDFP4, dq_CDFP4, ddq_CDFP4, smooth_q_CDFP4, smooth_dq_CDFP4, smooth_ddq_CDFP4, ~, rho0_CDFP4] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_CD4FP5.csv'; 
[time_CDFP5, q_CDFP5, dq_CDFP5, ddq_CDFP5, smooth_q_CDFP5, smooth_dq_CDFP5, smooth_ddq_CDFP5, ~, rho0_CDFP5] = ezloopdata_compare(dat, false, false, false, index_TnR4);

smooth_q_cells = {smooth_q_CDFP1, smooth_q_CDFP2, smooth_q_CDFP3, ...
                  smooth_q_CDFP4, smooth_q_CDFP5};

rho_cells = {rho0_CDFP1, rho0_CDFP2, rho0_CDFP3, rho0_CDFP4, rho0_CDFP5};

time_cells = {time_CDFP1, time_CDFP2, time_CDFP3, time_CDFP4, time_CDFP5};

TnRCDFP = ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_TnR4, smooth_q_TnR4, true, rho_cells, true);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% CBF Controller Response for High Conservative Case for Teach and Repeat 4

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CBFH1.csv'; 
[time_H1, q_H1, dq_H1, ddq_H1, smooth_q_H1, smooth_dq_H1, smooth_ddq_H1, ~, rho0_H1] = ezloopdata_compare(dat, false, false, false, index_TnR4);
 
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CBFH2.csv'; 
[time_H2, q_H2, dq_H2, ddq_H2, smooth_q_H2, smooth_dq_H2, smooth_ddq_H2, ~, rho0_H2] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CBFH3.csv'; 
[time_H3, q_H3, dq_H3, ddq_H3, smooth_q_H3, smooth_dq_H3, smooth_ddq_H3, ~, rho0_H3] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CBFH4.csv'; 
[time_H4, q_H4, dq_H4, ddq_H4, smooth_q_H4, smooth_dq_H4, smooth_ddq_H4, ~, rho0_H4] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CBFH5.csv'; 
[time_H5, q_H5, dq_H5, ddq_H5, smooth_q_H5, smooth_dq_H5, smooth_ddq_H5, ~, rho0_H5] = ezloopdata_compare(dat, false, false, false, index_TnR4);

smooth_q_cells = {smooth_q_H1, smooth_q_H2, smooth_q_H3, ...
                  smooth_q_H4, smooth_q_H5};

rho_cells = {rho0_H1, rho0_H2, rho0_H3, rho0_H4, rho0_H5};

time_cells = {time_H1, time_H2, time_H3, time_H4, time_H5};

TnRHigh = ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_TnR4, smooth_q_TnR4, true, rho_cells, true);

%% Example Plotting code used for Robosoft poster
%%%%%%%%%%%%%%%%%%%
sim_calib_data = load('callibration_data.mat');
sim_time = sim_calib_data.export_data(:,1);
sim_q1 = sim_calib_data.export_data(:,2);
sim_q2 = sim_calib_data.export_data(:,3);
sim_q1_inter = sim_calib_data.export_data(:,4);
sim_q2_inter = sim_calib_data.export_data(:,5);

% Side by Side of q1 q2 for Open-Loop trajectroy and CD tracking
% figure
% subplot(1,2,1)
% 
%     fill([CalibCD.time; flipud(CalibCD.time)], ...
%          [CalibCD.q0.upper; flipud(CalibCD.q0.lower)], ...
%          [0 0.604 0.192], ...
%          'EdgeColor','none','FaceAlpha',0.2);
%     hold on
% 
%     p1 = plot(time_cal, smooth_q_cal, ...
%               'LineWidth',2,'Color',[0.25 0.25 0.25]);
% 
%     p2 = plot(time_cal, CalibCD.q0.mu, ...
%               'LineWidth',1,'Color',[0 0.604 0.192]);

%% FIGURE 7 of RandR - Hardware experiment results
% th = figure(1);
% t2 = tiledlayout(5,1);
% t2.TileSpacing = 'compact';
% t2.Padding = 'tight';
% 
% ax1 = nexttile;
% x1 = plot(time_NONET1(idx),Tip_x_NONET1(idx),LineWidth=2,Color=[0.25, 0.25, 0.25],LineStyle="--");
% hold on 
% x2 = plot(time_LOWT1(idx),Tip_x_LOWT1(idx),LineWidth=2,Color=[0.9290, 0.6940, 0.1250]);
% x3 = plot(time_MEDT1(idx),Tip_x_MEDT1(idx),LineWidth=2,Color=[0 0.604 0.192]);
% x4 = plot(time_HIGHT1(idx),Tip_x_HIGHT1(idx),LineWidth=2,Color=[0.188 0.361 0.92]);
% hold off
% 
% ylabel('$r_x$ (m)') 
% 
% grid on;
% xlim([0 70])
% lgd = legend([x1 x2 x3 x4],{'None','Low','Medium','High'},'orientation','horizontal','Location','north');   
% 
% uistack(x2,'bottom')
% uistack(x1,'bottom')
% 
% ax2 = nexttile;
% y1 = plot(time_NONET1(idx),Tip_y_NONET1(idx),LineWidth=2,Color=[0.25, 0.25, 0.25],LineStyle="--");
% hold on 
% y2 = plot(time_LOWT1(idx),Tip_y_LOWT1(idx),LineWidth=2,Color=[0.9290, 0.6940, 0.1250]);
% y3 = plot(time_MEDT1(idx),Tip_y_MEDT1(idx),LineWidth=2,Color=[0 0.604 0.192]);
% y4 = plot(time_HIGHT1(idx),Tip_y_HIGHT1(idx),LineWidth=2,Color=[0.188 0.361 0.92]);
% hold off
% % ax4.FontSize = 16;
% ylabel('$r_y$ (m)') 
% grid on;
% xlim([0 70])
% 
% uistack(y2,'bottom')
% uistack(y1,'bottom')
% 
% ax3 = nexttile;
% f1 = plot(time_NONET1(idx),Fp0_NONET1(idx),LineWidth=2,Color=[0.25, 0.25, 0.25],LineStyle="--");
% hold on 
% f2 = plot(time_LOWT1(idx),Fp0_LOWT1(idx),LineWidth=2,Color=[0.9290, 0.6940, 0.1250]);
% f3 = plot(time_MEDT1(idx),Fp0_MEDT1(idx),LineWidth=2,Color=[0 0.604 0.192]);
% f4 = plot(time_HIGHT1(idx),Fp0_HIGHT1(idx),LineWidth=2,Color=[0.188 0.361 0.92]);
% f5 = plot(time_NONET1(idx),Fmax_vector,LineWidth=1,Color='r');
% hold off
% x_pos = time_LOWT1(end);
% text(x_pos-7, Fmax+0, '$F_{\mathrm{max}}$', 'Interpreter', 'latex', ...
%      'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', ...
%      'Color', 'r', 'FontSize', 12);
% % ax5.FontSize = 16;
% grid on;
% ylabel('F (N)') 
% xlim([0 70])
% 
% uistack(f2,'bottom')
% uistack(f1,'bottom')
% uistack(f5, 'top')
% 
% ax4 = nexttile;
% u1 = plot(time_NONET1(idx),u0_NONET1(idx),LineWidth=2,Color=[0.25, 0.25, 0.25],LineStyle="--");
% hold on 
% u2 = plot(time_LOWT1(idx),smoothed_u0_LOWT1(idx),LineWidth=2,Color=[0.9290, 0.6940, 0.1250]) ;
% u3 = plot(time_MEDT1(idx),smoothed_u0_MEDT1(idx),LineWidth=2,Color=[0 0.604 0.192]);
% u4 = plot(time_HIGHT1(idx),smoothed_u0_HIGHT1(idx),LineWidth=2,Color=[0.188 0.361 0.92]);
% hold off 
% % ax1.FontSize = 16;
% ylabel('$u_1$ (hPa)') 
% grid on;
% xlim([0 70])
% 
% uistack(u2,'bottom')
% uistack(u1,'bottom')
% 
% ax5 = nexttile;
% uu1 = plot(time_NONET1(idx),u1_NONET1(idx),LineWidth=2,Color=[0.25, 0.25, 0.25],LineStyle="--");
% hold on 
% uu2 = plot(time_LOWT1(idx),smoothed_u1_LOWT1(idx),LineWidth=2,Color=[0.9290, 0.6940, 0.1250]);
% uu3 = plot(time_MEDT1(idx),smoothed_u1_MEDT1(idx),LineWidth=2,Color=[0 0.604 0.192]);
% uu4 = plot(time_HIGHT1(idx),smoothed_u1_HIGHT1(idx),LineWidth=2,Color=[0.188 0.361 0.92]);
% hold off
% % ax2.FontSize = 16;
% ylabel('$u_2$ (hPa)') 
% grid on;
% 
% xlabel('Time (s)')
% xlim([0 70])
% 
% uistack(uu2,'bottom')
% uistack(uu1,'bottom')
% 
% hw_ratio = 1.1;
% 
% set(findall(t2,'-property','Interpreter'), 'Interpreter', 'latex')
% set(findall(t2,'-property','TickLabelInterpreter'), 'TickLabelInterpreter', 'latex')
% set(findall(t2,'-property','Box'), 'Box', 'off')
% % set(t2, 'Units', 'centimeters', ...
% %         'Position', [2.3 1 picturewidth_singlecolumn hw_ratio * picturewidth_singlecolumn]);
% set(t2, 'Units', 'centimeters', ...
%         'Position', [3 1 picturewidth_singlecolumn hw_ratio * picturewidth_singlecolumn]);
% grid on;
% 
% lgd.Units = 'normalized';       % Make position relative to axes
% pos1 = lgd.Position;
% pos1(1) = pos1(1) + 0.0;         % Move right
% pos1(2) = pos1(2) + 0.07;         % Move up
% lgd.Position = pos1;
% 
% fname = 'HW_results_RandR_v3';
% % exportgraphics(th, strcat(fname, '.eps'), 'ContentType', 'vector');
% % exportgraphics(th, strcat(fname, '.pdf'), 'ContentType', 'vector');
% exportgraphics(th, strcat(fname, '.png'), 'ContentType', 'vector');

figure(1)

out1 = TnRCDFP;

out2 = TnRHigh;
hold on
fill(out1.rho.fillX, out1.rho.fillY, [0 0.447 0.741], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.15);

fill(out2.rho.fillX, out2.rho.fillY, [0.850 0.325 0.098], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.15);

p1 = plot(out1.rho.time, out1.rho.mu, 'LineWidth', 1.5);
p2 = plot(out2.rho.time, out2.rho.mu, 'LineWidth', 1.5);
p3 = plot(out1.rho.time, out1.rho.max, '--r', 'LineWidth', 1.5);

ylabel('Rho');xlabel('Time(s)')
legend([p1,p2,p3],{'Tracking Only','Tracking with CBF','Max Allowable Rho'},'Location','best','FontSize',12);