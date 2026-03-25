function ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_ref, q_ref, includeRho, rho_cells)

% plotCDFPTracking
% Computes mean and ±2σ envelopes for multiple experiments and plots them.
%
% INPUTS
% smooth_q_cells : cell array {N} each [T x 2] matrix (q0,q1)
% time_cells     : cell array {N} each [T x 1]
% time_ref       : reference time vector
% q_ref          : reference trajectory
% includeRho     : boolean flag (true = plot rho, false = omit)
% rho_cells      : cell array {N} each [T x 1] (required only if includeRho=true)

nRuns = length(smooth_q_cells);

%% Determine common endpoint
lengths = zeros(nRuns,1);

for i = 1:nRuns
    lengths(i) = size(smooth_q_cells{i},1);
end

% Include rho lengths if rho is used
if includeRho
    for i = 1:nRuns
        lengths(i) = min(lengths(i), size(rho_cells{i},1));
    end
end

% Include reference trajectory lengths
refLength = min(length(time_ref), length(q_ref));

endpoint = min([lengths; refLength]);

%% Assemble matrices
q0_all = zeros(endpoint,nRuns);
q1_all = zeros(endpoint,nRuns);

for i = 1:nRuns
    q0_all(:,i) = smooth_q_cells{i}(1:endpoint,1);
    q1_all(:,i) = smooth_q_cells{i}(1:endpoint,2);
end

%% Statistics for q0 and q1
q0_mu = mean(q0_all,2);
q0_sigma = std(q0_all,0,2);

q1_mu = mean(q1_all,2);
q1_sigma = std(q1_all,0,2);

q0_upper = q0_mu + 2*q0_sigma;
q0_lower = q0_mu - 2*q0_sigma;

q1_upper = q1_mu + 2*q1_sigma;
q1_lower = q1_mu - 2*q1_sigma;

%% Rho statistics (only if requested)
if includeRho

    rho_all = zeros(endpoint,nRuns);

    for i = 1:nRuns
        rho_all(:,i) = rho_cells{i}(1:endpoint);
    end

    rho_mu = mean(rho_all,2);
    rho_sigma = std(rho_all,0,2);

    rho_upper = rho_mu + 2*rho_sigma;
    rho_lower = rho_mu - 2*rho_sigma;

end

%% Time vector
t = time_cells{1}(1:endpoint);

%% Determine number of subplots
if includeRho
    nSub = 3;
else
    nSub = 2;
end

figure

%% q0 subplot
subplot(nSub,1,1)

fill([t; flipud(t)], ...
     [q0_upper; flipud(q0_lower)], ...
     [0 0.604 0.192], ...
     'EdgeColor','none','FaceAlpha',0.2);
hold on

p1 = plot(time_ref(1:endpoint), q_ref(1:endpoint,1), ...
          'LineWidth',2,'Color',[0.25 0.25 0.25]);

p2 = plot(t, q0_mu, ...
          'LineWidth',1,'Color',[0 0.604 0.192]);

legend([p1 p2],{'Reference Trajectory','Achieved Trajectory'}, ...
       'Location','northeast','FontSize',15);

ylabel('q_0 (radians)','FontSize',14)
title('Tracking with CD with Force Plate')

hold off


%% q1 subplot
subplot(nSub,1,2)

fill([t; flipud(t)], ...
     [q1_upper; flipud(q1_lower)], ...
     [0.188 0.361 0.92], ...
     'EdgeColor','none','FaceAlpha',0.2);
hold on

p1 = plot(time_ref(1:endpoint), q_ref(1:endpoint,2), ...
          'LineWidth',2,'Color',[0.25 0.25 0.25]);

p2 = plot(t, q1_mu, ...
          'LineWidth',1,'Color',[0.188 0.361 0.92]);

legend([p1 p2],{'Reference Trajectory','Achieved Trajectory'}, ...
       'Location','northeast','FontSize',15);

ylabel('q_1 (radians)','FontSize',14)
xlabel('Time (s)','FontSize',14)

hold off


%% rho subplot (optional)
if includeRho

    subplot(3,1,3)

    rho_max = zeros(endpoint,1);

    fill([t; flipud(t)], ...
         [rho_upper; flipud(rho_lower)], ...
         [0.188 0.361 0.92], ...
         'EdgeColor','none','FaceAlpha',0.2);
    hold on

    p1 = plot(time_ref(1:endpoint), rho_max, ...
              '--','LineWidth',2,'Color','red');

    p2 = plot(t, rho_mu, ...
              'LineWidth',1,'Color',[0.188 0.361 0.92]);

    legend([p1 p2],{'Minimum Rho','Calculated Rho'}, ...
           'Location','northeast','FontSize',15);

    ylabel('Rho','FontSize',14)
    xlabel('Time (s)','FontSize',14)

    hold off

end

end