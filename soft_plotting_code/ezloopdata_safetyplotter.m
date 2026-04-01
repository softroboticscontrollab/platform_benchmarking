function out = ezloopdata_safetyplotter( ...
    smooth_q_cells, time_cells, time_ref, q_ref, includeRho, rho_cells, doPlot)

% plotCDFPTracking
% Computes mean and ±2σ envelopes for multiple experiments and optionally plots them.

nRuns = length(smooth_q_cells);

%% Initialize output struct
out = struct();
out.meta.nRuns = nRuns;
out.meta.includeRho = includeRho;
out.meta.doPlot = doPlot;

% Preallocate graphics handles so they always exist
out.fig = [];
out.ax = struct();
out.plotHandles = struct();

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
refLength = min(length(time_ref), size(q_ref,1));

endpoint = min([lengths; refLength]);

out.meta.endpoint = endpoint;

%% Assemble matrices
q0_all = zeros(endpoint,nRuns);
q1_all = zeros(endpoint,nRuns);

for i = 1:nRuns
    q0_all(:,i) = smooth_q_cells{i}(1:endpoint,1);
    q1_all(:,i) = smooth_q_cells{i}(1:endpoint,2);
end

%% Store raw matrices
out.raw.q0_all = q0_all;
out.raw.q1_all = q1_all;

%% Statistics for q0 and q1
q0_mu = mean(q0_all,2);
q0_sigma = std(q0_all,0,2);

q1_mu = mean(q1_all,2);
q1_sigma = std(q1_all,0,2);

q0_upper = q0_mu + 2*q0_sigma;
q0_lower = q0_mu - 2*q0_sigma;

q1_upper = q1_mu + 2*q1_sigma;
q1_lower = q1_mu - 2*q1_sigma;

%% Store q statistics
out.q0.mu = q0_mu;
out.q0.sigma = q0_sigma;
out.q0.upper = q0_upper;
out.q0.lower = q0_lower;

out.q1.mu = q1_mu;
out.q1.sigma = q1_sigma;
out.q1.upper = q1_upper;
out.q1.lower = q1_lower;

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

    %% Store rho
    out.raw.rho_all = rho_all;

    out.rho.mu = rho_mu;
    out.rho.sigma = rho_sigma;
    out.rho.upper = rho_upper;
    out.rho.lower = rho_lower;

end

%% Time vector
t = time_cells{1}(1:endpoint);
t_ref = time_ref(1:endpoint);

out.time = t;
out.time_ref = t_ref;

%% Store fill() data for q0 and q1 (for external plotting)

% q0 fill
out.q0.fillX = [t; flipud(t)];
out.q0.fillY = [out.q0.upper; flipud(out.q0.lower)];

% q1 fill
out.q1.fillX = [t; flipud(t)];
out.q1.fillY = [out.q1.upper; flipud(out.q1.lower)];

%% Store reference
out.reference.q = q_ref(1:endpoint,:);

%% (Optional) store original cells for debugging
out.raw.smooth_q_cells = smooth_q_cells;
out.raw.time_cells = time_cells;

if includeRho
    out.raw.rho_cells = rho_cells;
end

%% X-axis limit based on actual plotted data
xEnd = max([t(end), t_ref(end)]);
out.meta.xlim = [t(1), xEnd];

%% =========================
%% Plotting (conditional)
%% =========================
if doPlot

    if includeRho
        nSub = 3;
    else
        nSub = 2;
    end

    out.fig = figure;

    %% q0 subplot
    out.ax.q0 = subplot(nSub,1,1);

    out.plotHandles.q0.band = fill([t; flipud(t)], ...
         [q0_upper; flipud(q0_lower)], ...
         [0.188 0.361 0.92], ...
         'EdgeColor','none','FaceAlpha',0.2);
    hold on

    out.plotHandles.q0.reference = plot(t_ref, q_ref(1:endpoint,1), ...
              'LineWidth',2,'Color',[0.25 0.25 0.25]);

    out.plotHandles.q0.mean = plot(t, q0_mu, ...
              'LineWidth',1,'Color',[0.188 0.361 0.92]);

    out.plotHandles.q0.legend = legend( ...
        [out.plotHandles.q0.reference, out.plotHandles.q0.mean], ...
        {'Simulated','Achieved'}, ...
        'Location','northeast','FontSize',12);

    ylabel('q_0 (radians)','FontSize',14)
    xlabel('Time (s)','FontSize',14)
    title('Tracking with CD with Force Plate')

    xlim(out.ax.q0, out.meta.xlim)

    hold off


    %% q1 subplot
    out.ax.q1 = subplot(nSub,1,2);

    out.plotHandles.q1.band = fill([t; flipud(t)], ...
         [q1_upper; flipud(q1_lower)], ...
         [0.188 0.361 0.92], ...
         'EdgeColor','none','FaceAlpha',0.2);
    hold on

    out.plotHandles.q1.reference = plot(t_ref, q_ref(1:endpoint,2), ...
              'LineWidth',2,'Color',[0.25 0.25 0.25]);

    out.plotHandles.q1.mean = plot(t, q1_mu, ...
              'LineWidth',1,'Color',[0.188 0.361 0.92]);

    out.plotHandles.q1.legend = legend( ...
        [out.plotHandles.q1.reference, out.plotHandles.q1.mean], ...
        {'Simulated','Achieved'}, ...
        'Location','northeast','FontSize',12);

    ylabel('q_1 (radians)','FontSize',14)
    xlabel('Time (s)','FontSize',14)

    xlim(out.ax.q1, out.meta.xlim)

    hold off


    %% rho subplot (optional)
    if includeRho

        out.ax.rho = subplot(3,1,3);

        rho_max = zeros(endpoint,1);

        out.rho.all = rho_all;
        out.rho.time = t;
        out.rho.max = rho_max;

        out.rho.fillX = [t; flipud(t)];
        out.rho.fillY = [rho_upper; flipud(rho_lower)];
        out.rho.fillColor = [0.188 0.361 0.92];
        out.rho.fillAlpha = 0.2;

        out.plotHandles.rho.band = fill([t; flipud(t)], ...
             [rho_upper; flipud(rho_lower)], ...
             [0.188 0.361 0.92], ...
             'EdgeColor','none','FaceAlpha',0.2);
        hold on

        out.plotHandles.rho.maximum = plot(t_ref, rho_max, ...
                  '--','LineWidth',2,'Color','red');

        out.plotHandles.rho.mean = plot(t, rho_mu, ...
                  'LineWidth',1,'Color',[0.188 0.361 0.92]);

        out.plotHandles.rho.legend = legend( ...
            [out.plotHandles.rho.maximum, out.plotHandles.rho.mean], ...
            {'Minimum Rho','Calculated Rho'}, ...
            'Location','northeast','FontSize',12);

        ylabel('Rho','FontSize',14)
        xlabel('Time (s)','FontSize',14)

        xlim(out.ax.rho, out.meta.xlim)

        hold off

    end

end

end