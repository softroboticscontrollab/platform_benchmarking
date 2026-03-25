function u = u_Pressuretunner(~,~,t)
% Square-wave switching:
% u = 0 for first half-period
% u = 100 for second half-period

T = 40;

phase = mod(t, T);

% if phase < T/2
%     u = [0;-70];
% else
%     u = [0;0];
% end

% u = [79.8727;79.8756]; % location of forceplate in paper experiments

u = [52.0741;44.9907]; % useful for gathering data for teach_and_repeat

end
