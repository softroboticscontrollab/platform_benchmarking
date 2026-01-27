% function u = u_sim_traj(~,~,~)
% 
%     persistent u_sim idx
% 
%     if isempty(u_sim)
%         traj = load('u_traj_time.mat');
%         u_sim = traj.u_traj_time;
%         idx = 1;
%     end 
% 
%     u = u_sim(:,idx);
% 
%     if idx < size(u_sim,2)
%         idx = idx + 1;
%     end
% 
% 
% end

function u = u_sim_traj(~,~,t)

    persistent u_traj_time time_data tmin tmax

    if isempty(u_traj_time)
        S = load('u_traj_time.mat');
        u_traj_time = S.u_traj_time;
        time_data = S.time_data(:);
        time_data = time_data - time_data(1);
        tmin = time_data(1);
        tmax = time_data(end);
    end 
    tq = min(max(t,tmin),tmax);
    idx = find(time_data <= tq,1,'last');
    
    if isempty(idx)
        
        idx = 1;
    
    end

    u = u_traj_time(:,idx);

end