function u = u_sim_traj()

    persistent u_sim idx

    if isempty(u_sim)
        traj = load('u_sim.mat');
        u_sim = traj.u_traj;
        idx = 1;
    end 

    u = u_sim(:,idx) ;

    if idx < size(u_sim,2)
        idx = idx + 1;
    end

end

