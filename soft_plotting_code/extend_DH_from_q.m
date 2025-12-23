% [DHparams] = extend_DH_from_q(DHparams,q,L,m)
function [DHparams] = extend_DH_from_q(DHparams_previous_joints,q,L)
    previous_links_num = size(DHparams_previous_joints,1);
    linkno = (previous_links_num+1:1:previous_links_num+4)';
    thetas = [q/2, 0, 0, q/2]';
    % ds     = [0 (L*(sin(q/2)/q)) (L*(sin(q/2)/q)) 0]';
    % as     = [0, 0, 0, 0]';
    ds     = [0, 0, 0, 0]';
    as     = [0 (L*(sin(q/2)/q)) (L*(sin(q/2)/q)) 0]'; % ds and as switched??
                                                       % doing this switch just 
                                                       % swaps the x y coordinates
    alphas = [pi/2, 0, -pi/2, 0]';
    DHparams = [linkno, thetas, ds, alphas, as];
    DHparams = [DHparams_previous_joints;DHparams];
end
