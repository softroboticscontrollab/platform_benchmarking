function [M, C] = generate_M_and_C(x, c)  
    q = x(1:2);
    q_dot = x(3:4);

    % Now using numeric versions
    M = get_M_comp(q(1), q(2), c.l1, c.l2, c.m2, c.m6);
    C = get_C_comp(q(1), q(2), q_dot(1), q_dot(2), c.l1, c.l2, c.m6);
end
