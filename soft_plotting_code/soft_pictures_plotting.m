function soft_pictures_plotting(q_config, L, H, h, F_max, k_env)
    % q_config: 2x1 vector of joint angles [q1; q2] in radians
    % L: 2x1 vector of limb lengths
    % H, h: halfspace for polytope
    % F_max, k_env: safe set expansion parameters

    % Generate symbolic DH structure
    syms q1 q2 L1 L2 real;
    DHparams = [];
    DHparams = extend_DH_from_q(DHparams, q1, L1);
    DHparams = extend_DH_from_q(DHparams, q2, L2);

    nl1 = L(1); nl2 = L(2); nl = max([nl1, nl2]);

    % Generate safety polytopes
    P1 = Polyhedron(H, h);
    [H_prime, h_prime] = generate_safeset_expansion(H, h, F_max, k_env);
    P2 = Polyhedron(H, h_prime);

    % Begin plot
    figure; hold on;
    % plot(P2, 'color', 'm', 'EdgeColor', 'm');  % expanded safe set
    % plot(P1, 'color', [1 1 1], 'EdgeColor', 'k');       % original safe set

    axis([-1.15*2*nl 1.15*2*nl -1.15*2*nl 1.15*2*nl]);
    axis square;
    xlabel('X (m)'); ylabel('Y (m)');
    title('Soft Robot Arm at One Pose');
    % grid on;


    % Plot the robot at the given configuration
    plotRPPRarms(DHparams, [nl1, nl2], q_config, [0.3 0.3 1]);
end