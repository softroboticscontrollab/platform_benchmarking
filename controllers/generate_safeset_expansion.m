function [H_prime, h_prime] = generate_safeset_expansion(H_o, h_o, F_max, k_env)
    % H_o: original halfspace matrix (each row is a normal vector)
    % h_o: original offset vector
    % F_max: maximum allowable force
    % k_env: environment stiffness (spring constant)

    n_constraints = size(H_o, 1);
    h_prime = h_o;
    H_prime = H_o;

    % Max normal displacement from inverse force model
    n_max = psi_inv(F_max, k_env);

    for i = 1:n_constraints
        % Shift the offset along the normal direction
        norm_Hi = norm(H_o(i,:));      % Magnitude of the normal
        h_prime(i) = h_prime(i) + n_max * norm_Hi; % Move outwards
    end
end

function d = psi_inv(F, k)
    d = F / k;
end
