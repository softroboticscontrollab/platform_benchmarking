function [A, b] = computeSafetyConstraints(polygonVertices)
    % Compute the convex hull of the vertices to handle non-convex polygons
    hullIndices = convhull(polygonVertices(:, 1), polygonVertices(:, 2));
    polygonVertices = polygonVertices(hullIndices(1:end-1), :); % Remove duplicate vertex at the end

    % Ensure the polygon vertices are ordered counter-clockwise
    polygonVertices = ensureCounterClockwise(polygonVertices);

    numEdges = size(polygonVertices, 1); % Number of edges in the polygon
    A = zeros(numEdges, 2); % Initialize matrix A (normals of edges)
    b = zeros(numEdges, 1); % Initialize vector b (offsets of edges)

    % Compute the mean (centroid) of the polygon
    centroid = mean(polygonVertices);

    % Loop through all edges to compute the constraints
    for i = 1:numEdges
        % Get the vertices of the current edge
        p1 = polygonVertices(i, :);
        p2 = polygonVertices(mod(i, numEdges) + 1, :); % Next vertex (wraps around)

        % Calculate the edge vector
        edgeVector = p2 - p1;

        % Calculate the outward normal vector (90-degree rotation)
        normal = [-edgeVector(2), edgeVector(1)]; % Perpendicular vector
        normal = normal / norm(normal); % Normalize

        % Ensure the normal points outward by testing against the centroid
        if dot(normal, centroid - p1) > 0
        % if dot(normal, centroid - p1) < 0
            normal = -normal; % Flip to point outward
        end

        % Store the normal vector in the matrix A
        A(i, :) = normal;

        % Calculate the offset using the dot product with the first vertex of the edge
        b(i) = dot(normal, p1);
    end
end

function vertices = ensureCounterClockwise(vertices)
    % Ensure vertices are ordered counter-clockwise
    % Compute signed area (Shoelace formula)
    area = 0;
    numVertices = size(vertices, 1);
    for i = 1:numVertices
        p1 = vertices(i, :);
        p2 = vertices(mod(i, numVertices) + 1, :); % Next vertex (wraps around)
        area = area + (p2(1) - p1(1)) * (p2(2) + p1(2));
    end
    % Reverse order if not counter-clockwise
    if area > 0
        vertices = flipud(vertices);
    end
end