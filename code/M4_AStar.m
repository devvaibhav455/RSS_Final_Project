% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found] = M4_AStar(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii,sampling_strategy,num_samples, num_neighbors, samples, adjacency)
    % Generate graph using M2 function
%     [samples, adjacency] = M2(robot, q_min, q_max, num_samples, sampling_strategy, num_neighbors, link_radius, sphere_centers, sphere_radii);
%     save("samples_adjacency_matrix_new.mat", "samples", "adjacency");
%     load samples_adjacency_matrix_new.mat;
    k = 3;
    
    % Add start and goal configurations to graph if they are not already there
    if ~all(all(samples == q_start))
        samples(end+1, :) = q_start;
        adjacency(end+1, :) = 0;
        adjacency(:, end+1) = 0;

        % Find distances to all other configurations
        distances = pdist2(samples, q_start);

        % Add edges to k-nearest neighbors
        [~, indices] = sort(distances);
        for i = 2:k+1
            neighbor_idx = indices(i);
            dist = distances(neighbor_idx);
            adjacency(end, neighbor_idx) = dist;
            adjacency(neighbor_idx, end) = dist;
        end
    end

    if ~all(all(samples == q_goal))
        samples(end+1, :) = q_goal;
        adjacency(end+1, :) = 0;
        adjacency(:, end+1) = 0;

        % Find distances to all other configurations
        distances = pdist2(samples, q_goal);

        % Add edges to k-nearest neighbors
        [~, indices] = sort(distances);
        for i = 2:k+1
            neighbor_idx = indices(i);
            dist = distances(neighbor_idx);
            adjacency(end, neighbor_idx) = dist;
            adjacency(neighbor_idx, end) = dist;
        end
    end
%     disp(samples)
    % Find the indices of start and goal configurations in the graph
    start_idx = find(all(samples == q_start, 2));
%     disp(start_idx)
    goal_idx = find(all(samples == q_goal, 2));
%     disp(goal_idx)
    function h = euclidean_distance(q1, q2)
        h = norm(q1 - q2);
    end
    
    % Run A* algorithm to find the shortest path between start and goal configurations
    [path, path_found] = A_star(adjacency,samples, start_idx, goal_idx, @(q1, q2) euclidean_distance(samples(q1, :), samples(q2, :)));
end

function [path, path_found] = A_star(adj_matrix,samples, start_idx, goal_idx, heuristic_func)
    tic;
    num_nodes = size(adj_matrix, 1);
    g_scores = inf(1, num_nodes);
    f_scores = inf(1, num_nodes);
    open_list = start_idx;
    closed_list = [];
    g_scores(start_idx) = 0;
    f_scores(start_idx) = heuristic_func(start_idx, goal_idx);
    parent_nodes = -1 * ones(1, num_nodes);
    parent_nodes(start_idx) = start_idx;
    
    while ~isempty(open_list)
        % Get the node with the lowest f_score
        [~, idx] = min(f_scores(open_list));
        current_node = open_list(idx);

        if current_node == goal_idx
            % Reconstruct the path
            path = samples(current_node, :);
            node = current_node;
            while node ~= start_idx
                parent = parent_nodes(node);
                path = [samples(parent, :); path];
                node = parent;
            end
            path_found = true;
            return
        end

        % Move current node from open to closed list
        open_list(idx) = [];
        closed_list = [closed_list, current_node];

        % Check neighbors
        neighbor_indices = find(adj_matrix(current_node, :));
        for neighbor_idx = neighbor_indices
            if any(closed_list == neighbor_idx)
                continue
            end

            tentative_g_score = g_scores(current_node) + adj_matrix(current_node, neighbor_idx);
            if ~any(open_list == neighbor_idx)
                % The neighbor is not in the open list, add it
                open_list(end+1) = neighbor_idx;
                h_score = heuristic_func(neighbor_idx, goal_idx);
            elseif tentative_g_score >= g_scores(neighbor_idx)
                % The tentative score is greater than or equal to the current score, skip it
                continue
            else
                % Update the score of the neighbor
                h_score = f_scores(neighbor_idx) - g_scores(neighbor_idx);
            end

            g_scores(neighbor_idx) = tentative_g_score;
            f_scores(neighbor_idx) = g_scores(neighbor_idx) + h_score;
            parent_nodes(neighbor_idx) = current_node;
        end
    end

    % No path found
    path = [];
    path_found = false;
end
