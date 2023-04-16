% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        x_min -> 1x4 vector of minimum angle for each joint
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

function [path, path_found] = RRTStar(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii,sampling_strategy)
    %Src: https://www.mathworks.com/help/matlab/math/add-graph-node-names-edge-weights-and-other-attributes.html
    
    % Build a tree from q_start to q_goal.
    path_found = false;
    path = q_start;
    G = digraph; %Create an empty tree. It won't create graph because it won't be cyclic
    
    
    % Initialize the tree with the start configuration
    G = addnode(G,1); % Add one node to the graph.
    V = q_start; %Contains all the vertices

%     V = [V; q_goal]; %JUST FOR TESTING  TESTING TESTING


    % Number of nodes in the tree
    n = 500; 
    % Hyperparameter 1: step size/ alpha
    alpha = 0.2;
    %Hyperparameter 2: used to calculate the search radius
    gamma = 100;
    % Hyperparameter 3: used to calculate the search radius
    d = 3;
    %Hyperparameter 4: used to calculate the search radius
    eta = 0.1;

    % Threshold parameter to judge if q_goal is near q_near or not 
    threshold = 0.3;
    
    %Hyperparameter 2: goal biasing parameter/ beta
    beta = 0.1;
    
    % Boolean variable to keep track of whether goal has been found
    goal_found = false;
    
    % Start building the tree
    
    for i = 1:n
%         fprintf('\n############################# %d #########################', i)

        
        % Select a random configuration from the free space
        if rand() < beta
            q_rand = q_goal;
        else
            q_rand = M1(q_min, q_max, 1, sampling_strategy, robot, link_radius, sphere_centers, sphere_radii);
        end
%         q_rand
%         fprintf('Configuration selected')

        % Find the closest node in the tree to the target configuration
        q_nearest = closet_neighbor(q_rand, V);

        % Compute the new node by taking a step of size alpha from
        % q_nearest i.e. steering
        q_diff = (q_rand - q_nearest);
        q_new = q_nearest + alpha * q_diff / norm(q_diff);

        % Check if the edge from q_nearest to q_new is collision-free.
        if check_edge(robot, q_nearest, q_new, link_radius, sphere_centers, sphere_radii) == false
            %Find all the nodes in G which are within a certain radius away
            %from q_new | Src: https://www.mathworks.com/help/matlab/ref/graph.nearest.html
%             radius = min(gamma*(log(size(V,1))/size(V,1))^(1/d), eta)
            radius = 0.5; % TESTING
            
            
            Q_near = find_all_close_neighbors(q_new, V, radius);
            
            % Update V with the new node.
            V = [V; q_new];
            q_new_index = find(ismember(V,q_new,'rows'));
%             q_new_index = q_new_index(1)
            G = addnode(G,1); % Add one node to the graph.
            
            x_min = q_nearest;
            % Find index of q_nearest in V
            x_min_index = find(ismember(V,x_min,'rows'));
            c_min = distances(G,1,x_min_index) + vecnorm(q_new - q_nearest, 2, 2);
            
            for j = 1:numel(Q_near)
                q_near = V(Q_near(j),:);
                q_near_index = find(ismember(V,q_nearest,'rows'));
                
                if (check_edge(robot, q_near, q_new, link_radius, sphere_centers, sphere_radii) == false) && (distances(G,1,q_near_index) + vecnorm(q_near - q_new, 2, 2) < c_min)
                    x_min = q_near;
                    x_min_index = find(ismember(V,x_min,'rows')); 
                    c_min = distances(G,1,q_near_index) + vecnorm(q_near - q_new, 2, 2);
                end
            end
            
            G = addedge(G,x_min_index, q_new_index, vecnorm(x_min - q_new, 2, 2));

            
            % Rewire the tree
            for j = 1:numel(Q_near)
%                 disp('Rewiring the tree')
                q_near = V(Q_near(j),:);
                q_near_index = find(ismember(V,q_near,'rows'));
                
%                 distances(G,1,q_new_index)
%                 vecnorm(q_new - q_near, 2, 2)
                if (check_edge(robot, q_near, q_new, link_radius, sphere_centers, sphere_radii) == false) && (distances(G,1,q_new_index) + vecnorm(q_new - q_near, 2, 2) < distances(G,1,q_near_index))
                    disp('Deleting the edge from tree')
                    q_parent_index = predecessors(G,q_near_index);
                    % https://www.mathworks.com/help/matlab/ref/graph.rmedge.html
                    G = rmedge(G,q_parent_index, q_near_index);
                    G = addedge(G,q_new_index, q_near_index, vecnorm(q_new - q_near, 2, 2));

                end
            end

%             norm(q_goal - q_new)
            % When the new node is close enough to the goal node,
            % check if it can reach the goal node without collision.
            if norm(q_goal - q_new) < threshold
                % RRT  halts when the new node connects to the goal
                % node without collision.
                if ~check_edge(robot, q_new, q_goal, link_radius, sphere_centers, sphere_radii)
                    goal_found = true;
                    V = [V; q_goal];
                    q_goal_index = find(ismember(V,q_goal,'rows')) ;
                    G = addnode(G,1); % Add one node to the graph.
                    G = addedge(G,q_new_index, q_goal_index, vecnorm(q_new - q_goal, 2, 2));
                    fprintf('\n########### PATH FOUND ##############\n');
                    break;
%                     E = [E; q_new; q_goal];
                end
            end
%             V(:,1)
%             V(:,4)
        figure(2);
        plot3(V(:,1),V(:,2),V(:,4),".")
        grid on
        xlabel('q1')
        ylabel('q2')
        zlabel('q4')
        xlim([-3*pi/2 3*pi/2])
        ylim([-1.5*pi 0.1])
        zlim([-1.5*pi 0.1])

        txt_start = '\leftarrow START';
        txt_goal = '\leftarrow GOAL';
        text(q_start(1),q_start(2),q_start(4),txt_start)
        text(q_goal(1),q_goal(2),q_goal(4),txt_goal)
        M(i) = getframe;
        hold off
        end
%     pause(5)
    end
    
    figure(3);
    plot(G)
    hold off

    
    
    if goal_found == true
        % Index of q_start in V
        index_q_start = find(ismember(V,q_start,'rows'));
        % Index of q_goal in V
        index_q_goal = find(ismember(V,q_goal,'rows'));
        
        path_indexes = shortestpath(G,index_q_start,index_q_goal);
        path_indexes = path_indexes';
        path = zeros(length(path_indexes), 4);
        path_found = false;
        if isempty(path_indexes) == 0
            path_found = true;
            for i = 1:length(path_indexes)
                path(i,:) = V(path_indexes(i),:);
            end
        end
    end
    
end

% Input:    p           ->   1x4 vector of a point from which the distance to all the
%                            points needs to be calculated
%           points      ->   no. of points x 4 matrix, vertices in the tree
%      
% Output:   neighbor    ->   1x4 vector which is the closest neighbor of the
function neighbor = closet_neighbor(p, points)
    [~, I] = min(vecnorm(points - p, 2, 2)); % 2 norm in the 2 direction
    neighbor = points(I,:);
end

% Input:    p                    ->   1x4 vector of a point from which the distance to all the
%                                     points needs to be calculated
%           points               ->   no. of points x 4 matrix, vertices in the tree
%      
% Output:   neighbor_index       ->   1x4 vector which is the closest neighbor of the
function neighbor_index = find_all_close_neighbors(p, points, radius)
%     [~, I] = min(vecnorm(points - p, 2, 2)) % 2 norm in the 2 direction
%     vecnorm(points - p, 2, 2)
    neighbor_index = find(vecnorm(points - p, 2, 2) < radius);
%     sorted_dist_and_indexes = sortrows(vecnorm(points - p, 2, 2))
%     neighbor = points(I,:);
end
