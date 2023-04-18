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

function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii,sampling_strategy)
    % Build a tree from q_start to q_goal.
    
    % Number of nodes in the tree
    n = 500; 
    % Hyperparameter 1: step size/ alpha
    alpha = 0.2;
    %Hyperparameter 2: goal biasing parameter/ beta
    beta = 0.1;
    % Threshold parameter to judge if q_goal is near q_near or not 
    threshold = 0.3;
    
    % Initialize the tree with the start configuration
    V = q_start;
    E = [];

    % Boolean variable to keep track of whether goal has been found
    goal_found = false;
    
    % Start building the tree
    for i = 1:n
        if goal_found
            % If the goal has been found, stop building the tree
            break
        end
        
        % Select a target configuration. With probability < beta, the target
        % configuration is q_goal. Otherwise, it is a random configuration.
        if rand() < beta
            q_target = q_goal;
        else
            q_target = M1(q_min, q_max, 1, sampling_strategy, robot, link_radius, sphere_centers, sphere_radii);
            
        end

        % Find the closest node in the tree to the target configuration
        q_near = closet_neighbor(q_target, V);
        % Compute the new node by taking a step of size alpha from q_near
        q_diff = (q_target - q_near);
        q_new = q_near + alpha * q_diff / norm(q_diff);

        % Check if the new node is collision-free and the edge from q_near to
        % q_new is collision-free. If so, add the new node to the tree and add
        % the edge from q_near to q_new to the tree.
        if ~check_collision(robot, q_new, link_radius, sphere_centers, sphere_radii)...
           && ~check_edge(robot, q_near, q_new, link_radius, sphere_centers, sphere_radii)
            % Update V and E with the new node.
            V = [V; q_new]; 
            E = [E; q_near; q_new]; 

            % When the new node is close enough to the goal node,
            % check if it can reach the goal node without collision.
            if norm(q_goal - q_new) < threshold
                % RRT  halts when the new node connects to the goal
                % node without collision.
                if ~check_edge(robot, q_new, q_goal, link_radius, sphere_centers, sphere_radii)
                    goal_found = true;
                    V = [V; q_goal];
                    E = [E; q_new; q_goal];
                end
            end
        end
%         plot3(V(:,1),V(:,2),V(:,4))
%         grid on
%         xlabel('q1')
%         ylabel('q2')
%         zlabel('q4')
%         xlim([-3*pi/2 3*pi/2])
%         ylim([-1.5*pi 0.1])
%         zlim([-1.5*pi 0.1])
% 
%         txt_start = '\leftarrow START';
%         txt_goal = '\leftarrow GOAL';
%         text(q_start(1),q_start(2),q_start(4),txt_start)
%         text(q_goal(1),q_goal(2),q_goal(4),txt_goal)
%         M(i) = getframe;
%         hold on
    end

    %Construct matlab tree from the E matrix by using s and t approach.
    % s is configurations matrix of parent
    s = zeros(size(E,1)/2 , 4); 
    % t is configurations matrix of child
    t = zeros(size(E,1)/2 , 4);
    
    % Consecutive entries of E are parent-child pair
    for i = 1:size(E,1)/2
        s(i,:) = E((2*i) - 1 , :);
        t(i,:) = E(2*i , :);
    end
    
    % For each configuration in s and t, find its index in V
    s_indexes = zeros(size(s,1),1);
    t_indexes = zeros(size(s,1),1);
    for i = 1:size(s,1)
       s_indexes(i) = find(ismember(V,s(i,:),'rows')); 
       t_indexes(i) = find(ismember(V,t(i,:),'rows'));
    end
    
    % Generate a tree/ graph from parent and child
    G = graph(s_indexes,t_indexes);
%     figure;
%     plot(G)
    

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

% Input:    p           ->   1x4 vector of a point from which the distance to all the
%                            points needs to be calculated
%           points      ->   no. of points x 4 matrix, vertices in the tree
%      
% Output:   neighbor    ->   1x4 vector which is the closest neighbor of the
function neighbor = closet_neighbor(p, points)
    [~, I] = min(vecnorm(points - p, 2, 2)); % 2 norm in the 2 direction
    neighbor = points(I,:);
end
