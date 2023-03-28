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

function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    
    % Need to generate num_vertices_to_pick no. of vertices in the final output
    num_vertices_to_pick = 100;
    vertices = zeros(num_vertices_to_pick, 4);
    vertices_picked = 0;
     parent child array

    % Generating 100 vertices all of which are valid
    
    q_in_graph = [0 0 0 0; 0 0 0 0]; %any function didn't work as expected when it had only 1 row. So, initialized with 2 rows
    while vertices_picked < num_vertices_to_pick
        % Pick up a random index from 1 to 3*num_vertices_to_pick
        rng shuffle;
        
%         my_random_index = pick_sample_indices_from(randi(numel(pick_sample_indices_from)));
%         my_random_index = 1 + (3*num_vertices_to_pick-1)*rand()
        %Check if this random index is already picked or not/ pick a unique
        %sample each time
%         if ~any(indexes_in_graph(:) == my_random_index)
            q1 = q_min(1) + (q_max(1) - q_min(1))*rand();
            q2 = q_min(2) + (q_max(2) - q_min(2))*rand();
            q3 = q_min(3) + (q_max(3) - q_min(3))*rand();
            q4 = q_min(4) + (q_max(4) - q_min(4))*rand();
            q = [q1 q2 q3 q4];
%             q_in_graph(vertices_picked+1, :) = q
%             disp("New loop")
%             vertices_picked
%             pause(5)
            % Check if sampled configuration is in collision (will enter
            % loop when condition becomes true. i.e. (true (does not collide) && true (does not belong)
                     
            if ~check_collision(robot, q, link_radius, sphere_centers, sphere_radii) && all(q >= q_min) && all(q <= q_max) %&& ~all(any(q_in_graph == q))
                %Add the sampled configuration to vertices in graph if it is
                %collision free
%                 is_it_empty = isempty(q_in_graph)
%                 pause(5)
                if isempty(q_in_graph) == 1
%                     disp("Entered if")
                    vertices_picked = vertices_picked + 1;
                    vertices(vertices_picked, :) = q;
                    q_in_graph(vertices_picked, :) = q;
%                     is_it_present = all(any(q_in_graph == q))
                elseif all(any(q_in_graph == q)) == 0
%                     disp("Entered elseif")
%                     q
%                     q_in_graph
%                     is_it_present = all(any(q_in_graph == q))
 
                    vertices_picked = vertices_picked + 1;
                    vertices(vertices_picked, :) = q;
                    q_in_graph(vertices_picked, :) = q;
                end
            end
    end
end