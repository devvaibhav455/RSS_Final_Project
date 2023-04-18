% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)

function [samples, adjacency] = M2(robot, q_min, q_max, num_samples, sampling_strategy, num_neighbors, link_radius, sphere_centers, sphere_radii)
  
    adjacency = zeros(num_samples, num_samples);
    
    % Need to generate num_samples no. of samples in the final output
    samples = zeros(num_samples, 4);
    samples_picked = 0;

    % Generating 100 samples all of which are valid
    
    q_in_graph = []; %[0 0 0 0; 0 0 0 0]; %any function didn't work as expected when it had only 1 row. So, initialized with 2 rows
    while samples_picked < num_samples
        % Pick up a random index from 1 to 3*num_samples
        rng shuffle;
        %Check if this random index is already picked or not/ pick a unique
        %sample each time
%         fprintf('M2 called M1')
        q = M1(q_min, q_max, 1, sampling_strategy, robot, link_radius, sphere_centers, sphere_radii);
%         fprintf('M2 called M1 end')

%         q1 = q_min(1) + (q_max(1) - q_min(1))*rand();
%         q2 = q_min(2) + (q_max(2) - q_min(2))*rand();
%         q3 = q_min(3) + (q_max(3) - q_min(3))*rand();
%         q4 = q_min(4) + (q_max(4) - q_min(4))*rand();
%         q = [q1 q2 q3 q4];

        % Check if sampled configuration is in collision (will enter
        % loop when condition becomes true. i.e. (true (does not collide) && true (does not belong)
            
        if isempty(q_in_graph) == 1
            q_in_graph(1, :) = q;
        end
            
        %Add the sampled configuration to vertices in graph if it is collision free
        if ~check_collision(robot, q, link_radius, sphere_centers, sphere_radii) && all(q >= q_min) && all(q <= q_max) && ~all(all(q_in_graph == q))
            samples_picked = samples_picked + 1;
%             fprintf('\nSamples picked: %d', samples_picked);
            samples(samples_picked, :) = q;
            q_in_graph(samples_picked, :) = q;
        end
    end
       
    % Local planner
    for i = 1:size(samples,1)
        dist = zeros(size(samples,1), 2); %First column contains distance value, 2nd column contains the index of the neighbor for which distance is being calculated
        for j = 1:size(samples,1)
            %Check for distance of current sample from all other samples
            if j ~= i
                dist(j, 1) = sum(sqrt((samples(i,:) - samples(j,:)).^2) );
                dist(j, 2) = j; %Store the index of the neighbor
            end
        end 
        
        dist(i,:) = []; %Deleting the zero entry for distance when i==j (it is still zero because of zeros function)
        % Finding the minimum num_neighbors + 1 (as it includes 0 distance)
        % with the currently selected sample as well and their index
         
        
%         [IDX, D] = knnsearch(samples, samples(i,:), 'K', num_neighbors+1, 'Distance', 'euclidean')
        % Finding the k nearest neighbors
        sorted_dist_and_indexes = sortrows(dist);
        sorted_dist = sorted_dist_and_indexes(1:num_neighbors,1);
        sorted_dist_indexes = sorted_dist_and_indexes(1:num_neighbors,2);

        for j = 1:length(sorted_dist)
            %Check if the edge between jth neighbor and current sample (ith) is collision free or not
            if ~(check_edge(robot, samples(i,:), samples(sorted_dist_indexes(j)), link_radius, sphere_centers, sphere_radii, 30))
                adjacency(i,sorted_dist_indexes(j)) = sorted_dist(j);
                adjacency(sorted_dist_indexes(j),i) = sorted_dist(j);
            end
        end 
        adjacency(i,i) = 0;
    end
    
end
