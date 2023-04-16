% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        samples -> num_samples x 4 matrix, vertices in the roadmap
%        adjacency -> num_samples x num_samples matrix, the weighted
%                     adjacency matrix denoting edges in the roadmap
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

function [path, path_found] = M3(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_centers, sphere_radii)

    %Find node which is closest to q_start and q_goal and a path exists
    %between them
    
    % Add q_start and q_goal to the samples and calculate new adjacency
    % matrix in order to find a path between them
    samples_new = samples;
    adjacency_new = adjacency;
    samples_new = [samples_new; q_start; q_goal];
    
    % Starting loop from q_start (2nd last element) till the next element
    for i = size(samples_new,1)-1:size(samples_new,1) 
        % Finding the distance of all the points from q_start
        dist = zeros(size(samples_new,1), 2); %First column contains distance value, 2nd column contains the index of the neighbor for which distance is being calculated
        for j = 1:size(samples_new,1)
            %Check for distance of current sample from all other samples
            if j ~= i
                dist(j, 1) = sum(sqrt((samples_new(i,:) - samples_new(j,:)).^2) );
                dist(j, 2) = j;
            end
        end      
        dist(i,:) = []; %Deleting the zero entry for distance when i==j (it is still zero because of zeros function)
        
        % Finding the minimum num_neighbors + 1 (as it includes 0 distance)
        % with the currently selected sample as well and their index
        sorted_dist_and_indexes = sortrows(dist);
        sorted_dist = sorted_dist_and_indexes(:,1);
        sorted_dist_indexes = sorted_dist_and_indexes(:,2);
        for j = 1:length(sorted_dist)
            %Check if the edge between jth neighbor and current sample (ith) is collision free or not
%             disp(['i: ', num2str(i), ' | j: ', num2str(j)]);
            if ~(check_edge(robot, samples_new(i,:), samples_new(sorted_dist_indexes(j)), link_radius, sphere_centers, sphere_radii,30))
                adjacency_new(i,sorted_dist_indexes(j)) = sorted_dist(j);
                adjacency_new(sorted_dist_indexes(j),i) = sorted_dist(j);
            else
                adjacency_new(i,sorted_dist_indexes(j)) = 0;
                adjacency_new(sorted_dist_indexes(j),i) = 0;
            end
        end
        adjacency_new(i,i) = 0;
    end 

    G = graph(adjacency_new);
    fprintf('Distance between nodes ')
    G.Edges
%     f1 = figure('Name','My Graph' );
    % MATLAB does not show node number if the no. of nodes is greater than
    % 100
%     figure(f1);
%     plot(G,'NodeLabel',1:size(adjacency_new,1)); %Src: https://www.mathworks.com/matlabcentral/answers/290388-why-aren-t-node-labels-numbers-displayed
%     gjfg

    path_indexes = shortestpath(G,size(samples_new,1)-1,size(samples_new,1));
    path_indexes = path_indexes';
    path = zeros(length(path_indexes), 4);
    path_found = false;
    if isempty(path_indexes) == 0
        path_found = true;
        for i = 1:length(path_indexes)
            path(i,:) = samples_new(path_indexes(i),:);
        end
    end
