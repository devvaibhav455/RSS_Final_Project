% You must run startup_rvc FIRST before running this function.
% DO NOT MODIFY THIS FILE!
% Input: questionNum -> Integer between 0 and 6 that denotes question
%                       number to run.
%        samples, adjacency -> Optional pre-computed PRM samples and
%                              adjacency matrix to avoid re-computing it in
%                              question M3
% Output: samples, adjacency -> If PRM roadmap was computed, you can save
%                               it and pass it in on later calls to
%                               hw2_motion to avoid re-computing it

% load samples_adjacency_matrix.mat

function [samples, adjacency] = ex2_motion(samples, adjacency)
    clc;
    close all;
%     startup_rvc;
    
    % Create robot
	robot = create_robot();

    % Start and goal configuration
    q_start = [0 -pi/4 0 -pi/4];
    q_goal = [0 -3 0 -3];
    % Minimum and maximum joint angles for each joint
    q_min = [-pi/2 -pi 0 -pi];
    q_max = [pi/2 0 0 0];
    % Radius of each robot link's cylindrical body
    link_radius = 0.03;
    
    % Set up spherical obstacle
    sphere_center = [0.5 0 0]; % 1x3
    sphere_radius = 0.25;

    r = 0.25;
    sphere_centers = [sphere_center; 0 0.9 0; 0 -0.9 0];
    sphere_radii = [sphere_radius; r; r];
%     draw_sphere(sphere_centers(3,:)', sphere_radii(3));
    for i = 1:size(sphere_centers, 1)
        draw_sphere(sphere_centers(i,:)', sphere_radii(i));
        hold on;
    end


    % Plot robot and obstacle
    robot.plot(q_start);
    hold on;	
%     draw_sphere(sphere_center,sphere_radius);

    sampling_strategy = 'uniform';
%     sampling_strategy = 'gaussian';
%     sampling_strategy = 'bridge';

    algo_path = 'prm';
%     algo_path = 'rrt';

    smooth_path = 1; % 1 : shorten and smooth path using interpolation | 0 don't do path smoothing
    algo_smoothing = 'poly';
%     algo_smoothing = 'spline';
    
    if strcmp(algo_path, 'dijkstra')

    end

    if strcmp(algo_path, 'prm')
        % Parameters for PRM
        num_samples = 100;
        num_neighbors = 10;
        % Construct the roadmap, consisting of
        % configuration samples and weighted adjacency matrix
        % TODO: Implement this function
        if nargin < 2
            fprintf('Calculating adjacency matrix on runtime\n')
            [samples, adjacency] = M2(robot, q_min, q_max, num_samples, sampling_strategy, num_neighbors, link_radius, sphere_center, sphere_radius);
        end
        
%         save("samples_adjacency_matrix.mat", "samples", "adjacency");
        figure;
        % Visualize weighted adjacency matrix
        imshow(adjacency, [min(min(adjacency)), max(max(adjacency))]);

        % Use the roadmap to find a path from q_start to q_goal
        % TODO: Implement this function
        [path, path_found] = M3(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_center, sphere_radius);
        
    end

    


    if strcmp(algo_path, 'astar')

    end

    if strcmp(algo_path, 'rrt')
        % Use the RRT algorithm to find a path from q_start to q_goal
        % TODO: Implement this function
        [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_center, sphere_radius, sampling_strategy);        
    end

    if strcmp(algo_path, 'rrtstar')

    end

    if path_found
        fprintf('%s Path found with %d intermediate waypoints:\n', algo_path , size(path, 1) - 2);
        disp(path);
    else
        disp('No path found.');
    end

    % ========== Shorten the path by removing unnecessary waypoints ==========
    if path_found ==1
        % If trajectory is found, shorten the trajectory
        % TODO: Implement this function
        shortened_path = M5(robot, path, link_radius, sphere_center, sphere_radius);
        % Visualize the smoothed trajectory
        fprintf('Shortened path found with %d intermediate waypoints:\n',size(shortened_path, 1) - 2);
        disp(shortened_path);
%         robot.plot(shortened_path, 'fps', 10);
        %         robot.plot(interpolate_path(smoothed_path), 'fps', 10);
    end

    % ========== Smoothen the path depending on algo_smoothing  ==========
    if smooth_path == 1 && path_found ==1
        % If trajectory is found, smoothen the trajectory
        smoothed_path = Smooth_path(shortened_path, algo_smoothing);
        % Visualize the smoothed trajectory
        fprintf('%s Smoothed path found with %d intermediate waypoints:\n', algo_smoothing ,size(smoothed_path, 1) - 2);
        disp(smoothed_path);
        robot.plot(smoothed_path, 'fps', 10);
    end
end
   

% ========== Detect the number of collisions on the smoothed path  ==========   
%Check if the edge between jth neighbor and current sample (ith) is collision free or not
function num_collisions = count_collisions(path)
    
    for i = 1:size(path,1)
        if ~(check_edge(robot, samples(i,:), samples(sorted_dist_indexes(j)), link_radius, sphere_centers, sphere_radii, 30))

        end
    end
    

end
   

% ========== Helper functions ==========

% Given a path consisting of configuration waypoints,
% interpolates the waypoints to give a less abrupt trajectory.
% Consecutive pairs of waypoints are interpolated along a straight line
% in configuration space, where the newly generated points are
% less than max_velocity apart in configuration space.
% This helper function is primarily used for visualization purposes.
function trajectory = interpolate_path(path, max_velocity)
    if nargin < 2
        max_velocity = 0.05;
    end
    trajectory = [path(1,:)];
    for i = 2:size(path, 1)
        vec = path(i,:) - path(i-1,:);
        num_ticks = ceil(norm(vec) / max_velocity);
        ticks = linspace(0, 1, num_ticks + 1)';
        segment = repmat(path(i-1,:), num_ticks, 1) + repmat(ticks(2:end,:), 1, length(path(i,:))) .* repmat(vec, num_ticks, 1);
        trajectory = [trajectory; segment];
    end
end

% Create a 4-DOF arm with 2 links
function robot = create_robot()
    L(1) = Link([0 0 0 1.571]);
    L(2) = Link([0 0 0 -1.571]);
    L(3) = Link([0 0.4318 0 -1.571]);
    L(4) = Link([0 0 0.4318 1.571]);    
    robot = SerialLink(L, 'name', 'robot');
end

% Draw a sphere with specified center position and radius
function draw_sphere(position, radius)
    [X,Y,Z] = sphere;
    X = X * radius;
    Y = Y * radius;
    Z = Z * radius;
    X = X + position(1);
    Y = Y + position(2);
    Z = Z + position(3);
    surf(X,Y,Z);
end