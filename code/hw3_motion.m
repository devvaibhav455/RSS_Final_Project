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
%     startup_rvc;

clc;
clear all;
close all;
% load samples_adjacency_matrix_new.mat

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

resolution = 11;
num_samples = 100;
num_neighbors = 10;

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



path_found = 0;

sampling_strategy = 'uniform';
%     sampling_strategy = 'gaussian';
%     sampling_strategy = 'bridge';

% sampling_strategy_list = ["uniform" "gaussian" "bridge" ];
sampling_strategy_list = ["bridge"];
% algo_path_list = ["astar"];

%     algo_path = 'prm';
%     algo_path = 'rrt';
%     algo_path = 'dijkstra';
%     algo_path = 'rrtstar';
%     algo_path = 'astar';

algo_path_list = ["prm" "rrt" "dijkstra" "rrtstar" "astar"];


    smooth_path = 1; % 1 : shorten and smooth path using interpolation | 0 don't do path smoothing
%     algo_smoothing = 'linear';
%     algo_smoothing = 'poly';
%     algo_smoothing = 'spline';
%     algo_smoothing = 'bezier';
%     algo_smoothing = 'bspline';
%     algo_smoothing = 'pchip'; %Piecewise Cubic Hermite Interpolating Polynomial (PCHIP)

algo_smoothing_list = ["linear" "poly" "spline" "bezier" "bspline" "pchip"];
num_iterations = 10;

% fprintf(fileID,'sampling_strategy,algo_path,smooth_path,algo_smoothing\n');

fileID = fopen('exp1.txt','w');
fclose(fileID);
for a = 1:numel(sampling_strategy_list)
    sampling_strategy = sampling_strategy_list(a);
    
    % Generate samples using the sampling_strategy
    tic
    [samples, adjacency] = M2(robot, q_min, q_max, num_samples, sampling_strategy, num_neighbors, link_radius, sphere_center, sphere_radius);
    toc
    name_sampling_adjacency = sprintf('%s_sampling_adjacency.mat', sampling_strategy);
%     save(name_sampling_adjacency, "samples", "adjacency");
%     load bridge_sampling_adjacency.mat

    for b = 1:numel(algo_path_list)
        algo_path = algo_path_list(b);
        fprintf('sampling_strategy: %s | algo_path: %s\n',sampling_strategy, algo_path)
        fileID = fopen('exp1.txt','a');
        fprintf(fileID,'sampling_strategy: %s | algo_path: %s\n',sampling_strategy, algo_path  );
%         fprintf(fileID,'%s,%s\n',,algo_path);
        fprintf(fileID,'Runtime_algo_path,Path_Length\n');
        

        % ========== START WRITING THE VALUES TO TEXT FILE for each iteration ==========
        for iter = 1:num_iterations
            fprintf('Iteration number: %d\n',iter);

            % ========== Use Dijkstra to find path ==========
            if strcmp(algo_path, 'dijkstra')
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
            
                tic
                [dist, path_indexes] = Dijkstra(adjacency_new, size(adjacency_new,1)-1, size(adjacency_new,1));
                runtime_algo_path = toc;
%                 fprintf('\n%s: Time elapsed: %f\n', algo_path, toc)
            
                if size(path_indexes,1) > 1
                    path_found = 1;
                    path = zeros(size(path_indexes,1), 4);
                    %Find out the values of qs from indices.
                    for i = 1:length(path_indexes)
                        path(i,:) = samples_new(path_indexes(i),:);
                    end
                end
            end
            
            % ========== Use PRM to find path ==========
            if strcmp(algo_path, 'prm')
                % Parameters for PRM
                
                % Construct the roadmap, consisting of
                % configuration samples and weighted adjacency matrix
                % TODO: Implement this function
%                 if nargin < 2
%                     fprintf('Calculating adjacency matrix on runtime\n')
%                     [samples, adjacency] = M2(robot, q_min, q_max, num_samples, sampling_strategy, num_neighbors, link_radius, sphere_center, sphere_radius);
%                 end
                
            %         save("samples_adjacency_matrix.mat", "samples", "adjacency");
            %         figure;
                % Visualize weighted adjacency matrix
            %         imshow(adjacency, [min(min(adjacency)), max(max(adjacency))]);
            
                % Use the roadmap to find a path from q_start to q_goal
                % TODO: Implement this function
                tic
                [path, path_found] = M3(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_center, sphere_radius);
                runtime_algo_path = toc;
%                 fprintf('\n%s: Time elapsed: %f\n', algo_path, toc)
            end
            
            
            
            % ========== Use AStar to find path ==========
            if strcmp(algo_path, 'astar')
                % Tic started within function
                [path, path_found] = M4_AStar(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_center, sphere_radius, sampling_strategy, num_samples, num_neighbors,samples, adjacency);
                runtime_algo_path = toc;
%                 fprintf('\n%s: Time elapsed: %f\n', algo_path, toc)
            end
            
            % ========== Use RRT to find path ==========
            if strcmp(algo_path, 'rrt')
                % Use the RRT algorithm to find a path from q_start to q_goal
                % TODO: Implement this function
                tic
                [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_center, sphere_radius, sampling_strategy);        
                runtime_algo_path = toc;
%                 fprintf('\n%s: Time elapsed: %f\n', algo_path, toc)
            end
            
            % ========== Use RRT* to find path ==========
            if strcmp(algo_path, 'rrtstar')
                tic
                [path, path_found] = RRTStar(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_center, sphere_radius, sampling_strategy);        
                runtime_algo_path = toc;
%                 fprintf('\n%s: Time elapsed: %f\n', algo_path, toc)
            end

            fprintf(fileID,'%f,%d\n', runtime_algo_path, size(path, 1));
            name_algo_path = sprintf('path_%s_%s.mat', sampling_strategy ,algo_path);
            save(name_algo_path, "path", "path_found");
        end
    fclose(fileID);    
    end
end
  



% ========== Display the number of points in the path by the path finding algorithm ==========
if path_found == 1
    fprintf('%s Path found with %d intermediate waypoints:\n', algo_path , size(path, 1) - 2);
    disp(path);
else
    disp('No path found.');
end

%Plot all the paths from each algorithm with uniform sampling and try to
%find a path which does not look smooth. Most probably, it will be RRT*.
%Choose this path and compare various path smoothing algorithms.

% load PATH FROM UNIFORM SAMPLING

algo_smoothing_list = ["linear" "spline" "bezier" "bspline" "pchip"]
fileID = fopen('exp_smoothing.txt','w');
fclose(fileID);

% for a = 1:numel(algo_smoothing_list)
%     algo_smoothing = algo_smoothing_list(a);

%%

% load path_uniform_rrtstar.mat
clc;

algo_path = 'rrtstar';

%     algo_smoothing = 'linear';
%     algo_smoothing = 'poly';
%     algo_smoothing = 'spline';
%     algo_smoothing = 'bezier';
%     algo_smoothing = 'bspline';
    algo_smoothing = 'pchip'; %Piecewise Cubic Hermite Interpolating Polynomial (PCHIP)


% ========== Shorten the path by removing unnecessary waypoints ==========
if path_found ==1
    % If trajectory is found, shorten the trajectory
    tic
    shortened_path = M5(robot, path, link_radius, sphere_center, sphere_radius);
    fprintf('\nPath Shortening: Time elapsed: %f\n', toc)
    fprintf('Shortened path found with %d intermediate waypoints:\n',size(shortened_path, 1) - 2);
    %disp(shortened_path);
end

% ========== Smoothen the path depending on algo_smoothing  ==========
if smooth_path == 1 && path_found ==1
    % If trajectory is found, smoothen the trajectory
    tic
    smoothed_path = Smooth_path(shortened_path, algo_smoothing);
    fprintf('\n%s: Path Smoothening Time elapsed: %f\n', algo_smoothing, toc)
    % Visualize the smoothed trajectory
    fprintf('%s Smoothed path found with %d intermediate waypoints:\n', algo_smoothing ,size(smoothed_path, 1) - 2);
    %disp(smoothed_path);
    fprintf('%s Smoothed: Number in collision: %d\n',algo_smoothing, count_collisions(smoothed_path, robot, link_radius, sphere_centers, sphere_radii, resolution));
    fprintf('%s Smoothed: Percent points in collision: %f\n',algo_smoothing, 100*count_collisions(smoothed_path, robot, link_radius, sphere_centers, sphere_radii, resolution)/size(smoothed_path, 1));
    figure(1) %Src: https://www.mathworks.com/matlabcentral/answers/12640-plot-on-different-figures-during-a-loop
%     robot.plot(smoothed_path, 'fps', 10);
end

% ========== Plot the original path and the smoothed path  ==========
if smooth_path == 1 && path_found ==1 
    figure(4);
    plot3(path(:,1),path(:,2),path(:,4),"o-")
    grid on
    xlabel('q1')
    ylabel('q2')
    zlabel('q4')
%     xlim([-3*pi/2 3*pi/2])
    xlim("tight")
    ylim("tight")
    zlim("tight")

%     ylim([-1.5*pi 0.1])
%     zlim([-1.5*pi 0.1])

    txt_start = '\leftarrow START';
    txt_goal = '\leftarrow GOAL';
    text(q_start(1),q_start(2),q_start(4),txt_start)
    text(q_goal(1),q_goal(2),q_goal(4),txt_goal)
    
    hold on
    plot3(shortened_path(:,1),shortened_path(:,2),shortened_path(:,4),"x-")
    hold on
    plot3(smoothed_path(:,1),smoothed_path(:,2),smoothed_path(:,4),"-.")
    name_opath = sprintf('%s: Original Path', algo_path);
    name_smoothed_path = sprintf('%s: Smoothed Path', algo_smoothing);
    legend(name_opath,'Shortened path',name_smoothed_path)
    hold off
end
  

% ========== Detect the number of collisions on the smoothed path  ==========   
%Check if the edge between jth neighbor and current sample (ith) is collision free or not
function num_collision = count_collisions(path, robot, link_radius, sphere_centers, sphere_radii, resolution)
    num_collision = 0;
    for i = 1:size(path,1)
        q = path(i,:);
        if check_collision(robot, q, link_radius, sphere_centers, sphere_radii, resolution)
                num_collision = num_collision + 1;
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
    figure(1)
    surf(X,Y,Z);
end