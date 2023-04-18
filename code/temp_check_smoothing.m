clc;
close all;

% algo_smoothing = 'linear';
%     algo_smoothing = 'poly'; YES
%     algo_smoothing = 'spline'; YES
    algo_smoothing = 'bezier';
%     algo_smoothing = 'bspline';
%     algo_smoothing = 'pchip'; %Piecewise Cubic Hermite Interpolating Polynomial (PCHIP)

% ========== Smoothen the path depending on algo_smoothing  ==========
if smooth_path == 1 && path_found ==1
    % If trajectory is found, smoothen the trajectory
    tic
    smoothed_path = Smooth_path(shortened_path, algo_smoothing);
    fprintf('\n%s: Path Smoothening Time elapsed: %f\n', algo_smoothing, toc)
    % Visualize the smoothed trajectory
    fprintf('%s Smoothed path found with %d intermediate waypoints:\n', algo_smoothing ,size(smoothed_path, 1) - 2);
    disp(smoothed_path);
    fprintf('%s Smoothed: Number in collision: %d\n',algo_smoothing, count_collisions(smoothed_path, robot, link_radius, sphere_centers, sphere_radii, resolution));
    figure(1) %Src: https://www.mathworks.com/matlabcentral/answers/12640-plot-on-different-figures-during-a-loop
    robot.plot(smoothed_path, 'fps', 10);
end

% ========== Plot the original path and the smoothed path  ==========
if smooth_path == 1 && path_found ==1 
    figure(4);
    plot3(path(:,1),path(:,2),path(:,4),"o-")
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