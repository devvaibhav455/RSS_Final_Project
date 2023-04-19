% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
% Output: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise

function cspace = gen_config_space(robot, q_min, q_max, link_radius, sphere_centers, sphere_radii)
    cspace_resolution = 100;
    q1_grid = linspace(q_min(1), q_max(1), cspace_resolution);
    q2_grid = linspace(q_min(2), q_max(2), cspace_resolution);
    q4_grid = linspace(q_min(4), q_max(4), cspace_resolution);

    cspace = zeros(size(q1_grid,2),size(q1_grid,2),size(q1_grid,2));
%     plot_obstacles(obstacles);
    %Loop for q1
    for i = 1:length(q1_grid)
        %Loop for q2
        for j = 1:length(q2_grid)
            for k = 1:length(q4_grid)
                q = [q1_grid(i) q2_grid(j) 0 q4_grid(k)];
                %Check if the links intersect with all/ any of the obstacles
                if check_collision(robot, q, link_radius, sphere_centers, sphere_radii) == false && all(q >= q_min) && all(q <= q_max)
                    cspace(i,j,k) = 0;
                else
                    cspace(i,j,k) = 1;
                end
%                 fprintf("k: %d", k)
            end
%         fprintf("\nk: %d", k)
        end
    fprintf("\ni: %d", i)
    end
end