% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function interp_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)
    smoothed_path = path
    % Starting checking from the start configuration
    checking_from = [];
    % Start seeing towards the end configuration initially
    checking_towards = [];
    
    i = 1; %Start checking from the starting position
    while i < size(smoothed_path,1) % Start checking from the ith entry in the path
        checking_from = smoothed_path(i, :);
        j = size(smoothed_path,1); % Start checking from the end config in the path
        while j > i  %Look towards the jth entry in the path
            checking_towards = smoothed_path(j,:);
            % If no collision, check_edge returns false, ~ of it is true |
            % 2nd condition i~= j-1 means that we are not checking the
            % paths next to each other
            if ~check_edge(robot, checking_from, checking_towards, link_radius, sphere_centers, sphere_radii) && i ~= j-1
                % If there is no collision between those two nodes, short
                % those two nodes by a path or delete the paths between
                % them
%                 disp(['Size of smoothed_path: ', num2str(size(smoothed_path,1)), ' | Deleting entries from: ', num2str(i+1), ' to: ', num2str(j-1)]);
                smoothed_path(i+1:j-1, :) = [];
%                 disp(['Size of smoothed_path after deletion: ', num2str(size(smoothed_path,1))]);
                j = size(smoothed_path,1);
            else
                j = j - 1;
            end
        end
        i = i + 1;
    end
    disp(smoothed_path)
    %if smoothingAlgo = CubicSpline
    %     t = 1:size(smoothed_path, 1)
    %     ti = 1:0.2:size(smoothed_path, 1) % Interpolate at 0.1 intervals we can tweak this for smoother and smoother path
    %     interp_path = interp1(t, smoothed_path, ti, 'spline')
    %end
    %elseif smoothingAlgo = PolyInterp
    t = 1:size(smoothed_path, 1)
    ti = 1:0.4:size(smoothed_path, 1) % Interpolate at any intervals

    % Interpolate each joint separately using a polynomial of degree 3 can
    % change it to any degree
    interp_path = zeros(length(ti), size(smoothed_path, 2));
    for j = 1:size(smoothed_path, 2)
        p = polyfit(t, smoothed_path(:,j), 3);
        interp_path(:,j) = polyval(p, ti)
    end
    %end
end
