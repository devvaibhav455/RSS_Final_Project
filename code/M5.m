% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: shortened_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

% IMO, shortened_path is a wrong word to use. The code shortens it by
% removing some unnecessary waypoints. So, in the main file, I have used
% shortened_path
function shortened_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)
    shortened_path = path;
    % Starting checking from the start configuration
    checking_from = [];
    % Start seeing towards the end configuration initially
    checking_towards = [];
    
    i = 1; %Start checking from the starting position
    while i < size(shortened_path,1) % Start checking from the ith entry in the path
        checking_from = shortened_path(i, :);
        j = size(shortened_path,1); % Start checking from the end config in the path
        while j > i  %Look towards the jth entry in the path
            checking_towards = shortened_path(j,:);
            % If no collision, check_edge returns false, ~ of it is true |
            % 2nd condition i~= j-1 means that we are not checking the
            % paths next to each other
            if ~check_edge(robot, checking_from, checking_towards, link_radius, sphere_centers, sphere_radii) && i ~= j-1
                % If there is no collision between those two nodes, short
                % those two nodes by a path or delete the paths between
                % them
%                 disp(['Size of shortened_path: ', num2str(size(shortened_path,1)), ' | Deleting entries from: ', num2str(i+1), ' to: ', num2str(j-1)]);
                shortened_path(i+1:j-1, :) = [];
%                 disp(['Size of shortened_path after deletion: ', num2str(size(shortened_path,1))]);
                j = size(shortened_path,1);
            else
                j = j - 1;
            end
        end
        i = i + 1;
    end
end