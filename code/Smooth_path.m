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

function smoothed_path = M5(path, algo_smoothing)
    
    if strcmp(algo_smoothing, 'poly')    
        t = 1:size(path, 1);
        ti = 1:0.4:size(path, 1); % Interpolate at any intervals
        degree = size(path, 1) - 1;
    
        % Interpolate each joint separately using a polynomial of degree 3 can
        % change it to any degree
        smoothed_path = zeros(length(ti), size(path, 2));
        for j = 1:size(path, 2)
            p = polyfit(t, path(:,j), degree);
            smoothed_path(:,j) = polyval(p, ti);
        end
    end
    
    if strcmp(algo_smoothing, 'spline')    
        t = 1:size(path, 1);
        ti = 1:0.2:size(path, 1); % Interpolate at 0.1 intervals we can tweak this for smoother and smoother path
        smoothed_path = interp1(t, path, ti, 'spline');
    end


