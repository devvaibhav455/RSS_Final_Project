% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q -> 1x4 vector denoting the configuration to check for collision
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: in_collision -> Boolean, true if the robot at configuration q is
%                         in collision with the given spherical obstacles

function in_collision = check_collision(robot, q, link_radius, sphere_centers, sphere_radii, resolution)
    x1 = [0 0 0]'; %Origin's co-ordinate
    T2 = robot.A(1,q) * robot.A(2,q) * robot.A(3,q); %Calculating some transformation matrix
    x2 = T2.t; % Blue link's end coordinates .tgives the translation component from the homogenous transformation matrix
    T3 = T2 * robot.A(4,q);
    x3 = T3.t; %End-effector tip co-ordinates
     
    if nargin < 6
        resolution = 11;
    end
    ticks = linspace(0, 1, resolution);
    n = length(ticks);
    % 11 equidistant discrete points on the vector joining x1 to x2
    x12 = repmat(x1, 1, n) + repmat(x2 - x1, 1, n) .* repmat(ticks, 3, 1); % 3x11 | Generates a set of discrete points (count = resolution). Each consecutive point has same deltax (final-initial/(resolution-1)), deltay, deltaz. Starting point: x1 (origin); end point: x2 
    % 11 equidistant discrete points on the vector joining x2 to x3
    x23 = repmat(x2, 1, n) + repmat(x3 - x2, 1, n) .* repmat(ticks, 3, 1); % 3x11 | Generates a set of discrete points (count = resolution). Each consecutive point has same deltax, deltay, deltaz. Starting point: x2; end point: x3 
    points = [x12 x23]; % 3x22 | Concatenating x12 and x23 % 2*resolution points. viz. 22 points here
    in_collision = false;
    for i = 1:size(sphere_centers, 1) %Loop through all the obstacles. Here, its just one.
         %LHS condition: calculates the cartesian distance (no square root) between each
         %point in "points" and the spherical obstacle's center
        if any(sum((points - repmat(sphere_centers(i,:)', 1, size(points, 2))).^2, 1) < (link_radius + sphere_radii(i)).^2)
            in_collision = true;
            break;
        end
    end
end