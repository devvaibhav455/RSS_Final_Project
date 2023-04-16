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

    if strcmp(algo_smoothing, 'bezier')    
        % Src: https://youtu.be/8Jb2f9R6br8 as the reference 
        numControl = size(path,1);
        n = numControl - 1;
        
        i = 0:n;
        coeff = factorial(n)./(factorial(i).*factorial(n-i));
        sbcp = 0.01; % Spacing between control points
        t = 0:sbcp:1;
        

        for j =1:numControl
            b(j,:) = coeff(j)*t.^i(j).*(1-t).^(n-i(j));
        end
        
        q1_bezier = zeros(1,numel(t));
        q2_bezier = zeros(1,numel(t));
        q3_bezier = zeros(1,numel(t));
        q4_bezier = zeros(1,numel(t));
        
        for j = 1:numControl
            q1_bezier = b(j,:)*path(j,1) + q1_bezier;
            q2_bezier = b(j,:)*path(j,2) + q2_bezier;
            q3_bezier = b(j,:)*path(j,3) + q3_bezier;
            q4_bezier = b(j,:)*path(j,4) + q4_bezier;
        end
        size(q1_bezier);
        
        smoothed_path = [q1_bezier', q2_bezier', q3_bezier', q4_bezier'];
        
    end

    if strcmp(algo_smoothing, 'bspline')  
        % Src: https://www.mathworks.com/help/robotics/ref/bsplinepolytraj.html?s_tid=doc_ta#mw_c3cb56b1-ffe5-4071-a655-68cfc0762775
        
        % Create waypoints to interpolate with a B-Spline.
        x = 1:size(path,1);
        
        q1 = [x; path(:,1)'];
        q2 = [x; path(:,2)'];
        q3 = [x; path(:,3)'];
        q4 = [x; path(:,4)'];

        L = length(q1) - 1;
        
        if L == 1
            smoothed_path = path;
            return
        end

        % Form matrices used to compute interior points of control polygon
        r1 = zeros(L+1, size(q1,1));
        r2 = zeros(L+1, size(q2,1));
        r3 = zeros(L+1, size(q3,1));
        r4 = zeros(L+1, size(q4,1));
        A = eye(L+1);
        for i= 1:(L-1)
            A(i+1,(i):(i+2)) = [1 4 1];
            r1(i+1,:) = 6*q1(:,i+1)';
            r2(i+1,:) = 6*q2(:,i+1)';
            r3(i+1,:) = 6*q3(:,i+1)';
            r4(i+1,:) = 6*q4(:,i+1)';
        end

        % Override end points and choose r0 and rL.
        A(2,1:3) = [3/2 7/2 1]; 
        A(L,(L-1):(L+1)) = [1 7/2 3/2]; 
        
        r1(1,:) = (q1(:,1) + (q1(:,2) - q1(:,1))/2)';
        r1(end,:) = (q1(:,end-1) + (q1(:,end) - q1(:,end-1))/2)';

        r2(1,:) = (q2(:,1) + (q2(:,2) - q2(:,1))/2)';
        r2(end,:) = (q2(:,end-1) + (q2(:,end) - q2(:,end-1))/2)';

        r3(1,:) = (q3(:,1) + (q3(:,2) - q3(:,1))/2)';
        r3(end,:) = (q3(:,end-1) + (q3(:,end) - q3(:,end-1))/2)';

        r4(1,:) = (q4(:,1) + (q4(:,2) - q4(:,1))/2)';
        r4(end,:) = (q4(:,end-1) + (q4(:,end) - q4(:,end-1))/2)';
        
        dInterior_q1 = (A\r1)';
        dInterior_q2 = (A\r2)';
        dInterior_q3 = (A\r3)';
        dInterior_q4 = (A\r4)';

        % Construct a complete control polygon and use bsplinepolytraj to compute a polynomial with the new control points
        cpts_q1 = [q1(:,1) dInterior_q1 q1(:,end)];
        cpts_q2 = [q2(:,1) dInterior_q2 q2(:,end)];
        cpts_q3 = [q3(:,1) dInterior_q3 q3(:,end)];
        cpts_q4 = [q4(:,1) dInterior_q4 q4(:,end)];

        t = 0:0.01:1;

        q1_smoothed = bsplinepolytraj(cpts_q1, [0 1], t);
        q2_smoothed = bsplinepolytraj(cpts_q2, [0 1], t);
        q3_smoothed = bsplinepolytraj(cpts_q3, [0 1], t);
        q4_smoothed = bsplinepolytraj(cpts_q4, [0 1], t);

        figure;
        hold all
        plot(q1(1,:), q1(2,:), 'o');
        plot(cpts_q1(1,:), cpts_q1(2,:), 'x-');
        plot(q1_smoothed(1,:), q1_smoothed(2,:));
        legend('Original waypoints', 'Computed control polygon', 'B-spline');
        
        
        smoothed_path = [q1_smoothed(2,:)' ,q2_smoothed(2,:)', q3_smoothed(2,:)', q4_smoothed(2,:)'];
    
    end

    if strcmp(algo_smoothing, 'pchip')    
        t = 1:size(path, 1);
        ti = 1:0.2:size(path, 1); % Interpolate at 0.1 intervals we can tweak this for smoother and smoother path
        smoothed_path = interp1(t, path, ti, 'pchip');
    end


