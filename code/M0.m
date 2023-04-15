% Output: q -> 1x4 vector of joint angles

function q = M0()
%     q = [0 -pi/4 0 -pi/4];
% q1 : Rotation about the black link's length in anticlock-wise direction
% q2 : makes the blue link rotate about the axis perpendicular to its length . 0 means it is aligned with the vertical black link; pi/4 means
% rotation about the red cylinder
% q3 : it also rotates the blue link but along its length in anti-clockwise
% direction.
% q4 :Rotates the red link about axis perpendicular to its length 


% Path found with 9 intermediate waypoints:
%          0   -0.7854         0   -0.7854
%          0   -0.7854         0   -0.7854
%     0.3228   -0.8643         0   -1.2849
%     0.2881   -0.9348         0   -1.8798
%    -0.2054   -1.0074         0   -2.2133
%    -0.5934   -1.4320         0   -2.3842
%    -0.5032   -1.9958         0   -2.1999
%     0.0089   -2.3050         0   -2.1535
%     0.0040   -2.6857         0   -2.6172
%    -0.0009   -3.0664         0   -3.0809
%          0   -3.0000         0   -3.0000
% 
% Smoothed path found with 1 intermediate waypoints:
%          0   -0.7854         0   -0.7854
%    -0.2054   -1.0074         0   -2.2133
%          0   -3.0000         0   -3.0000    

q_start = [0 -pi/4 0 -pi/4];
q_goal = [0 -3 0 -3];
% q = [0 0 0 0];
% q = q_goal;
% q = [-0.5934 -1.4320 0 -2.3842]

q = [-0.5032 -1.9958 0 -2.1999];


end