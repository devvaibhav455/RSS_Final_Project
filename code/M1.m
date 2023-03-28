% Input: q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples to sample
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits

function qs = M1(q_min, q_max, num_samples)

    samples_picked = 0;
    qs = zeros(num_samples, 4);
    q_already_selected = zeros(num_samples, 4);
    while samples_picked < num_samples     
        % Src: https://www.mathworks.com/help/matlab/ref/rand.html#buiavoq-9
        q1 = q_min(1) + (q_max(1) - q_min(1))*rand();
        q2 = q_min(2) + (q_max(2) - q_min(2))*rand();
        q3 = q_min(3) + (q_max(3) - q_min(3))*rand();
        q4 = q_min(4) + (q_max(4) - q_min(4))*rand();
        q = [q1 q2 q3 q4];

        if ~all(all(q_already_selected == q)) && all(q >= q_min) && all(q <= q_max)
            samples_picked = samples_picked + 1;
            qs(samples_picked, :) = [q1 q2 q3 q4];
            q_already_selected(samples_picked, :) = q;
        end
    end
end