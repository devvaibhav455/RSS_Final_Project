% Input: q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples to sample
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits

% NORMAL DISTRIBUTION ALSO NEEDS TO BE WITHIN BOUNDS
% TODO: https://www.mathworks.com/matlabcentral/answers/619368-obtain-a-random-number-from-a-truncated-normal-distribution
function qs = M1(q_min, q_max, num_samples, sampling_strategy, robot, link_radius, sphere_centers, sphere_radii)
%     fprintf('Entered inside M1');
    
    samples_picked = 0;
    qs = zeros(num_samples, 4);
    q_already_selected = zeros(num_samples, 4);

    sigma = 0.75*(q_max - q_min); % Using 1/6th of the range as standard deviation)
    while samples_picked < num_samples     
        % Src: https://www.mathworks.com/help/matlab/ref/rand.html#buiavoq-9
        
        q1 = q_min(1) + (q_max(1) - q_min(1))*rand();
        q2 = q_min(2) + (q_max(2) - q_min(2))*rand();
        q3 = q_min(3) + (q_max(3) - q_min(3))*rand();
        q4 = q_min(4) + (q_max(4) - q_min(4))*rand();         
        
        if strcmp(sampling_strategy, 'gaussian')
%             q1 = q_min(1) + (q_max(1) - q_min(1))*rand();
            q1_dash = normrnd(q1,sigma(1)); 
            
%             q2 = q_min(2) + (q_max(2) - q_min(2))*rand();
            q2_dash = normrnd(q2,sigma(2));
            
%             q3 = q_min(3) + (q_max(3) - q_min(3))*rand();
            q3_dash = normrnd(q3,sigma(3));

%             q4 = q_min(4) + (q_max(4) - q_min(4))*rand();
            q4_dash = normrnd(q4,sigma(4));

            q_dash = [q1_dash q2_dash q3_dash q4_dash];

        elseif strcmp(sampling_strategy, 'bridge')
%             q1 = q_min(1) + (q_max(1) - q_min(1))*rand();
            q1_dash = normrnd(q1,sigma(1));
            q1_ddash = (q1 + q1_dash)/2;
            
%             q2 = q_min(2) + (q_max(2) - q_min(2))*rand();
            q2_dash = normrnd(q2,sigma(2));
            q2_ddash = (q2 + q2_dash)/2;
            
%             q3 = q_min(3) + (q_max(3) - q_min(3))*rand();
            q3_dash = normrnd(q3,sigma(3));
            q3_ddash = (q3 + q3_dash)/2;

%             q4 = q_min(4) + (q_max(4) - q_min(4))*rand();
            q4_dash = normrnd(q4,sigma(4));
            q4_ddash = (q4 + q4_dash)/2;

            q_dash = [q1_dash q2_dash q3_dash q4_dash];
            q_ddash = [q1_ddash q2_ddash q3_ddash q4_ddash];
        end
        
        q = [q1 q2 q3 q4];
        
        % q_picked contains the sample which is picked
        % Check if uniform random sampling config is free or not
        if strcmp(sampling_strategy, 'gaussian')
            
            %check_collision will return true if it is in collision
            is_q_colliding  = check_collision(robot, q, link_radius, sphere_centers, sphere_radii); 
            is_q_dash_colliding  = check_collision(robot, q_dash, link_radius, sphere_centers, sphere_radii); 

            if (is_q_colliding == true && is_q_dash_colliding == false)
                q_selected = q_dash;
            elseif (is_q_colliding == false && is_q_dash_colliding == false)
                continue
            end

        elseif strcmp(sampling_strategy, 'bridge')
            %check_collision will return true if it is in collision
            is_q_colliding  = check_collision(robot, q, link_radius, sphere_centers, sphere_radii)
            is_q_dash_colliding  = check_collision(robot, q_dash, link_radius, sphere_centers, sphere_radii)
            is_q_ddash_colliding  = check_collision(robot, q_ddash, link_radius, sphere_centers, sphere_radii)

            if ((is_q_colliding && is_q_dash_colliding) == true) && (is_q_ddash_colliding == false)
%                 fprintf('\nBridge sample found')    
                q_selected = q_ddash;
            else
%                 fprintf('\nBridge condition not matched. Trying next iteration')
                continue
            end
        end
        
%         fprintf('\nTrying to get samples')
%         q
%         q_already_selected
%         ~all(all(q_already_selected == q)) && all(q >= q_min) && all(q <= q_max)

        if ~all(all(q_already_selected == q_selected)) && all(q >= q_min) && all(q_selected <= q_max)
            q
            q_dash
            q_ddash
            
            figure(5);
            hold on
            plot3(q(:,1),q(:,2),q(:,4),"^")
            hold on
            plot3(q_dash(:,1),q_dash(:,2),q_dash(:,4),"square")
            hold on
            plot3(q_ddash(:,1),q_ddash(:,2),q_ddash(:,4),"diamond")
            hold on
            samples_picked = samples_picked + 1;
%             qs(samples_picked, :) = [q1 q2 q3 q4];
            qs(samples_picked, :) = q_selected;
            q_already_selected(samples_picked, :) = q_selected;
            pause(5000);
%             fprintf('\nPicked bridge sample because its unique');
%             fprintf('\nSamples picked: %d', samples_picked);
        end
         
%         samples_picked = samples_picked + 1;
%         qs(samples_picked, :) = q;
%         qs(samples_picked, :) = [q1 q2 q3 q4];
        
    end
    
end