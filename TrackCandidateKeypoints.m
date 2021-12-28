function [C_temp, F_temp, Tau_temp] = TrackCandidateKeypoints(C_prev, F_prev, Tau_prev, Frame_prev, Frame_curr)
%TRACKCANDIDATEKEYPOINTS Summary of this function goes here
%   Detailed explanation goes here

C_prev_xy = [C_prev(2,:)', C_prev(1,:)'];

pointTracker = vision.PointTracker('NumPyramidLevels',3);

initialize(pointTracker, C_prev_xy, Frame_prev)

[C_temp_xy, validity_xy] = pointTracker(Frame_curr);

C_temp = [C_temp_xy(:,2)'; C_temp_xy(:,1)'];

validity = validity_xy';

F_temp = F_prev(:,validity);

Tau_temp = Tau_prev(:,validity);

C_temp = floor(C_temp(:,validity));


%parameters:
%r_T = 15;
%num_iters = 50;
%lambda = 0.1;
%dkp = zeros(size(C_prev));
%keep = true(1, size(C_prev, 2));

%parfor key = 1:size(C_prev, 2)
%        [dkp(:,key), keep(key)] = trackKLTRobustly(Frame_prev, Frame_curr, C_prev(:,key)', r_T, num_iters, lambda);
%end
    
%C_prev = C_prev + dkp;
%C_temp = C_prev(:, keep);
%F_temp = F_prev(:, keep);
%Tau_temp = Tau_prev(:, keep);
end

