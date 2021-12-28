function [P_temp, X_temp] = TrackPreviousPoints(P_prev, X_prev, Frame_prev, Frame_curr)
%TRACKPREVIOUSPOINTS Summary of this function goes here
%   Detailed explanation goes here

P_prev_xy = [P_prev(2,:)', P_prev(1,:)'];

pointTracker = vision.PointTracker('NumPyramidLevels',3);

initialize(pointTracker, P_prev_xy, Frame_prev)

[P_temp_xy, validity_xy] = pointTracker(Frame_curr);

P_temp = [P_prev_xy(:,2)'; P_prev_xy(:,1)'];

validity = validity_xy';

X_temp = X_prev(:,validity);

P_temp = floor(P_temp(:,validity));

%parameters:
%r_T = 15;
%num_iters = 50;
%lambda = 0.1;
%dkp = zeros(size(P_prev));
%keep = true(1, size(P_prev, 2));

%parfor key = 1:size(P_prev, 2)
%        [dkp(:,key), keep(key)] = trackKLTRobustly(Frame_prev, Frame_curr, P_prev(:,key)', r_T, num_iters, lambda);
%end
    
%P_prev = P_prev + dkp;
%P_temp = P_prev(:, keep);
%X_temp = X_prev(:, keep);

end

