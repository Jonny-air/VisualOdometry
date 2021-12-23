function [tracked_points, validity_idx] = TrackPreviousPoints(keypoints, frame2, keypointsTracker)
%TRACKPREVIOUSPOINTS Summary of this function goes here
%   Detailed explanation goes here

% track keypoints next frame and keep only valid ones
setPoints(keypointsTracker, keypoints')
[tracked_points_transp, validity_idx]= keypointsTracker(frame2);

tracked_points = tracked_points_transp(validity_idx, :)';
%parameters:
% r_T = 15;
% num_iters = 50;
% lambda = 0.1;
% dkp = zeros(size(P_prev));
% keep = true(1, size(P_prev, 2));
% 
% parfor key = 1:size(P_prev, 2)
%         [dkp(:,key), keep(key)] = trackKLTRobustly(Frame_prev, Frame_curr, P_prev(:,key)', r_T, num_iters, lambda);
% end
%     
% P_prev = P_prev + dkp;
% P_temp = P_prev(:, keep);
% X_temp = X_prev(:, keep);

end

