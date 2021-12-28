function [Key_matched, P_3] = InitialKLT(frame_1, frame_3, Key_initial)
%INITIALKLT Summary of this function goes here
%   Detailed explanation goes here

Key_initial_xy = [Key_initial(2,:)', Key_initial(1,:)'];

pointTracker = vision.PointTracker('NumPyramidLevels',3);

initialize(pointTracker, Key_initial_xy, frame_1)

[P_3_xy, validity_xy] = pointTracker(frame_3);

P_3 = [P_3_xy(:,2)'; P_3_xy(:,1)'];

validity = validity_xy';

Key_matched = Key_initial(:,validity);

P_3 = floor(P_3(:,validity));


%parameters:
%r_T = 15;
%num_iters = 50;
%lambda = 0.1;
%dkp = zeros(size(Key_initial));
%keep = true(1, size(Key_initial, 2));


%parfor key = 1:size(Key_initial, 2)
%        [dkp(:,key), keep(key)] = trackKLTRobustly(frame_1, frame_3, Key_initial(:,key)', r_T, num_iters, lambda);
%end
    
%P_3 = Key_initial + dkp;
%Key_matched = Key_initial(:, keep);
%P_3 = P_3(:, keep);
end

