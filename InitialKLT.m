function [Key_matched, P_3] = InitialKLT(frame_1, frame_3, Key_initial)
%INITIALKLT Summary of this function goes here
%   Detailed explanation goes here
%parameters:
r_T = 15;
num_iters = 50;
lambda = 0.1;
dkp = zeros(size(Key_initial));
keep = true(1, size(Key_initial, 2));

parfor key = 1:size(Key_initial, 2)
        [dkp(:,key), keep(key)] = trackKLTRobustly(frame_1, frame_3, Key_initial(:,key)', r_T, num_iters, lambda);
end
    
P_3 = Key_initial + dkp;
Key_matched = Key_initial(:, keep);
P_3 = P_3(:, keep);
end

