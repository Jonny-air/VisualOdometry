function [P_temp, X_temp] = TrackPreviousPoints(P_prev, X_prev, Frame_prev, Frame_curr)
%TRACKPREVIOUSPOINTS Summary of this function goes here
%   Detailed explanation goes here

%parameters:
r_T = 15;
num_iters = 50;
lambda = 0.1;
dkp = zeros(size(P_prev));
keep = true(1, size(P_prev, 2));

parfor key = 1:size(P_prev, 2)
        [dkp(:,key), keep(key)] = trackKLTRobustly(Frame_prev, Frame_curr, P_prev(:,key)', r_T, num_iters, lambda);
end
    
P_prev = P_prev + dkp;
P_temp = P_prev(:, keep);
X_temp = X_prev(:, keep);

end

