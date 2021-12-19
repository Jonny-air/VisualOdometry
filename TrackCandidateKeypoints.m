function [C_temp, F_temp, Tau_temp] = TrackCandidateKeypoints(C_prev, F_prev, Tau_prev, Frame_prev, Frame_curr)
%TRACKCANDIDATEKEYPOINTS Summary of this function goes here
%   Detailed explanation goes here

%parameters:
r_T = 15;
num_iters = 50;
lambda = 0.1;
dkp = zeros(size(C_prev));
keep = true(1, size(C_prev, 2));

parfor key = 1:size(C_prev, 2)
        [dkp(:,key), keep(key)] = trackKLTRobustly(Frame_prev, Frame_curr, C_prev(:,key)', r_T, num_iters, lambda);
end
    
C_prev = C_prev + dkp;
C_temp = C_prev(:, keep);
F_temp = F_prev(:, keep);
Tau_temp = Tau_prev(:, keep);
end

