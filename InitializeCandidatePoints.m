function [C_3, F_3, Tau_3] = InitializeCandidatePoints(frame_2, P_3, M_3)
%INITIALIZECANDIDATEPOINTS Summary of this function goes here
%   Detailed explanation goes here
%parameters
corner_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 100;
nonmaximum_supression_radius = 8;

% Harris
harris_scores = harris(frame_2, corner_patch_size, harris_kappa);
assert(min(size(harris_scores) == size(harris_scores)));

keypoints = selectCandidateKeypoints(P_3, harris_scores, num_keypoints, nonmaximum_supression_radius);

C_3 = keypoints;
F_3 = C_3;
Tau_3 = ones(1,size(C_3,2));
Tau_3 = kron(Tau_3, M_3(:));

end

