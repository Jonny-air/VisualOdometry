function [C_curr, F_curr, Tau_curr] = findNewCandidateKeyPoints(P_curr, C_cleaned, F_cleaned, Tau_cleaned, Frame_curr, M)
%INITIALIZECANDIDATEPOINTS Summary of this function goes here
%   Detailed explanation goes here
%parameters
corner_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 200;
nonmaximum_supression_radius = 8;

% Harris
harris_scores = harris(Frame_curr, corner_patch_size, harris_kappa);
assert(min(size(harris_scores) == size(harris_scores)));

oldkeypoints = [P_curr, C_cleaned];

keypoints = selectCandidateKeypoints(oldkeypoints, harris_scores, num_keypoints, nonmaximum_supression_radius);

C_curr = [C_cleaned, keypoints];
F_curr = [F_cleaned, keypoints];

Tau_new = ones(1,size(keypoints,2));
Tau_new = kron(Tau_new, M(:));
Tau_curr = [Tau_cleaned, Tau_new];

end

