function [R_curr, T_curr, inlierIdx] = p3pRansac(P_world, P_img, intrinsics)
%P3PRANSAC Summary of this function goes here
% get current pose from points and world positions (Jonny)
% this function solves the perspective-n-point (PnP) problem using the perspective-three-point (P3P) algorithm [1]. 
% The function also eliminates spurious correspondences using the M-estimator sample consensus (MSAC) algorithm.
[R_curr, T_temp, inlierIdx] = estimateWorldCameraPose(P_img', P_world', intrinsics, 'Confidence', 95, 'MaxReprojectionError', 2);
T_curr = T_temp';
end

