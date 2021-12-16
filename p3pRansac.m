function [R_curr, T_curr] = p3pRansac(P_world, P_img, K)
%P3PRANSAC Summary of this function goes here
% get current pose from points and world positions (Jonny)
focalLength    = [K(1, 1), K(2, 2)]; 
principalPoint = [K(1, 3), K(2, 3)];
imageSize      = [2*principalPoint(1), 2*principalPoint(2)];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
%this function solves the perspective-n-point (PnP) problem using the perspective-three-point (P3P) algorithm [1]. 
% The function also eliminates spurious correspondences using the M-estimator sample consensus (MSAC) algorithm.
[R_curr, T_curr] = estimateWorldCameraPose(P_img, P_world, intrinsics);
end

