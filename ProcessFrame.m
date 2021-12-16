function [S_curr, Pose_curr] = ProcessFrame(S_prev,  Frame_prev, Frame_curr, K)
%PROCESSFRAME take previous state, previous frame and current frame and
%output current state and current pose.
%   -keypoints traking (KLT)
%   -estimate current camera pose (P3P (DLT refinement)) reject outliers (RANSAC)
% !?-triangulate new points (C, F and Tau as in statements)
%   -eliminate old keypoints and old candidate keypoints ()
%
% S_i [P_i, X_i, C_i, F_i, Tau_i]: State at frame i;
% P_i [2 x Kp]: keypoints in pixel coordinates; 
% X_i [3 x Kp]: 3D Landmarks (relative tp P_i); 
% Kp: num keypoints; 
% C_i [1 x M]: candidate keypoints; 
% F_i [2 x M]: first observation of candidates keypoints; 
% Tau_i [12 x M]: camera pose at first observation of keypoint; 
% M: num candidates; 
% Pose_curr [R|T]: pose with respect to worldframe; 
% R: relative rotation matrix; 
% T: relative translation;

% calculate current transformation from KLT with previous P's and X's
% (Andrea)
% (KLT to find new P-s)
% P_i = [2,3 ; 4,5; 7,8] X_i = [2,3 ; 4,5; 7,8]
% P_i+1 = [4,5 ; 7,9] X_i+1 = [2,3 ; 4,5]
P_temp, X_temp= trackPreviousKeypoints(P_prev, X_prev, Frame_prev, Frame_curr);

% get current pose from points and world positions (Jonny)
[R_curr, T_curr] = p3pRansac(X_temp, P_temp, K);

% track C's from before with KLT and get new coordinates (Andrea)
C_temp, F_temp, Tau_temp = trackCandidateKeypoints(C_prev, F_prev, Tau_prev, Frame_prev, Frame_curr);
 
% transform some of the C_temp into P_new (triangulation)
P_new, X_new, C_cleaned, F_cleaned, Tau_cleaned = calculateNewKeypoints(C_temp, R, T, F_temp, Tau_temp);

% append them to P_temp, and X_temp
P_curr = [P_temp, P_new];
X_curr = [X_temp, X_new];

% find new candidate keypoints, supress ones already tracked (harris,
% supress previous points) (Jeremy)
C_curr, F_curr, Tau_curr = findNewCandidateKeypoints(P_curr, C_cleaned, F_cleaned, Tau_cleaned, Frame_curr, R, T);
end

