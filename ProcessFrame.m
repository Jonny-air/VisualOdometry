function [S_curr, Pose_curr] = ProcessFrame(S_prev,  Frame_prev, Frame_curr)
%PROCESSFRAME take previous state, previous frame and current frame and
%output current state and current pose.
%   -keypoints traking (KLT)
%   -estimate current camera pose (P3P (DLT refinement)) reject outliers (RANSAC)
% !?-triangulate new points (C, F and Tau as in statements)
%   -eliminate old keypoints and old candidate keypoints ()
%
% S_i [P_i, X_i, C_i, F_i, Tau_i]: State at frame i;
% P_i [2 x Kp]: keypoints; 
% X_i [3 x Kp]: 3D Landmarks (relative tp P_i); 
% Kp: num keypoints; 
% C_i [1 x M]: candidate keypoints; 
% F_i [2 x M]: first observation of candidates keypoints; 
% Tau_i [12 x M]: camera pose at first observation of keypoint; 
% M: num candidates; 
% Pose_curr [R|T]: pose with respect to worldframe; 
% R: relative rotation matrix; 
% T: relative translation;


end

