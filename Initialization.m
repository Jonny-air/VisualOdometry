function [S_init,T_WC_init] = Initialization(frame1,frame2, K)
%INITIALIZATION Initialize the states and pose (with respect to worldframe (1st frame)) using 2 initial frames (1st and 3rd suggested)
%   -keypoints detection (harris) and matching (KLT) 
%   -Calculate relative pose (8-points or 5-points) and triangulate 3D point cloud and outlier rejection (RANSAC)
% Kp: keypoints count; 
% P_0 [2 x Kp]: keypoints;
% X_0 [3 x Kp]: 3D Landmarks (relative tp P0); 
% Pose [R|T]: pose with respect to worldframe; 
% R: relative rotation matrix (to express point from frame 2 to frame 1); 
% T: relative translation (to express point from frame 2 to frame 1);

% Get camera intrinsic parameters object
intrinsics = getIntrinsicsCam(K);

% Find keypoints in first frame
keypoints_frame1 = findInitialKeypoints(frame1, K);

% use KLT to track keypoints to next frame
[keypoints_frame1,keypoints_frame2] = trackKeypointsKLT(keypoints_frame1, frame1, frame2);

% Estimate pose between the two initial frames
[R_init,T_init] = poseP3pRansac(keypoints_frame1,keypoints_frame2,K,intrinsics);
T_WC_init = [R_init,T_init'];


% Triangulate world points
X_initial = triangulateInitialLandmark(keypoints_frame1,keypoints_frame2,intrinsics,R_init,T_init);

% Find new candidates and return current pose < S = [P,X,C,F,Tau] >
S_init = findNewCandidateKeypoints(frame1,frame2,keypoints_frame1,keypoints_frame2,X_initial, T_WC_init, true);

end
