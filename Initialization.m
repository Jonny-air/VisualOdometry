function [S_init,T_WC_init] = Initialization(frame1,frame2, K, intrinsics, keypointsTracker)
%INITIALIZATION Initialize the states and pose (with respect to worldframe (1st frame)) using 2 initial frames (1st and 3rd suggested)
%   -keypoints detection (harris) and matching (KLT) 
%   -Calculate relative pose (8-points or 5-points) and triangulate 3D point cloud and outlier rejection (RANSAC)
% Kp: keypoints count; 
% P_0 [2 x Kp]: keypoints;
% X_0 [3 x Kp]: 3D Landmarks (relative tp P0); 
% Pose [R|T]: pose with respect to worldframe; 
% R: relative rotation matrix (to express point from frame 2 to frame 1); 
% T: relative translation (to express point from frame 2 to frame 1);

% Find keypoints in first frame
keypoints_frame1 = findInitialKeypoints(frame1);

% use KLT to track keypoints to next frame
% initializes keypoint tracker
[keypoints_frame1,keypoints_frame2] = trackKeypointsKLT(keypoints_frame1, frame1, frame2, keypointsTracker);

% Estimate pose between the two initial frames
[R_init,T_init,idx_validity] = poseP3PRansac(keypoints_frame1,keypoints_frame2,K,intrinsics);
T_WC_init = [R_init,T_init'];
keypoints_frame1 = keypoints_frame1(idx_validity,:);
keypoints_frame2 = keypoints_frame2(idx_validity,:);

figure(1);
imshow(frame1);
hold on;
plot(keypoints_frame1(:,1), keypoints_frame1(:,2)', 'r.', 'Linewidth', 2);
%plot(c(1,:), c(2,:), 'gx', 'Linewidth', 2);
hold off

% Triangulate world points
X_initial = triangulateInitialLandmark(keypoints_frame1,keypoints_frame2,intrinsics,R_init,T_init);

% Find new candidates and return current pose < S = [P,X,C,F,Tau] >
% frame2, prev_P, prev_C, prev_F, prev_Tau, M, initial
P_curr = keypoints_frame2';
prev_C = [];
prev_F = prev_C;
prev_Tau = prev_C;
[C_curr, F_curr, Tau_curr] = findNewCandidateKeypoints(frame2, P_curr, prev_C, prev_F, prev_Tau, T_WC_init, true);

S_init.P = P_curr;
S_init.X = X_initial';
S_init.C = C_curr;
S_init.F = F_curr;
S_init.Tau = Tau_curr;

end
