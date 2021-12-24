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
keypointsTracker = vision.PointTracker('MaxBidirectionalError', 3);
initialize(keypointsTracker, keypoints_frame1, frame1);
[keypoints_frame2_all, validity] = keypointsTracker(frame2);
keypoints_frame2 =keypoints_frame2_all(validity, :);
keypoints_frame1 =keypoints_frame1(validity, :);


% Estimate pose between the two initial frames
[E,validity] = estimateEssentialMatrix(keypoints_frame1, keypoints_frame2,intrinsics);
[R_init,T_init] = relativeCameraPose(E,intrinsics,keypoints_frame1,keypoints_frame2);
T_WC_init = [R_init,T_init'];
keypoints_frame1 = keypoints_frame1(validity,:);
keypoints_frame2 = keypoints_frame2(validity,:);


% figure(1);
% imshow(frame1);
% hold on;
% plot(keypoints_frame1(:,1), keypoints_frame1(:,2)', 'r.', 'Linewidth', 2);
% hold off

% Triangulate world points
% Triangulate initial landmarks
camMatrix = cameraMatrix(intrinsics,R_init,T_init);
X_init = triangulate(keypoints_frame1,keypoints_frame2,camMatrix,camMatrix);
% X_init = X_init(validity,:);

% Find new candidates and return current pose < S = [P,X,C,F,Tau] >
% frame2, prev_P, prev_C, prev_F, prev_Tau, M, initial
P_curr = keypoints_frame2';
prev_C = [];
prev_F = prev_C;
prev_Tau = prev_C;
[C_curr, F_curr, Tau_curr] = findNewCandidateKeypoints(frame2, keypoints_frame2_all, prev_C, prev_F, prev_Tau, T_WC_init, true);

S_init.P = P_curr;
S_init.X = X_init';
S_init.C = C_curr;
S_init.F = F_curr;
S_init.Tau = Tau_curr;

end
