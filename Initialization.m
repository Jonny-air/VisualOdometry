function [S_3,Pose_3] = Initialization(frame_1,frame_2, K, intrinsics)
%INITIALIZATION Initialize the states and pose (with respect to worldframe (1st frame)) using 2 initial frames (1st and 3rd suggested)
%   -keypoints detection (harris) and matching (KLT) 
%   -Calculate relative pose (8-points or 5-points) and triangulate 3D point cloud and outlier rejection (RANSAC)
% Kp: keypoints count; 
% P_0 [2 x Kp]: keypoints;
% X_0 [3 x Kp]: 3D Landmarks (relative tp P0); 
% Pose [R|T]: pose with respect to worldframe; 
% R: relative rotation matrix (to express point from frame 2 to frame 1); 
% T: relative translation (to express point from frame 2 to frame 1);

% Find keypoints in first frame (use function from process frame, but don't
% supress anything) (Jeremy)
Key_initial = findInitialKeypoints(frame_1);

% use KLT to track keypoints to third frame (Andrea)
[Key_matched, P_3] = InitialKLT(frame_1, frame_2, Key_initial);


% Do 8 point RANSAC with keypoint pairs
%%%%%%TODO
[E,validity] = estimateEssentialMatrix(Key_matched, P_3,intrinsics);
[R_init,T_init] = relativeCameraPose(E,intrinsics,keypoints_frame1,keypoints_frame2);
M_3 = [R_init, T_init'];


% triangulate X'sm M_0 is 0
X_3 = triangulateInitialLandmarks(Key_initial, Key_matched, M_3, P_3, intrinsics);
    
% find keypoints in frame 2 (supress P's) --> (Jeremy)
[C_3, F_3, Tau_3] = InitializeCandidatePoints(frame_2, P_3, M_3);

S_3 = {P_3, X_3, C_3, F_3, Tau_3};
Pose_3 = M_3;

end

