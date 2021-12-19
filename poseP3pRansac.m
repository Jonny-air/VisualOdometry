function [R_init,T_init,idx_validity] = poseP3pRansac(keypoints_frame1,keypoints_frame2,K,intrinsics)
% [keypoints_frame1 ones(length(keypoints_frame1),1)]'
% [keypoints_frame2 ones(length(keypoints_frame2),1)]'
[E,idx_validity] = estimateEssentialMatrix(keypoints_frame1, keypoints_frame2,intrinsics);
[R_init,T_init] = relativeCameraPose(E,intrinsics,keypoints_frame1,keypoints_frame2);

end

