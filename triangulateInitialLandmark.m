function X_initial = triangulateInitialLandmark(keypoints_frame1,keypoints_frame2,intrinsics,R_init,T_init)

camMatrix = cameraMatrix(intrinsics,R_init,T_init);
X_initial = triangulate(keypoints_frame1,keypoints_frame2,camMatrix,camMatrix);

X_initial = X_initial';
end



