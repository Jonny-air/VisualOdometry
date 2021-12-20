function keypoints_frame_1 = findInitialKeypoints(frame_1)

corner_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 200;
nonmaximum_supression_radius = 8;

% Harris
harris_scores = harris(frame_1, corner_patch_size, harris_kappa);
assert(min(size(harris_scores) == size(harris_scores)));



keypoints_frame_1 = selectKeypoints(harris_scores, num_keypoints, nonmaximum_supression_radius);

end

