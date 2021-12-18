function keypoints_frame1 = findInitialKeypoints(frame_1, K)

num_keypoints_desired = false; 
keypointsInitial = detectHarrisFeatures(frame_1);
if num_keypoints_desired   
    keypointsInitial = keypoints_initial.selectStrongest(num_keypoints_desired);
end
keypoints_frame1 = keypointsInitial.Location;

end

