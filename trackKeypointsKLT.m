function [keypoints_frame1,keypoints_frame2] = trackKeypointsKLT(keypoints_frame1, frame1, frame2)

% track keypoints next frame and keep only valid ones
keypointsTracker = vision.PointTracker('MaxBidirectionalError', 3);
initialize(keypointsTracker, keypoints_frame1, frame1);
[keypoints_frame2 ,point_validity] = keypointsTracker(frame2);
keypoints_frame2 =keypoints_frame2(point_validity, :);
keypoints_frame1 =keypoints_frame1(point_validity, :);

end
