function S_current = findNewCandidateKeypoints(frame1,frame2,prev_keypoints,keypoints_frame2,X_world, M_current, initial)

% matced keypoints from previous frame
keypointsTracker = vision.PointTracker('MaxBidirectionalError', 3);
initialize(keypointsTracker, prev_keypoints, frame1);
matched_keypoints = keypointsTracker(frame2);
% all keypoints on next frame
next_keypoints = detectHarrisFeatures(frame2);
next_keypoints = next_keypoints.Location;
% only new keypoints 
new_keypoints = setdiff(floor(next_keypoints),floor(matched_keypoints),'rows');

if initial 
    F_current = [];
    Tau_current = [];
end

C_current = new_keypoints';
F_current = cat(2,F_current,C_current); % initialize F_frame2 = []; in main
Tau_current = cat(2,Tau_current,M_current(:));

S_current.P = keypoints_frame2';
S_current.X = X_world';
S_current.C = C_current;
S_current.F = F_current;
S_current.Tau = Tau_current;

end
