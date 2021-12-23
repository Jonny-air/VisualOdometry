function [C_curr, F_curr, Tau_curr] = findNewCandidateKeypoints(frame2, prev_P, prev_C, prev_F, prev_Tau, M, initial)
% find new candidate keypoints, supress ones already tracked (harris,
% supress previous points) (Jeremy)
% P_curr, C_cleaned, F_cleaned, Tau_cleaned, Frame_curr, M)

    % all keypoints on next frame
    next_keypoints = detectHarrisFeatures(frame2);
    next_keypoints = next_keypoints.Location;

    % only new keypoints 
    [new_keypoints] = setdiff(floor(next_keypoints),floor(prev_P'),'rows');
    if ~initial 
        [new_keypoints] = setdiff(floor(new_keypoints),floor(prev_C'),'rows');
    end
    C_curr = [prev_C, new_keypoints'];
    F_curr = [prev_F, new_keypoints'];
    Tau_curr = [prev_Tau, repmat(M(:), 1, length(new_keypoints))];

end

