function keypoints = selectCandidateKeypoints(oldkeypoints, scores, num, r)
% Selects the num best scores as keypoints and performs non-maximum 
% supression of a (2r + 1)*(2r + 1) box around the current maximum.

keypoints = zeros(2, num);
temp_scores = padarray(scores, [r r]);

num_old_keypoints = size(oldkeypoints,2);

for j=1:num_old_keypoints
    kp = [oldkeypoints(1,j); oldkeypoints(2,j)]+r;
    temp_scores(kp(1)-r:kp(1)+r, kp(2)-r:kp(2)+r) = ...
        zeros(2*r + 1, 2*r + 1);
end

for i = 1:num
    [~, kp] = max(temp_scores(:));
    [row, col] = ind2sub(size(temp_scores), kp);
    kp = [row;col];
    keypoints(:, i) = kp - r;
    temp_scores(kp(1)-r:kp(1)+r, kp(2)-r:kp(2)+r) = ...
        zeros(2*r + 1, 2*r + 1);
end

end

