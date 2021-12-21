function [P_new, X_new, C_cleaned, F_cleaned, Tau_cleaned] = calculateNewKeypoints(C_temp, R_curr, T_curr, F_temp, Tau_temp, K)
%CALCULATENEWKEYPOINTS Summary of this function goes here
% transform some of the C_temp into P_new (triangulation)

    % check if baseline threshold was passed
    thresh_baseline = 1; %m

    X_new = [];
    P_new= [];
    C_cleaned = C_temp;
    F_cleaned = F_temp;
    Tau_cleaned = Tau_temp;

    % current camera matrix
    intrinsics = getIntrinsicsCam(K);
    camMatrix_curr = cameraMatrix(intrinsics, R_curr, T_curr');

    % get translation vectors from taus to current frame
    motions = T_curr - Tau_temp(10:12, :);

    % project onto the orthogonal vector for which the taus were found, (ignore forward motion)
    % - project onto z vector and then subsctract
    forward_vectors = sum(motions * Tau_temp(7:9, :)) * Tau_temp(7:9, :);
    % get the norm of the columns
    baselines = vecnorm(motions - forward_vectors);

    for i=1:size(baselines, 2)
        if (baselines(i) > thresh_baseline)
            % triangulate 
            rot_matrix_i = reshape(Tau_temp(1:9, i), [3,3]);
            trans_vec_i = Tau_temp(10:12, i); 
            camMatrix_i = cameraMatrix(intrinsics,rot_matrix_i,trans_vec_i);
            worldPoint = triangulate(F_temp(i)',C_temp(i)',camMatrix_i,camMatrix_curr);
            X_new = [X_new, worldPoint'];
            P_new = [P_new, C_temp(i)];
            C_cleaned(:, i) = [];
            F_cleaned(:, i) = [];
            Tau_cleaned(:, i) = [];
        end
    end
end

