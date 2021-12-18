function intrinsics = getIntrinsicsCam(K)

focalLength    = [K(1, 1), K(2, 2)]; 
principalPoint = [K(1, 3), K(2, 3)];
imageSize      = [2*principalPoint(1), 2*principalPoint(2)];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);

end