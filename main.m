
%% Setup
clear all

ds = 2; % 0: KITTI, 1: Malaga, 2: parking
parking_path = './data/parking';
bootstrap_frames = [1 3];

if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/05.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    % Path containing images, depths and all...
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames
if ds == 0
    img0 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end

%% Initialisation

% record poses for plotting later on
Poses = zeros([3, 4, last_frame]);

% Get camera intrinsic parameters object
intrinsics = getIntrinsicsCam(K);

% KLT Keypoint tracker, keep it here since we need for both init and
% process
keypointsTracker = vision.PointTracker('MaxBidirectionalError', 3);

[S,Poses(:, :, bootstrap_frames(2))] = Initialization(img0,img1,K, intrinsics, keypointsTracker);

prev_img = img1; % second image from bootstrapping


%% Continuous operation

range = (bootstrap_frames(2)+1):last_frame;
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
    
    % Update S and Pose for current frame:
    [S, Poses(:, :, i)] = ProcessFrame(S, prev_img, image, K, intrinsics, keypointsTracker);
    
    % get keypoints in pixel coordinates
    keypoints = S.P;
    
    % plot keypoints and image
    subplot(2, 1, 1);
    imshow(image); hold on;
    plot(keypoints(1,:), keypoints(2,:), 'rx', 'Linewidth', 2);
    % %plot(c(1,:), c(2,:), 'gx', 'Linewidth', 2);
    
    subplot(2, 1, 2);
    % plot pose 
    x_pos = Poses(1, 4, (bootstrap_frames(2)+1):i);
    y_pos = Poses(2, 4, (bootstrap_frames(2)+1):i);
    plot(x_pos(:), y_pos(:), 'rx', 'Linewidth', 2);
    hold off;
    
    % Makes sure that plots refresh.    
    pause(0.01);
    
    prev_img = image;
end

