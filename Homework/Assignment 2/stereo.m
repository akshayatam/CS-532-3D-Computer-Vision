% Problem 2
% Fish Stereo

clc
clear all
% load variables: BackgroundPointCloudRGB,ForegroundPointCloudRGB,K,crop_region,filter_size)
load data.mat

data3DC = {BackgroundPointCloudRGB,ForegroundPointCloudRGB};
R = eye(3);
move = [0.02 0 0]'; % move in x direction

for step=0:20
    tic
    fname = sprintf('p2new/img%03d.jpg', step);
    fprintf('\nGenerating %s\n',fname);
    t = step * move;
    z = 3.4 + t(3);
    
    % reduced dimension
    % scale to 1/4 of original
    % 400/4 and 640/4
    fx = 100/(0.323 + 0.05) * z;
    fy = 160/(0.053 + 0.55) * z;
    
    % add fx and fy to calibration matrix K
    K(2,2) = fx;
    K(1,1) = fy;

    M           = K*[R t];
    im          = PointCloud2Image(M,data3DC,crop_region,filter_size);
    imwrite(im,fname);

    toc
end

% writing images into video
% reference
% www.mathworks.com/matlabcentral/answers/521001-convert-images-to-video
writerObj = VideoWriter('Video.avi');
writerObj.FrameRate = 15;
open(writerObj);

src = dir('p2new/*.jpg');
for i = 1: length(src)
    filename = strcat('p2new/',src(i).name);
    images = imread(filename);
    frame = im2frame(images);
    writeVideo(writerObj, frame);
end
close(writerObj);