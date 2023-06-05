%%%%%%%%%%%%%%%%
% Assignment 3 %
%%%%%%%%%%%%%%%%

% get all projection matrices from 8 cameras
P_0 = [776.649963 -298.408539 -32.048386 993.1581875;
    132.852554 120.885834 -759.210876 1982.174000;
    0.744869 0.662592 -0.078377 4.629312012];
P_1 = [431.503540 586.251892 -137.094040 1982.053375;
    23.799522 1.964373 -657.832764 1725.253500;
    -0.321776 0.869462 -0.374826 5.538025391];
P_2 = [-153.607925 722.067139 -127.204468 2182.4950;
    141.564346 74.195686 -637.070984 1551.185125;
    -0.769772 0.354474 -0.530847 4.737782227];
P_3 = [-823.909119 55.557896 -82.577644 2498.20825;
    -31.429972 42.725830 -777.534546 2083.363250;
    -0.484634 -0.807611 -0.335998 4.934550781];
P_4 = [-715.434998 -351.073730 -147.460815 1978.534875;
    29.429260 -2.156084 -779.121704 2028.892750;
    0.030776 -0.941587 -0.335361 4.141203125];
P_5 = [-417.221649 -700.318726 -27.361042 1599.565000;
    111.925537 -169.101776 -752.020142 1982.983750;
    0.542421 -0.837170 -0.070180 3.929336426];
P_6 = [94.934860 -668.213623 -331.895508 769.8633125;
    -549.403137 -58.174614 -342.555359 1286.971000;
    0.196630 -0.136065 -0.970991 3.574729736];
P_7 = [452.159027 -658.943909 -279.703522 883.495000;
    -262.442566 1.231108 -751.532349 1884.149625;
    0.776201 0.215114 -0.592653 4.235517090];

% read all camera images
cam_0 = imread('cameras/cam00_00023_0000008550.png');
cam_1 = imread('cameras/cam01_00023_0000008550.png');
cam_2 = imread('cameras/cam02_00023_0000008550.png');
cam_3 = imread('cameras/cam03_00023_0000008550.png');
cam_4 = imread('cameras/cam04_00023_0000008550.png');
cam_5 = imread('cameras/cam05_00023_0000008550.png');
cam_6 = imread('cameras/cam06_00023_0000008550.png');
cam_7 = imread('cameras/cam07_00023_0000008550.png');

% read all corresponding silhouettes from cameras
sil_0 = imread('silhouettes/silh_cam00_00023_0000008550.pbm');
sil_1 = imread('silhouettes/silh_cam01_00023_0000008550.pbm');
sil_2 = imread('silhouettes/silh_cam02_00023_0000008550.pbm');
sil_3 = imread('silhouettes/silh_cam03_00023_0000008550.pbm');
sil_4 = imread('silhouettes/silh_cam04_00023_0000008550.pbm');
sil_5 = imread('silhouettes/silh_cam05_00023_0000008550.pbm');
sil_6 = imread('silhouettes/silh_cam06_00023_0000008550.pbm');
sil_7 = imread('silhouettes/silh_cam07_00023_0000008550.pbm');

% create projection matrix, store all above matrices into one
P = zeros(3, 4, 8);
P(:,:,1) = P_0; P(:,:,2) = P_1; 
P(:,:,3) = P_2; P(:,:,4) = P_3; 
P(:,:,5) = P_4; P(:,:,6) = P_5; 
P(:,:,7) = P_6; P(:,:,8) = P_7;

% create camera matrix, store all above matrices into one
cam = zeros(582, 780, 3, 8);
cam(:,:,:,1) = cam_0; cam(:,:,:,2) = cam_1; 
cam(:,:,:,3) = cam_2; cam(:,:,:,4) = cam_3; 
cam(:,:,:,5) = cam_4; cam(:,:,:,6) = cam_5; 
cam(:,:,:,7) = cam_6; cam(:,:,:,8) = cam_7;

% create silhouette matrix, store all above matrices into one
sil = zeros(582, 780, 8);
sil(:,:,1) = sil_0; sil(:,:,2) = sil_1; 
sil(:,:,3) = sil_2; sil(:,:,4) = sil_3; 
sil(:,:,5) = sil_4; sil(:,:,6) = sil_5; 
sil(:,:,7) = sil_6; sil(:,:,8) = sil_7;

% defining voxel grid
vox_x = 5;          % x = (-2.5, 2.5)
vox_y = 6;          % y = (-3, 3)
vox_z = 2.5;        % z = (0, 2.5)

% calculate the volume of voxel
vox_vol = vox_x * vox_y * vox_z;

% initialize number of voxels (10^5 to 10^8)
num_voxels = 100000000;
vox_cr = nthroot(vox_vol/num_voxels,3);

% variable declaration for storing true voxels, surface voxels and
% color matrix and their count
voxels = [];
surface_voxels = [];
RGB_matrix = [];
num_true_voxels = 0;
total_voxels = 0;
surf_vox_var = 0;
prev = [];

tic
% loop through the range created
for x = -vox_x/2:vox_cr:vox_x/2
    for y = -vox_y/2:vox_cr:vox_y/2
        for z = 0:vox_cr:vox_z
            P_arr = [0 0 0 0 0 0 0 0];

            % increment total number of voxels
            total_voxels = total_voxels + 1;
            world_coord = [x y z 1.0].';

            % loop through each projection matrix
            for i = 1:8
                cam_coord = P(:,:,i)*world_coord;
                cam_coord = round(cam_coord/cam_coord(3));

                % condition to add the silhouette value to P_arr
                if (1<=cam_coord(1)) && (cam_coord(1)<=780) && (1<=cam_coord(2)) && (cam_coord(2)<=582)
                    P_arr(i) = sil(cam_coord(2),cam_coord(1),i);
                end
            end

            % check if voxel is the one we need
            if all(P_arr) == 1 
                % add to the true voxels and add to the matrix for output
                num_true_voxels = num_true_voxels + 1;
                voxels = [voxels;[x y z]];

                % get RGB values from last image
                R = cam(cam_coord(2), cam_coord(1), 1, 8);
                G = cam(cam_coord(2), cam_coord(1), 2, 8);
                B = cam(cam_coord(2), cam_coord(1), 3, 8);

                % add the RGB value to the matrix used for output
                RGB_matrix = [RGB_matrix;[R G B]];
                
                % condition to detect surface voxels
                % discard voxels that are not surface voxels
                if surf_vox_var == 0
                    % similar to above condition, but here for 
                    % surface voxels
                    surface_voxels = [surface_voxels;[x y z]];
                    surf_vox_var = surf_vox_var+1;
                    prev = [x y z];
                    continue;
                end
                if (prev(1)==x) && (prev(2)==y)
                    surf_vox_var = surf_vox_var+1;
                else
                    if surf_vox_var > 1
                        surface_voxels = [surface_voxels;prev;[x y z]];
                        tmp=1;
                    else
                        surface_voxels = [surface_voxels;[x y z]];
                        tmp=1;
                    end
                end
                if surf_vox_var > 0
                    prev = [x y z];
                end
            end
        end
    end
end
% generate output (voxel point cloud without color)
voxels_pc = pointCloud(voxels);
pcwrite(voxels_pc,'output/10^8 voxels/voxel_pc_noRGB','PLYFormat','ascii');

% generate output (voxel point cloud)
voxels_pc.Color = uint8(RGB_matrix);
pcwrite(voxels_pc,'output/10^8 voxels/voxel_pc','PLYFormat','ascii');

% generate output (surface voxel point cloud)
surface_voxels_pc = pointCloud(surface_voxels);
pcwrite(surface_voxels_pc,'output/10^8 voxels/surface_voxels_pc','PLYFormat','ascii');

toc
