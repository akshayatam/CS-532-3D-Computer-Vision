import xml.etree.ElementTree as ET
import numpy as np
import os
import open3d as o3d
from PIL import Image

CALIBRATION_DIR = 'calibration/'
CAMERA_DIR = 'cameras/'
SILHOUETTE_DIR = 'silhouettes/'
OUTPUT_DIR = 'output/'

def voxel_reconstruction(P, cam, sil, num_voxels):
    print(f"Voxel size: {num_voxels}\n")
    # voxel grid
    vox_x = 5
    vox_y = 6
    vox_z = 2.5

    # voxel volume
    vox_vol = vox_x * vox_y * vox_z

    # voxel count
    #num_voxels = 1000000
    vox_cr = (vox_vol / num_voxels) ** (1/3)

    # other variables
    voxels = []
    surface_voxels = []
    rgb_matrix = []
    num_true_voxels = 0
    total_voxels = 0
    surf_vox_var = 0
    prev = []

    # loop through the range created
    for x in np.arange(-vox_x/2, vox_x/2, vox_cr):
        for y in np.arange(-vox_y/2, vox_y/2, vox_cr):
            for z in np.arange(0, vox_z, vox_cr):
                P_arr = np.zeros(8)

                # increment total number of voxels
                total_voxels += 1
                world_coord = np.array([x, y, z, 1.0]).T

                # loop through each projection matrix
                for i in range(8):
                    cam_coord = np.dot(P[:, :, i], world_coord)
                    cam_coord = np.round(cam_coord / cam_coord[2])

                    # condition to add the silhouette value to P_arr
                    if (0 <= int(cam_coord[0]) <= 779) and (0 <= int(cam_coord[1]) <= 581):
                        #P_arr[i] = sil[cam_coord[1], cam_coord[0], i]
                        P_arr[i] = sil[int(cam_coord[1]), int(cam_coord[0]), i]


                # check if voxel is the one we need
                if np.all(P_arr == 1):
                    # add to the true voxels and add to the matrix for output
                    num_true_voxels += 1
                    voxels.append([x, y, z])

                    # get RGB values from last image
                    R = cam[int(cam_coord[1]), int(cam_coord[0]), 0, 7]
                    G = cam[int(cam_coord[1]), int(cam_coord[0]), 1, 7]
                    B = cam[int(cam_coord[1]), int(cam_coord[0]), 2, 7]

                    # add the RGB value to the matrix used for output
                    rgb_matrix.append([R, G, B])
                    
                    # condition to detect surface voxels
                    # discard voxels that are not surface voxels
                    if surf_vox_var == 0:
                        # similar to above condition, but here for 
                        # surface voxels
                        surface_voxels.append([x, y, z])
                        surf_vox_var += 1
                        prev = [x, y, z]
                        continue
                    if (prev[0] == x) and (prev[1] == y):
                        surf_vox_var += 1
                    else:
                        if surf_vox_var > 1:
                            surface_voxels.append(prev)
                            surface_voxels.append([x, y, z])
                            tmp = 1
                        else:
                            surface_voxels.append([x, y, z])
                            tmp = 1
                    if surf_vox_var > 0:
                        prev = [x, y, z]
    '''
    OUTPUT GENERATION
    '''
    voxel_noRGB = OUTPUT_DIR + 'noRGB_' + str(num_voxels) + '.ply'
    voxel_RGB = OUTPUT_DIR + 'RGB_' + str(num_voxels) + '.ply'
    voxel_surface = OUTPUT_DIR + 'surface_' + str(num_voxels) + '.ply'

    # generate output (voxel point cloud without color)
    voxels_pc = o3d.geometry.PointCloud()
    voxels_pc.points = o3d.utility.Vector3dVector(voxels)
    o3d.io.write_point_cloud(voxel_noRGB, voxels_pc, write_ascii=True)

    # generate output (voxel point cloud)
    voxels_pc.colors = o3d.utility.Vector3dVector(np.array(rgb_matrix) / 255.0)
    o3d.io.write_point_cloud(voxel_RGB, voxels_pc, write_ascii=True)

    # generate output (surface voxel point cloud)
    surface_voxels_pc = o3d.geometry.PointCloud()
    surface_voxels_pc.points = o3d.utility.Vector3dVector(surface_voxels)
    o3d.io.write_point_cloud(voxel_surface, surface_voxels_pc, write_ascii=True)

    '''
    RATIO OF VOXELS
    '''
    print(f"Total voxels: {total_voxels}")
    print(f"True voxels: {num_true_voxels}")
    print(f"Surface voxels: {surf_vox_var}")
    print()
    print("-------------------------------------------------------")

if __name__ == "__main__":
    '''
    Get the projection matrices from all the cameras
    '''

    P = []

    for file in os.listdir(CALIBRATION_DIR):
        if file.endswith(".xml"):
            with open(os.path.join(CALIBRATION_DIR, file), 'r') as f:
                xml_string = f.read()
                root = ET.fromstring(xml_string)
                numbers = root.text.split()

            calib_matrices = np.array(numbers, dtype=np.float32).reshape((3, 4))
            P.append(calib_matrices)

    P = np.array(P)
    P = np.transpose(P, (1, 2, 0))
    #print(P.shape)

    '''
    Get the camera images into one array
    '''

    cam = np.zeros((582, 780, 3, 8))
    cam_count = 0

    for file in os.listdir(CAMERA_DIR):
        image = Image.open(os.path.join(CAMERA_DIR, file))
        cam[:,:,:,cam_count] = image
        cam_count += 1

    '''
    Get the silhouette images into one array
    '''

    sil = np.zeros((582, 780, 8))
    sil_count = 0

    for file in os.listdir(SILHOUETTE_DIR):
        silhouette = Image.open(os.path.join(SILHOUETTE_DIR, file))
        sil[:,:,sil_count] = silhouette
        sil_count += 1

    voxel_range = [100000, 1000000, 10000000, 100000000]

    for num in voxel_range:
        voxel_reconstruction(P, cam, sil, num)

    # construct the RGB voxel point cloud of 10^8 voxels
    test_ply = o3d.io.read_point_cloud("output/RGB_100000000.ply")
    o3d.visualization.draw_geometries([test_ply])
