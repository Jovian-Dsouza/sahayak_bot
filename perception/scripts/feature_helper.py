# Helper module to extract features from the pointcloud data
# This features are then used to train a ML classifier


import matplotlib.colors
import matplotlib.pyplot as plt
import numpy as np
from pcl_helper import *
import pcl


def rgb_to_hsv(rgb_list):
    rgb_normalized = [1.0*rgb_list[0]/255, 1.0*rgb_list[1]/255, 1.0*rgb_list[2]/255]
    hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
    return hsv_normalized


def compute_color_histograms(cloud, using_hsv=False):

    # Compute histograms for the clusters
    point_colors_list = []

    # Step through each point in the point cloud
    for point in pc2.read_points(cloud, skip_nans=True): 
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    # Populate lists with color values
    channel_1_vals = []
    channel_2_vals = []
    channel_3_vals = []

    for color in point_colors_list:
        channel_1_vals.append(color[0])
        channel_2_vals.append(color[1])
        channel_3_vals.append(color[2])
    
    # Compute histograms

    hist_1 = np.histogram(channel_1_vals[:], bins=32, range=(0,256))
    hist_2 = np.histogram(channel_2_vals[:], bins=32, range=(0,256))
    hist_3 = np.histogram(channel_3_vals[:], bins=32, range=(0,256))

    # Concatenate and normalize the histograms
    hist_features = np.concatenate((hist_1[0], hist_2[0], hist_3[0])).astype(np.float64)
    
    normed_features = hist_features / np.sum(hist_features)
    return normed_features 

def compute_normal_histograms(normal_cloud, nbins=32, nrange=(-1,1)):
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    for norm_component in normal_cloud:
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])

    # Compute histograms

    hist_1 = np.histogram(norm_x_vals[:], bins=nbins, range=nrange)
    hist_2 = np.histogram(norm_y_vals[:], bins=nbins, range=nrange)
    hist_3 = np.histogram(norm_z_vals[:], bins=nbins, range=nrange)

    #  Concatenate and normalize the histograms
    hist_features = np.concatenate((hist_1[0], hist_2[0], hist_3[0])).astype(np.float64)
    
    normed_features = hist_features / np.sum(hist_features)

    return normed_features

def get_normals(pcl_cloud_rgb):
    pcl_cloud = XYZRGB_to_XYZ(pcl_cloud_rgb)
    feature = pcl_cloud.make_NormalEstimation()
    feature.set_KSearch(3)
    normals = feature.compute()
    return normals

def extract_feature(pcl_cluster):
    ros_cluster = pcl_to_ros(pcl_cluster)
    chists = compute_color_histograms(ros_cluster, using_hsv=True)
    # normals = get_normals(pcl_cluster)
    # nhists = compute_normal_histograms(normals)
    # feature = np.concatenate((chists, nhists))
    return chists

def plot_hist(features, nbins=32):
    plt.hist(features, nbins)
    plt.xlabel('Weight', fontsize = 14)
    plt.xticks(fontsize = 14)
    plt.yticks(fontsize = 14)
    plt.show()


# if __name__ == "__main__":
#     cloud_path = 'allObjects.pcd'
#     cloud = pcl.load(cloud_path)
#     print(cloud.size)
#     print(cloud[0])
#     print(type(cloud))
    # normals=get_normals(cloud)
    # normed_features=compute_normal_histograms(normals)
    # plot_hist(normed_features,32)
    # chists = compute_color_histograms(ros_cluster, using_hsv=True)
    # plot_hist(chists, features=32)
