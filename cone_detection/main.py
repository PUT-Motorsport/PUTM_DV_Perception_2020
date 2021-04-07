from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import rospy
from math import sqrt
import pcl
import numpy as np

rospy.init_node('test_points')
rospy.Rate(50)
pub = rospy.Publisher("pc2", PointCloud2, queue_size=1)

cone_width = 0.228
cone_height = 0.325


def euclidean_distance(x, y, z):
    return sqrt(x**2 + y**2 + z**2)


def pc2_to_list(cloud):
    return list(pc2.read_points(cloud))


def list_to_pcl(cloud):
    p = pcl.PointCloud_PointXYZI()
    p.from_list(cloud)
    return p


def list_to_pc2(cloud, ros_cl):
    header = ros_cl.header
    fields = ros_cl.fields
    return pc2.create_cloud(header, fields, cloud)


def pcl_to_pc2(cloud, ros_cl):
    cl = []
    for point in cloud:
        cl.append(point)
    return list_to_pc2(cl, ros_cl)


def euclid_cluster(cloud):
    cl = []
    for point in cloud:
        cl.append([point[0], point[1], point[2]])
    colorless_cloud = pcl.PointCloud()
    colorless_cloud.from_list(cl)
    tree = colorless_cloud.make_kdtree()
    ec = colorless_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.4)
    ec.set_MinClusterSize(3)
    ec.set_MaxClusterSize(500)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    return cluster_indices, colorless_cloud


def get_cluster_cloud(cluster_indices, whole_cloud):
    clusters = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            clusters.append([whole_cloud[indice][0],
                             whole_cloud[indice][1],
                             whole_cloud[indice][2],
                             whole_cloud[indice][3]])

    cluster_cloud = pcl.PointCloud_PointXYZI()
    cluster_cloud.from_list(clusters)

    return cluster_cloud


def get_centroids_cloud(cluster_indices, cloud_filtered):
    centroids_cloud = []
    for i, indices in enumerate(cluster_indices):
        x = 0
        y = 0
        z = 0
        num_of_points = len(indices)
        for indice in indices:
            x += cloud_filtered[indice][0]
            y += cloud_filtered[indice][1]
            z += cloud_filtered[indice][2]
        x = x / num_of_points
        y = y / num_of_points
        z = z / num_of_points
        centroids_cloud.append([x, y, z, 1.0])

    return centroids_cloud


def reconstruction(centroid_cloud, whole_cloud):
    cloud = []

    for center in centroid_cloud:
        border_width = cone_width / 2
        border_height = cone_height / 2
        for point in whole_cloud:
            if center[0] - border_width < point[0] < center[0] + border_width and \
                    center[1] - border_width < point[1] < center[1] + border_width and \
                    center[2] - border_height < point[2] < center[2] + border_height:
                cloud.append(point)

    return cloud


def callback(ros_point_cloud):
    input_cloud = pc2_to_list(ros_point_cloud)
    cloud_gl = []

    for x in input_cloud:
        if x[2] < -0.5 or euclidean_distance(x[0], x[1], x[2]) > 4:
            continue
        else:
            cloud_gl.append([x[0], x[1], x[2], x[3]])

    cloud_pcl = list_to_pcl(cloud_gl)

    vg = cloud_pcl.make_voxel_grid_filter()
    vg.set_leaf_size(0.05, 0.05, 0.05)
    cloud_filtered = vg.filter()

    cluster_indices, colorless_cloud = euclid_cluster(cloud_filtered)
    # cluster_cloud = get_cluster_cloud(cluster_indices, cloud_filtered)
    centroid_cloud = get_centroids_cloud(cluster_indices, colorless_cloud)
    cloud_reco = reconstruction(centroid_cloud, input_cloud)

    cloud_pc2 = pcl_to_pc2(cloud_reco, ros_point_cloud)
    pub.publish(cloud_pc2)


rospy.Subscriber('/cloud', PointCloud2, callback, queue_size=1)
rospy.spin()
