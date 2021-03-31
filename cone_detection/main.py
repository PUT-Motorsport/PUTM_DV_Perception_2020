from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import rospy
from sklearn.cluster import Birch, MeanShift, estimate_bandwidth
from math import sqrt


rospy.init_node('test_points')
rospy.Rate(50)
pub = rospy.Publisher("pc2", PointCloud2, queue_size=1)

cone_width = 0.228
cone_height = 0.325


def euclidean_distance(x, y, z):
    return sqrt(x**2 + y**2 + z**2)


def callback(ros_point_cloud):
    gen = pc2.read_points(ros_point_cloud)
    int_data = list(gen)
    new_data = []
    result = []
    #cluster = MeanShift(bandwidth=6)
    cluster = Birch(threshold=0.3)

    for x in int_data:
        if x[2] < -0.5 or euclidean_distance(x[0], x[1], x[2]) > 4:
            continue
        else:
            new_data.append(x)
    cluster.fit(new_data)

    #centers = cluster.cluster_centers_
    centers = cluster.subcluster_centers_
    '''for center in centers:
        dist = euclidean_distance(center[0], center[1], center[2])
        border_width = cone_width / dist
        border_height = cone_height / (2 * dist)
        for point in int_data:
            if center[0] - border_width < point[0] < center[0] + border_width and \
                    center[1] - border_width < point[1] < center[1] + border_width and \
                    center[2] - border_height < point[2] < center[2] + border_height:
                result.append(point)'''

    header = ros_point_cloud.header
    fields = ros_point_cloud.fields
    #_pc2 = pc2.create_cloud(header, fields, result)
    _pc2 = pc2.create_cloud(header, fields, new_data)
    pub.publish(_pc2)


rospy.Subscriber('/cloud', PointCloud2, callback, queue_size=1)
rospy.spin()
