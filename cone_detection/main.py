from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import rospy


rospy.init_node('test_points')
rospy.Rate(10)
pub = rospy.Publisher("pc2", PointCloud2, queue_size=1)


def callback(ros_point_cloud):
    gen = pc2.read_points(ros_point_cloud)
    int_data = list(gen)
    new_data = []
    for x in int_data:
        if x[2] < -0.15 or x[0] < 0:
            continue
        else:
            new_data.append(x)
    header = ros_point_cloud.header
    fields = ros_point_cloud.fields
    _pc2 = pc2.create_cloud(header, fields, new_data)
    pub.publish(_pc2)


rospy.Subscriber('/velodyne_points', PointCloud2, callback, queue_size=1)
rospy.spin()
