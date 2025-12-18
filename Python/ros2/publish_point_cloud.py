import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

def publish_point_cloud(self, disparity_map, Q_matrix):
    # 1. OpenCV: Reproject to 3D
    points_3d = cv2.reprojectImageTo3D(disparity_map, Q_matrix)

    # 2. Filter: Remove points at infinity (optional but recommended)
    mask = disparity_map > disparity_map.min()
    valid_points = points_3d[mask]

    # 3. Create Header
    header = Header()
    header.stamp = self.get_clock().now().to_msg()
    header.frame_id = "camera_link"  # This must match your TF frame

    # 4. Convert to ROS PointCloud2 message
    # This function is the "Bridge" that makes it compatible with ROS
    pc2_msg = pc2.create_cloud_xyz32(header, valid_points)

    # 5. Publish
    self.publisher.publish(pc2_msg)