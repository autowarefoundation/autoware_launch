import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2 
import math

class PointCloudConverterNode(Node):
    def __init__(self):
        super().__init__('pointcloud_converter')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lslidar_point_cloud',  # 输入话题名称
            self.pointcloud_callback,
            10)
        self.publisher_ = self.create_publisher(PointCloud2, '/sensing/lidar/concatenated/pointcloud', 10)

    def pointcloud_callback(self, msg):
        # 读取原始点云数据
        points = list(pc2.read_points(msg, 
                                     field_names=["x", "y", "z", "intensity", "ring", "time"],
                                     skip_nans=True))
        
        # 定义新字段结构
        new_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.UINT8, count=1),
            PointField(name='return_type', offset=13, datatype=PointField.UINT8, count=1),
            PointField(name='channel', offset=14, datatype=PointField.UINT16, count=1),
            PointField(name='azimuth', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='elevation', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='distance', offset=24, datatype=PointField.FLOAT32, count=1),
            PointField(name='time_stamp', offset=28, datatype=PointField.UINT32, count=1),
        ]

        # 转换并计算每个点
        converted_points = []
        for point in points:
            x = point[0]  # FLOAT32
            y = point[1]  # FLOAT32
            z = point[2]  # FLOAT32
            intensity = int(point[3]) & 0xFF  # UINT8
            ring = int(point[4]) & 0xFFFF  # UINT16
            time_sec = point[5]  # 原始time字段（假设为秒）
            time_ns = int(time_sec * 1e9)  # 转换为纳秒 UINT32
            return_type = 0
            
            # 计算派生字段
            distance = math.sqrt(x**2 + y**2 + z**2)
            azimuth = math.atan2(y, x)
            elevation = math.atan2(z, distance)

            # 组合新点的字段顺序与new_fields对应
            converted_point = [
                float(x), float(y), float(z),  # X,Y,Z
                intensity,                    # I
                return_type,
                ring,                         # C
                azimuth,                      # A
                elevation,                    # E
                distance,                     # D
                time_ns                       # T
            ]
            converted_points.append(converted_point)

        # 创建新的PointCloud2消息
        header = msg.header
        msg.header.frame_id = 'base_link'
        new_msg = pc2.create_cloud(header, new_fields, converted_points)
        self.publisher_.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudConverterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()