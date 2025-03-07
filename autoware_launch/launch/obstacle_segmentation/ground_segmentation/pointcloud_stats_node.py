#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import time
from std_msgs.msg import String
import threading

class PointCloudStatsNode(Node):
    def __init__(self):
        super().__init__('pointcloud_stats_node')
        
        # Get parameters
        self.declare_parameter('input_topics', ['/sensing/lidar/concatenated/pointcloud', 
                                              '/perception/obstacle_segmentation/pointcloud'])
        self.declare_parameter('publish_frequency', 1.0)
        
        self.input_topics = self.get_parameter('input_topics').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        
        # Create subscribers for each input topic
        self.subscribers = {}
        self.topic_stats = {}
        
        for topic in self.input_topics:
            self.get_logger().info(f"Subscribing to: {topic}")
            topic_name = topic.replace('/', '_')[1:] if topic.startswith('/') else topic.replace('/', '_')
            self.subscribers[topic] = self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, t=topic: self.pointcloud_callback(msg, t),
                10)
            
            self.topic_stats[topic] = {
                'count': 0,
                'last_received': 0,
                'frequency': 0.0,
                'points_min': float('inf'),
                'points_max': 0,
                'points_avg': 0,
                'total_points': 0,
                'fields': [],
                'frame_id': '',
                'last_timestamp': 0
            }
        
        # Create publishers for stats
        self.stats_publisher = self.create_publisher(String, '/debug/pointcloud_stats', 10)
        
        # Start timer for publishing stats
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.publish_stats)
        self.get_logger().info("PointCloud Stats Node initialized")
        
        # Lock for thread safety
        self.lock = threading.Lock()
        
    def pointcloud_callback(self, msg, topic):
        with self.lock:
            stats = self.topic_stats[topic]
            
            # Update basic counters
            stats['count'] += 1
            current_time = time.time()
            
            if stats['last_received'] > 0:
                dt = current_time - stats['last_received']
                stats['frequency'] = 0.8 * stats['frequency'] + 0.2 * (1.0 / dt) if dt > 0 else 0
            
            stats['last_received'] = current_time
            stats['frame_id'] = msg.header.frame_id
            
            # Count points
            try:
                # Get number of points - we'll avoid converting to numpy for large clouds
                num_points = msg.width * msg.height
                
                # Update stats
                stats['points_min'] = min(stats['points_min'], num_points)
                stats['points_max'] = max(stats['points_max'], num_points)
                stats['total_points'] += num_points
                stats['points_avg'] = stats['total_points'] / stats['count']
                
                # Store field info only on first message
                if not stats['fields']:
                    stats['fields'] = [field.name for field in msg.fields]
                
                # Track timestamp
                stats['last_timestamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                
                # Only log occasionally to avoid spam
                if stats['count'] % 10 == 0:
                    self.get_logger().info(f"Received {topic}: {num_points} points, frame: {msg.header.frame_id}")
                    
            except Exception as e:
                self.get_logger().error(f"Error processing {topic}: {str(e)}")
    
    def publish_stats(self):
        with self.lock:
            # Create a formatted stats message
            stats_msg = String()
            stats_text = "PointCloud Statistics:\n"
            
            for topic, stats in self.topic_stats.items():
                stats_text += f"\n=== {topic} ===\n"
                stats_text += f"  Messages: {stats['count']}\n"
                stats_text += f"  Frequency: {stats['frequency']:.2f} Hz\n"
                stats_text += f"  Last received: {time.time() - stats['last_received']:.2f} seconds ago\n"
                stats_text += f"  Frame ID: {stats['frame_id']}\n"
                
                if stats['count'] > 0:
                    stats_text += f"  Points (min/avg/max): {stats['points_min']}/{stats['points_avg']:.1f}/{stats['points_max']}\n"
                    stats_text += f"  Fields: {', '.join(stats['fields'])}\n"
                    
                    # Calculate time delay between topics if we have multiple
                    if len(self.topic_stats) > 1 and topic == self.input_topics[1] and stats['last_timestamp'] > 0:
                        input_stats = self.topic_stats[self.input_topics[0]]
                        if input_stats['last_timestamp'] > 0:
                            delay = stats['last_timestamp'] - input_stats['last_timestamp']
                            stats_text += f"  Delay from input: {delay:.6f} seconds\n"
                
            stats_msg.data = stats_text
            self.stats_publisher.publish(stats_msg)
            self.get_logger().info(stats_text)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudStatsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()