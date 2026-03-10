import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class ActionChunkPublisher(Node):
    def __init__(self):
        super().__init__('action_chunk_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, '/action_chunk', 10)
        self.chunk_data = np.load('action_chunk.npy')  # shape: (12, 497)
        self.idx = 0
        self.total = self.chunk_data.shape[0]
        self.timer = self.create_timer(1.0 , self.publish_chunk)

    def publish_chunk(self):
        if self.idx < self.total:
            row = self.chunk_data[self.idx].copy()
            # Convert right hand joint order in data[241:497]
            convert_indices = [4, 0, 8, 12, 5, 1, 9, 13, 6, 2, 10, 14, 3, 7, 11, 15]
            rh = row[241:497].reshape(16, 16)
            rh_converted = np.array([[step[i] for i in convert_indices] for step in rh])
            row[241:497] = rh_converted.flatten()
            msg = Float64MultiArray()
            msg.data = row.tolist()
            self.publisher.publish(msg)
            self.get_logger().info(f'Published action_chunk {self.idx+1}/{self.total}')
            self.idx += 1
        else:
            self.get_logger().info('All chunks published, shutting down.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ActionChunkPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()