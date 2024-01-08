import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float64
import tf2_ros
from tf2_ros import TransformListener



class MyTransformNode(Node):
    def __init__(self):
        super().__init__('my_transform_node')

        self.declare_parameter('base_frame', 'base_link')
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        
        self.declare_parameter('laser_frame', 'laser')
        self.laser_frame = self.get_parameter('laser_frame').get_parameter_value().string_value

        self.declare_parameter('camera_frame', 'camera_link')
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        self.declare_parameter('global_frame', 'map')
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value

        self.declare_parameter('rate', 10.0)
        self.rate = self.get_parameter('rate').get_parameter_value().double_value


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pose_publisher_base = self.create_publisher(
            PoseStamped, '/tf_'+self.base_frame, 10)
        self.pose_publisher_laser = self.create_publisher(
            PoseStamped, '/tf_'+self.laser_frame, 10)
        self.pose_publisher_camera = self.create_publisher(
            PoseStamped, '/tf_'+self.camera_frame, 10)

      
          

        # Parameters for timer rates
        self.declare_parameter('map_to_base_rate', self.rate)
        self.declare_parameter('map_to_laser_rate', self.rate)
        self.declare_parameter('map_to_camera_rate',self.rate)

        # Timers
        self.map_to_base_timer = self.create_timer(
            self.get_parameter('map_to_base_rate').value,
            self.map_to_base_callback
        )

        self.map_to_laser_timer = self.create_timer(
            self.get_parameter('map_to_laser_rate').value,
            self.map_to_laser_callback
        )

        self.map_to_camera_timer = self.create_timer(
            self.get_parameter('map_to_camera_rate').value,
            self.map_to_camera_callback
        )

    def map_to_base_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.global_frame,
                self.base_frame,  rclpy.time.Time())
            pose_msg = self.create_pose_msg(transform)
            self.pose_publisher_base.publish(pose_msg)
        except Exception as e:
            self.get_logger().error(str(e))

    def map_to_laser_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.global_frame
                self.laser_frame,rclpy.time.Time())
            pose_msg = self.create_pose_msg(transform)
            self.pose_publisher_laser.publish(pose_msg)
        except Exception as e:
            self.get_logger().error(str(e))

    def map_to_camera_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform( self.global_frame,
                self.camera_frame, rclpy.time.Time())
            pose_msg = self.create_pose_msg(transform)
            self.pose_publisher_camera.publish(pose_msg)
        except Exception as e:
            self.get_logger().error(str(e))

    def create_pose_msg(self, transform):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.global_frame
        pose_msg.header.stamp = rclpy.time.Time().to_msg()
        pose_msg.pose.position.x = transform.transform.translation.x
        pose_msg.pose.position.y = transform.transform.translation.y
        pose_msg.pose.position.z = transform.transform.translation.z
        pose_msg.pose.orientation = transform.transform.rotation
        return pose_msg

def main(args=None):
    rclpy.init(args=args)
    node = MyTransformNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
