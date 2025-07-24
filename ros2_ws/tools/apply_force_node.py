import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Vector3, Point
from gazebo_msgs.srv import ApplyLinkWrench

class ApplyForceNode(Node):
    def __init__(self):
        super().__init__('apply_force_node')
        self.cli = self.create_client(ApplyLinkWrench, '/apply_link_wrench')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /apply_link_wrench service...')

        self.force = Wrench()
        self.force.force = Vector3(x=10.0, y=0.0, z=0.0)
        self.force.torque = Vector3(x=0.0, y=0.0, z=0.0)

        self.apply_force()

    def apply_force(self):
        req = ApplyLinkWrench.Request()
        req.link_name = 'force_box::sensor_link'
        req.reference_frame = 'force_box::sensor_link'
        req.reference_point = Point(x=0.0, y=0.0, z=0.0)
        req.wrench = self.force
        req.start_time.sec = 0
        req.start_time.nanosec = 0
        req.duration.sec = 1
        req.duration.nanosec = 0

        future = self.cli.call_async(req)

        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Success: {future.result().success}')
        else:
            self.get_logger().error('Error applying force: Service call failed.')

def main():
    rclpy.init()
    node = ApplyForceNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
