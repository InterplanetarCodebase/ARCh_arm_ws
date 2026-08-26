import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np
from ikpy.chain import Chain
import matplotlib.cm as cm

class WorkspacePublisher(Node):
    def __init__(self):
        super().__init__('workspace_publisher')
        # Publish a Marker message to the 'workspace_cloud' topic
        self.publisher_ = self.create_publisher(Marker, 'workspace_cloud', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("Calculating Workspace Points (This takes a few seconds)...")
        
        # 1. Load URDF and extract bounds
        urdf_path = "src/rover_arm_urdf/urdf/rover_arm_urdf.urdf"
        arm_chain = Chain.from_urdf_file(urdf_path)
        
        lower_bounds, upper_bounds = [], []
        for link in arm_chain.links:
            if link.bounds is None:
                lower_bounds.append(0.0)
                upper_bounds.append(0.0)
            else:
                low, high = link.bounds
                if low is None or low == -np.inf: low = -np.pi
                if high is None or high == np.inf: high = np.pi
                if low > high: low, high = high, low
                lower_bounds.append(low)
                upper_bounds.append(high)

        # 2. Monte Carlo Sampling
        num_samples = 15000
        self.points = []
        self.colors = []
        fk_results = []
        z_vals = []
        
        for _ in range(num_samples):
            angles = [np.random.uniform(l, h) for l, h in zip(lower_bounds, upper_bounds)]
            fk = arm_chain.forward_kinematics(angles)
            fk_results.append((fk[0, 3], fk[1, 3], fk[2, 3]))
            z_vals.append(fk[2, 3])
            
        # 3. Apply Viridis Colormap based on Z-Height
        z_min, z_max = min(z_vals), max(z_vals)
        colormap = cm.get_cmap('viridis')
        
        for x, y, z in fk_results:
            # Append Point
            p = Point()
            p.x, p.y, p.z = x, y, z
            self.points.append(p)
            
            # Append Color
            norm_z = (z - z_min) / (z_max - z_min) if z_max > z_min else 0
            rgba = colormap(norm_z)
            c = ColorRGBA()
            c.r, c.g, c.b, c.a = float(rgba[0]), float(rgba[1]), float(rgba[2]), 0.6
            self.colors.append(c)
            
        self.get_logger().info("Calculation complete! Broadcasting to RViz...")
        
        # 4. Construct the Marker
        self.marker = Marker()
        self.marker.header.frame_id = "base_link" # Anchors cloud to the robot base
        self.marker.ns = "workspace"
        self.marker.id = 0
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.015 # Size of each point
        self.marker.scale.y = 0.015
        self.marker.points = self.points
        self.marker.colors = self.colors

    def timer_callback(self):
        # Update the timestamp and publish
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    node = WorkspacePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()