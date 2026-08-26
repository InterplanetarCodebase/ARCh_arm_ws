import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np
from ikpy.chain import Chain
import matplotlib.cm as cm
import csv # Added for file saving

class WorkspacePublisher(Node):
    def __init__(self):
        super().__init__('workspace_publisher')
        self.publisher_ = self.create_publisher(Marker, 'workspace_cloud', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("Calculating Workspace Points & Singularities...")
        
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
        det_vals = []
        
        # List to hold the exact data of the singularity points
        singularity_data_log = [] 
        
        delta = 1e-4 
        singularity_threshold = 0.0005 
        
        for _ in range(num_samples):
            angles = [np.random.uniform(l, h) for l, h in zip(lower_bounds, upper_bounds)]
            
            fk = arm_chain.forward_kinematics(angles)
            x, y, z = fk[0, 3], fk[1, 3], fk[2, 3]
            
            J = np.zeros((3, len(angles)))
            for i in range(len(angles)):
                perturbed_angles = list(angles)
                perturbed_angles[i] += delta
                p_fk = arm_chain.forward_kinematics(perturbed_angles)
                
                J[0, i] = (p_fk[0, 3] - x) / delta
                J[1, i] = (p_fk[1, 3] - y) / delta
                J[2, i] = (p_fk[2, 3] - z) / delta
                
            det = np.linalg.det(J @ J.T)
            
            fk_results.append((x, y, z))
            z_vals.append(z)
            det_vals.append(det)
            
            # Log the data if it falls in the singularity zone
            if det < singularity_threshold:
                singularity_data_log.append({
                    'x': x, 'y': y, 'z': z,
                    'angles': angles
                })
            
        # 3. Apply Colormap and Construct Marker
        z_min, z_max = min(z_vals), max(z_vals)
        colormap = cm.get_cmap('viridis')
        
        for (x, y, z), det in zip(fk_results, det_vals):
            p = Point(x=x, y=y, z=z)
            self.points.append(p)
            
            c = ColorRGBA()
            if det < singularity_threshold:
                c.r, c.g, c.b, c.a = 1.0, 0.0, 0.0, 1.0
            else:
                norm_z = (z - z_min) / (z_max - z_min) if z_max > z_min else 0
                rgba = colormap(norm_z)
                c.r, c.g, c.b, c.a = float(rgba[0]), float(rgba[1]), float(rgba[2]), 0.4
                
            self.colors.append(c)
            
        # 4. Save Singularity Data to a CSV File
        csv_filename = "singularity_points.csv"
        with open(csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            
            # Create dynamic headers (X, Y, Z, Joint_0, Joint_1, etc.)
            headers = ['X', 'Y', 'Z'] + [f'Joint_{i}' for i in range(len(lower_bounds))]
            writer.writerow(headers)
            
            # Write the recorded data
            for data in singularity_data_log:
                row = [data['x'], data['y'], data['z']] + data['angles']
                writer.writerow(row)
                
        self.get_logger().info(f"Saved {len(singularity_data_log)} singularity points to {csv_filename}!")
        self.get_logger().info("Broadcasting to RViz...")
        
        self.marker = Marker()
        self.marker.header.frame_id = "base_link" 
        self.marker.ns = "workspace"
        self.marker.id = 0
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.015 
        self.marker.scale.y = 0.015
        self.marker.points = self.points
        self.marker.colors = self.colors

    def timer_callback(self):
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