import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped

class MapMerger(Node):
    def __init__(self):
        super().__init__('map_merger')
        
        # Create a TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create subscriptions to the individual SLAM maps
        self.map_left_sub = self.create_subscription(
            OccupancyGrid,
            '/left_map',
            self.map_left_callback,
            10)
        
        self.map_right_sub = self.create_subscription(
            OccupancyGrid,
            '/right_map',
            self.map_right_callback,
            10)

        # Publisher for the merged map
        self.merged_map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # Variables to store maps
        self.left_map = None
        self.right_map = None

    def map_left_callback(self, msg):
        self.left_map = msg
        self.merge_maps()

    def map_right_callback(self, msg):
        self.right_map = msg
        self.merge_maps()

    def merge_maps(self):
        if self.left_map and self.right_map:
            # Obtain the transform between the 'map' and the right map's frame
            try:
                # Checking if the transform is available
                if self.tf_buffer.can_transform('map', self.right_map.header.frame_id, rclpy.time.Time()):
                    transform = self.tf_buffer.lookup_transform('map', self.right_map.header.frame_id, rclpy.time.Time())

                    # Apply the transform to the right map to align it with the left map's coordinate frame
                    transformed_map = self.apply_transform_to_map(self.right_map, transform)

                    # Now, combine the two maps into a single grid
                    merged_map = OccupancyGrid()
                    merged_map.header.stamp = self.get_clock().now().to_msg()
                    merged_map.header.frame_id = 'map'

                    # Assuming both maps are the same resolution, we can simply combine their data in a naive way.
                    merged_map.info = self.left_map.info  # Assume same grid resolution, size for simplicity.
                    merged_map.info.width = self.left_map.info.width + transformed_map.info.width  # Example: horizontal concat
                    merged_map.info.height = max(self.left_map.info.height, transformed_map.info.height)  # Take the max height

                    # Initialize an empty grid with the merged size
                    merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)  # Initialize with unknown values

                    # Fill in the merged map's data. This is where you'd handle the actual spatial alignment and data merging.
                    self.fill_merged_map_data(merged_map, self.left_map, transformed_map)

                    # Publish the merged map
                    self.merged_map_pub.publish(merged_map)

                else:
                    self.get_logger().warn("Transform not available between 'map' and 'right_map' frame.")

            except TransformException as e:
                self.get_logger().error(f"Could not transform map: {e}")

    def apply_transform_to_map(self, map, transform):
        """
        Apply the transform to the map to adjust it to the target frame.
        This will modify the map’s origin and data based on the transform.
        """
        # In this example, we will simply print the transform. In a real scenario, you need to apply
        # the translation and rotation to the map's data (occupancy grid cells).
        self.get_logger().info(f"Applying transform: {transform}")
        transformed_map = map  # Placeholder: In practice, you would adjust the map's position and data based on the transform.
        return transformed_map

    def fill_merged_map_data(self, merged_map, left_map, right_map):
        """
        Fill the merged map data by copying the left and right map data into the correct positions.
        The right map may need to be offset depending on the transformation.
        """
        for i in range(len(left_map.data)):
            merged_map.data[i] = left_map.data[i]  # Copy left map data into merged map

        # Assume right_map's data needs to be placed after left_map's data
        for i in range(len(right_map.data)):
            merged_map.data[len(left_map.data) + i] = right_map.data[i]  # Place right map data after the left map

def main(args=None):
    rclpy.init(args=args)
    node = MapMerger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
