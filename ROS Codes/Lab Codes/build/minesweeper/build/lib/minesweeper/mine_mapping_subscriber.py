import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator
import numpy as np

mine_Material_Type = {
    'Surface': '#40E0D0',
    'Buried': '#008080'
}

class MineMappingSubscriber(Node):
    def __init__(self):
        super().__init__('mine_mapping_subscriber')
        
        # Create subscriptions
        self.create_subscription(Int32, 'mine_x', self.x_callback, 10)
        self.create_subscription(Int32, 'mine_y', self.y_callback, 10)
        self.create_subscription(String, 'mine_type', self.type_callback, 10)

        # Initialize plot and mines list
        self.fig, self.ax = self.initialize_map()
        self.mines = []

    def initialize_map(self):
        fig, ax = plt.subplots(figsize=(10, 10))

        ax.set_xticks(np.arange(1, 21))
        ax.set_yticks(np.arange(1, 21))

        ax.set_xticklabels(['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T'])

        ax.set_aspect('equal')
        plt.grid(which='both', color='gray', linestyle='-', linewidth=0)

        ax.xaxis.set_minor_locator(AutoMinorLocator(2))
        ax.yaxis.set_minor_locator(AutoMinorLocator(2))
        plt.grid(which='minor', color='gray', linestyle='-', linewidth=1)

        ax.set_xlim(0.5, 20.5)
        ax.set_ylim(0.5, 20.5)

        ax.set_xlabel('X Coordinate (A to T)')
        ax.set_ylabel('Y Coordinate (1 to 20)')
        ax.set_title('Mine Detection Map')

        plt.ion()  # Enable interactive mode for live updating
        plt.show()
        
        return fig, ax

    def update_map(self, x, y, mine_type):
        color = mine_Material_Type[mine_type]
        self.ax.scatter(x, y, c=color, marker="s", s=800)
        plt.draw()
        plt.pause(0.1)  # Pause to update the plot

    def x_callback(self, msg):
        self.x = msg.data

    def y_callback(self, msg):
        self.y = msg.data

    def type_callback(self, msg):
        mine_type = msg.data
        self.mines.append((self.x, self.y, mine_type))  # Store the received mine data
        self.get_logger().info(f"Received mine at ({self.x}, {self.y}): {mine_type}")
        self.update_map(self.x, self.y, mine_type)  # Update the map with the new mine

def main(args=None):
    rclpy.init(args=args)
    node = MineMappingSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close(node.fig)  # Close the plot on shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
