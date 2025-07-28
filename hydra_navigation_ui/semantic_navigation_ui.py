import tkinter as tk
from tkinter import messagebox
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import threading # For spinning ROS2 in a separate thread

class SemanticNavigationNode(Node):
    def __init__(self):
        super().__init__('semantic_navigation_node')
        # Publisher for Nav2 goal_pose
        self.goal_publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.get_logger().info('Semantic Navigation Node started. Publishing to /goal_pose.')

    def publish_nav2_goal(self, x, y, z):
        """
        Publishes a PoseStamped message to the /goal_pose topic for Nav2.
        """
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'  # Assuming robot operates in the 'map' frame
        goal_msg.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.position.z = float(z)

        # Default orientation (facing forward along x-axis)
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0 # Identity quaternion (no rotation)

        self.goal_publisher_.publish(goal_msg)
        self.get_logger().info(f"Published Nav2 goal for (x: {x}, y: {y}, z: {z})")


class SemanticNavigationUI:
    def __init__(self, root, ros_node):
        self.root = root
        self.root.title("Semantic Navigation UI")
        self.ros_node = ros_node

        self.rooms_frame = tk.Frame(root)
        self.objects_frame = tk.Frame(root)

        # --- Hardcoded Dataset ---
        self.dataset = {
            "kitchen": {
                "id": 1,
                "objects": {
                    "oven": {"id": 101, "x": 1.0, "y": 0.5, "z": 0.0},
                    "fridge": {"id": 102, "x": 1.5, "y": 2.0, "z": 0.0},
                    "microwave": {"id": 103, "x": 0.8, "y": 1.0, "z": 0.0}
                }
            },
            "living room": {
                "id": 2,
                "objects": {
                    "table": {"id": 201, "x": -2.0, "y": -2.0, "z": 0.0},
                    "chair": {"id": 202, "x": -4.5, "y": -1.5, "z": 0.0},
                    "couch": {"id": 203, "x": -1.0, "y": -1.0, "z": 0.0}
                }
            },
            "bathroom": {
                "id": 3,
                "objects": {
                    "bath": {"id": 301, "x": -3.0, "y": 2.0, "z": 0.0},
                    "sink": {"id": 302, "x": -2.5, "y": 2.5, "z": 0.0},
                    "trash can": {"id": 303, "x": -1.2, "y": 1.2, "z": 0.0}
                }
            }
        }

        self.current_room = None
        self.create_room_buttons()

    def create_room_buttons(self):
        # Clear any existing widgets
        for widget in self.rooms_frame.winfo_children():
            widget.destroy()
        self.objects_frame.pack_forget() # Hide objects frame if visible

        self.rooms_frame.pack(padx=20, pady=20)
        tk.Label(self.rooms_frame, text="Select a Room:", font=("Arial", 14)).pack(pady=10)

        for room_name in self.dataset.keys():
            btn = tk.Button(self.rooms_frame, text=room_name.capitalize(),
                            command=lambda r=room_name: self.show_objects_in_room(r),
                            width=20, height=2)
            btn.pack(pady=5)

    def show_objects_in_room(self, room_name):
        self.current_room = room_name
        # Clear any existing widgets in objects frame
        for widget in self.objects_frame.winfo_children():
            widget.destroy()

        self.rooms_frame.pack_forget() # Hide rooms frame
        self.objects_frame.pack(padx=20, pady=20)

        tk.Label(self.objects_frame, text=f"{room_name.capitalize()}", font=("Arial", 25, "bold")).pack(pady=5)
        tk.Label(self.objects_frame, text=f"Select an object to navigate to:", font=("Arial", 12)).pack(pady=10)

        objects = self.dataset[room_name]["objects"]
        for obj_name, obj_data in objects.items():
            btn_text = f"{obj_name.capitalize()} (ID: {obj_data['id']})"
            btn = tk.Button(self.objects_frame, text=btn_text,
                            command=lambda o_data=obj_data: self.send_goal_to_object(o_data),
                            width=30, height=2)
            btn.pack(pady=5)

        # Back button
        back_btn = tk.Button(self.objects_frame, text="Back to Rooms", command=self.create_room_buttons)
        back_btn.pack(pady=15)

    def send_goal_to_object(self, obj_data):
        x = obj_data['x']
        y = obj_data['y']
        z = obj_data['z'] # Z is typically 0.0 for 2D navigation

        # Call the ROS2 node method to publish the goal
        self.ros_node.publish_nav2_goal(x, y, z)
        messagebox.showinfo("Nav2 Goal Sent",
                            f"Goal sent for {self.current_room} - {obj_data['id']}\n"
                            f"Position: X={x}, Y={y}, Z={z}")


def main(args=None):
    rclpy.init(args=args)
    ros_node = SemanticNavigationNode()

    root = tk.Tk()
    app = SemanticNavigationUI(root, ros_node)

    # --- ROS2 Spinning in a separate thread ---
    # This is a more robust way to handle ROS2 callbacks without blocking the Tkinter mainloop.
    def ros_spin_thread():
        rclpy.spin(ros_node)

    spin_thread = threading.Thread(target=ros_spin_thread)
    spin_thread.daemon = True # Allows the thread to exit when the main program exits
    spin_thread.start()

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
        # You might need to explicitly join the thread if it's not daemonized
        # or if cleanup needs to wait for it. For this simple case, daemon=True is fine.

if __name__ == '__main__':
    main()