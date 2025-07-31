import tkinter as tk
from tkinter import messagebox
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

try:
    from dsg_query.msg import Room, RoomObject
    from dsg_query.srv import ListRooms, ListRoomObjects, MoveToObject 
except ImportError:
    print("Warning: Could not import custom ROS2 messages/services from 'dsg_query'.")
    print("Please ensure your 'dsg_query' package is built and sourced correctly.")
    print("Falling back to dummy data for demonstration.")
    
    class Room:
        def __init__(self, room_id=0, name=""):
            self.room_id = room_id
            self.name = name # Currently, names of rooms have not been implemented in the C++ code, so this is a placeholder.

    class RoomObject:
        def __init__(self, object_id=0, room_id=0, semantic_label="", x=0.0, y=0.0, z=0.0):
            self.object_id = object_id
            self.room_id = room_id
            self.semantic_label = semantic_label
            self.x = x
            self.y = y
            self.z = z

    class ListRooms:
        class Request:
            pass
        class Response:
            def __init__(self):
                self.rooms = []

    class ListRoomObjects:
        class Request:
            def __init__(self, room=None):
                self.room = room if room is not None else Room(room_id=0) # Default to an empty Room if not provided
        class Response:
            def __init__(self):
                self.objects = []


class SemanticNavigationNode(Node):
    def __init__(self):
        super().__init__('semantic_navigation_node')
        # Publisher for Nav2 goal_pose
        self.goal_publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.get_logger().info('Semantic Navigation Node started. Publishing to /goal_pose.')

        # Create a callback group for service clients to allow concurrent calls
        self.callback_group = ReentrantCallbackGroup()

        # Service clients for DSG Map Server
        self.list_rooms_client = self.create_client(
            ListRooms, '/hydra/backend/rooms', callback_group=self.callback_group)
        self.list_all_objects_client = self.create_client(
            ListRoomObjects, '/hydra/backend/all_objects', callback_group=self.callback_group)
        self.list_objects_client = self.create_client(
            ListRoomObjects, '/hydra/backend/objects', callback_group=self.callback_group)

        # Wait for services to be available
        self.get_logger().info("Waiting for DSG Map Server services...")
        self.list_rooms_client.wait_for_service(timeout_sec=5.0)
        self.list_all_objects_client.wait_for_service(timeout_sec=5.0)
        self.list_objects_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info("DSG Map Server services are available.")


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

    def get_rooms(self):
        """
        Calls the /hydra/backend/rooms service to get a list of rooms.
        Returns a Future that will resolve to the service response.
        """
        request = ListRooms.Request()
        self.get_logger().info("Requesting list of rooms...")
        return self.list_rooms_client.call_async(request)

    def get_all_objects(self):
        """
        Calls the /hydra/backend/all_objects service to get a list of all objects.
        Returns a Future that will resolve to the service response.
        """
        request = ListRoomObjects.Request() # Request is empty for all objects
        self.get_logger().info("Requesting list of all objects...")
        return self.list_all_objects_client.call_async(request)

    def get_objects_in_room(self, room_id):
        """
        Calls the /hydra/backend/objects service to get a list of objects in a specific room.
        Returns a Future that will resolve to the service response.
        """
        request = ListRoomObjects.Request()
        request.room.room_id = room_id # Populate the room_id in the request
        self.get_logger().info(f"Requesting objects for room ID: {room_id}...")
        return self.list_objects_client.call_async(request)


class SemanticNavigationUI:
    def __init__(self, root, ros_node):
        self.root = root
        self.root.title("Semantic Navigation UI")
        self.ros_node = ros_node

        self.rooms_frame = tk.Frame(root)
        self.objects_frame = tk.Frame(root)

        # Store the dynamically fetched dataset
        # Structure: {room_id: {"name": room_name, "objects": {object_id: object_data}}}
        self.dataset = {}
        # A mapping from object ID to its full data, useful for 'All Objects'
        self.all_objects_map = {}

        self.current_room_id = None
        
        # Initial call to fetch data and populate the UI
        self.fetch_and_display_rooms()

    def fetch_and_display_rooms(self):
        """
        Asynchronously fetches room data and then calls create_room_buttons.
        """
        self.ros_node.get_logger().info("Fetching rooms from DSG Map Server...")
        future = self.ros_node.get_rooms()
        future.add_done_callback(self.rooms_response_callback)

    def rooms_response_callback(self, future):
        """
        Callback for when the rooms service call returns.
        """
        try:
            response = future.result()
            self.dataset = {} # Clear existing data
            if response.rooms:
                for room in response.rooms:
                    # Assuming Room.msg has a 'name' field or we can infer it.
                    # If not, you might need to adjust 'room.name' to something like f"Room {room.room_id}"
                    # For now, let's assume `Room` has a `name` attribute.
                    # If Room.msg only has room_id, you might want to adjust the DsgMapServer to send a name,
                    # or infer a name like "Room X" in the UI. For this example, let's assume there's a 'name'
                    # field in the `Room` message from `dsg_query`. If `Room` only has `room_id`, you'll
                    # need to modify the C++ DsgMapServer to add `string name` to `Room.msg` or
                    # generate dummy names here.
                    room_name = f"Room {room.room_id}" # Fallback if no name in Room.msg
                    # If your Room.msg actually has a 'name' field, use it:
                    # room_name = room.name
                    # Todo: implement room names in the DsgMapServer C++ code
                    self.dataset[room.room_id] = {"name": room_name, "objects": {}}
                self.ros_node.get_logger().info(f"Fetched {len(self.dataset)} rooms.")
            else:
                self.ros_node.get_logger().warn("No rooms found from DSG Map Server.")
            
            # Populating the UI with the fetched rooms
            self.create_room_buttons()

        except Exception as e:
            self.ros_node.get_logger().error(f"Failed to call list_rooms service: {e}")
            messagebox.showerror("ROS2 Service Error", f"Failed to fetch rooms: {e}")
            # Optionally, fallback to some dummy data or keep the UI empty
            self.create_room_buttons() # Still create buttons, but they might be empty


    def create_room_buttons(self):
        # Clear any existing widgets
        for widget in self.rooms_frame.winfo_children():
            widget.destroy()
        self.objects_frame.pack_forget() # Hide objects frame if visible

        # Header
        self.rooms_frame.pack(padx=20, pady=20)
        tk.Label(self.rooms_frame, text="Select a Room:", font=("Arial", 14)).pack(pady=10)

        # Create buttons for each room
        if not self.dataset:
            tk.Label(self.rooms_frame, text="No rooms available. Ensure DSG Map Server is running.", fg="red").pack(pady=10)
        else:
            for room_id, room_data in self.dataset.items():
                btn = tk.Button(self.rooms_frame, text=room_data["name"].capitalize(),
                                command=lambda r_id=room_id: self.fetch_and_show_objects_in_room(r_id),
                                width=20, height=2)
                btn.pack(pady=5)

        # Create a button for all objects
        all_objects_btn = tk.Button(self.rooms_frame, text="All Objects",
                                    command=self.fetch_and_show_all_objects,
                                    width=20, height=2)
        all_objects_btn.pack(pady=10)

    def fetch_and_show_all_objects(self):
        """
        Asynchronously fetches all object data and then calls show_all_objects.
        """
        self.ros_node.get_logger().info("Fetching all objects from DSG Map Server...")
        future = self.ros_node.get_all_objects()
        future.add_done_callback(self.all_objects_response_callback)

    def all_objects_response_callback(self, future):
        """
        Callback for when the all_objects service call returns.
        """
        try:
            response = future.result()
            self.all_objects_map = {} # Clear existing data
            if response.objects:
                for obj in response.objects:
                    # Store object with its full data
                    self.all_objects_map[obj.object_id] = {
                        "id": obj.object_id,
                        "semantic_label": obj.semantic_label,
                        "room_id": obj.room_id, # Keep room_id for context if needed
                        "x": obj.x,
                        "y": obj.y,
                        "z": obj.z
                    }
                self.ros_node.get_logger().info(f"Fetched {len(self.all_objects_map)} total objects.")
            else:
                self.ros_node.get_logger().warn("No objects found from DSG Map Server.")
            
            # Now, populate the UI
            self.show_all_objects()

        except Exception as e:
            self.ros_node.get_logger().error(f"Failed to call list_all_objects service: {e}")
            messagebox.showerror("ROS2 Service Error", f"Failed to fetch all objects: {e}")
            self.show_all_objects() # Still attempt to show, but it might be empty


    def show_all_objects(self):
        # Clear any existing widgets in objects frame
        for widget in self.objects_frame.winfo_children():
            widget.destroy()

        self.current_room_id = None # Indicate "All Objects" mode

        self.rooms_frame.pack_forget() # Hide rooms frame
        self.objects_frame.pack(padx=20, pady=20)

        # Header
        tk.Label(self.objects_frame, text=f"All Objects", font=("Arial", 25, "bold")).pack(pady=5)
        tk.Label(self.objects_frame, text=f"Select an object to navigate to:", font=("Arial", 12)).pack(pady=10)

        # Create buttons for each object
        if not self.all_objects_map:
            tk.Label(self.objects_frame, text="No objects available. Ensure DSG Map Server is running and has objects.", fg="red").pack(pady=10)
        else:
            for obj_id, obj_data in self.all_objects_map.items():
                room_name = "Unknown Room"
                if obj_data['room_id'] in self.dataset:
                    room_name = self.dataset[obj_data['room_id']]["name"]
                
                btn_text = f"{obj_data['semantic_label'].capitalize()} (ID: {obj_data['id']}) - Room: {room_name}"
                btn = tk.Button(self.objects_frame, text=btn_text,
                                command=lambda o_data=obj_data: self.send_goal_to_object(o_data),
                                width=40, height=2)
                btn.pack(pady=5)

        # Back button
        back_btn = tk.Button(self.objects_frame, text="Back to Rooms", command=self.create_room_buttons)
        back_btn.pack(pady=15)

    def fetch_and_show_objects_in_room(self, room_id):
        """
        Asynchronously fetches object data for a specific room and then calls show_objects_in_room.
        """
        self.current_room_id = room_id
        self.ros_node.get_logger().info(f"Fetching objects for room {room_id} from DSG Map Server...")
        future = self.ros_node.get_objects_in_room(room_id)
        future.add_done_callback(lambda future: self.objects_in_room_response_callback(future, room_id))

    def objects_in_room_response_callback(self, future, room_id):
        """
        Callback for when the objects in a room service call returns.
        """
        try:
            response = future.result()
            # Update the dataset for the specific room
            if room_id in self.dataset:
                self.dataset[room_id]["objects"] = {} # Clear existing
                if response.objects:
                    for obj in response.objects:
                        self.dataset[room_id]["objects"][obj.object_id] = {
                            "id": obj.object_id,
                            "semantic_label": obj.semantic_label,
                            "x": obj.x,
                            "y": obj.y,
                            "z": obj.z
                        }
                    self.ros_node.get_logger().info(f"Fetched {len(response.objects)} objects for room ID: {room_id}.")
                else:
                    self.ros_node.get_logger().warn(f"No objects found for room ID: {room_id}.")
            
            # Populating the UI with the fetched objects
            self.show_objects_in_room(room_id)

        except Exception as e:
            self.ros_node.get_logger().error(f"Failed to call list_objects service for room {room_id}: {e}")
            messagebox.showerror("ROS2 Service Error", f"Failed to fetch objects for room {room_id}: {e}")
            self.show_objects_in_room(room_id) # Still attempt to show, but it might be empty


    def show_objects_in_room(self, room_id):
        # Clear any existing widgets in objects frame
        for widget in self.objects_frame.winfo_children():
            widget.destroy()

        self.rooms_frame.pack_forget() # Hide rooms frame
        self.objects_frame.pack(padx=20, pady=20)

        # Header
        room_name = self.dataset[room_id]["name"].capitalize() if room_id in self.dataset else "Unknown Room"
        tk.Label(self.objects_frame, text=f"{room_name}", font=("Arial", 25, "bold")).pack(pady=5)
        tk.Label(self.objects_frame, text=f"Select an object to navigate to:", font=("Arial", 12)).pack(pady=10)

        objects = self.dataset[room_id]["objects"] if room_id in self.dataset else {}
        
        # Create buttons for each object in the room
        if not objects:
            tk.Label(self.objects_frame, text="No objects available in this room.", fg="red").pack(pady=10)
        else:
            for obj_id, obj_data in objects.items():
                btn_text = f"{obj_data['semantic_label'].capitalize()} (ID: {obj_data['id']})"
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
        
        obj_label = obj_data['semantic_label']
        obj_id = obj_data['id']
        room_info = ""
        if self.current_room_id is not None and self.current_room_id in self.dataset:
            room_info = f" for {self.dataset[self.current_room_id]['name']}"
        elif self.current_room_id is None: # All Objects mode
             # Try to find the room name for the object if in 'All Objects' view
             # This assumes 'obj_data' from 'all_objects_map' contains 'room_id'
            if 'room_id' in obj_data and obj_data['room_id'] in self.dataset:
                room_info = f" in {self.dataset[obj_data['room_id']]['name']}"

        messagebox.showinfo("Nav2 Goal Sent",
                            f"Goal sent for Object: {obj_label.capitalize()} (ID: {obj_id}){room_info}\n"  
                            f"Position: X={x}, Y={y}, Z={z}")


def main(args=None):
    rclpy.init(args=args)
    
    # Use MultiThreadedExecutor to handle service client callbacks concurrently
    # This is important for GUI responsiveness while waiting for service responses.
    executor = MultiThreadedExecutor()
    ros_node = SemanticNavigationNode()
    executor.add_node(ros_node)

    root = tk.Tk()
    app = SemanticNavigationUI(root, ros_node)

    # --- ROS2 Spinning in a separate thread using the MultiThreadedExecutor ---
    def ros_spin_thread():
        executor.spin()

    spin_thread = threading.Thread(target=ros_spin_thread)
    spin_thread.daemon = True # Allows the thread to exit when the main program exits
    spin_thread.start()

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()