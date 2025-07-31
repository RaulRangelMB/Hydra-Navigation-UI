# Semantic Navigation UI for Dynamic Scene Graphs

This project provides a graphical user interface (GUI) built with Tkinter that allows users to interact with a ROS2 Dynamic Scene Graph (DSG) map server. Designed to be paired with [Hydra-ROS](https://github.com/MIT-SPARK/Hydra-ROS), it enables semantic navigation by dynamically fetching room and object information from the DSG and sending navigation goals to a Nav2-enabled robot.

## Table of Contents

1.  [Description](#description)
2.  [Dependencies](#dependencies)
3.  [Installation](#installation)
4.  [Topics and Services Used/Expected](#topics-and-services-usedexpected)
5.  [Usage](#usage)

## Description

The `Semantic Navigation UI` offers a user-friendly way to leverage a Dynamic Scene Graph for robot navigation.

Users can:
* Select a specific room to view objects within that room.
* View all available objects across the entire environment, regardless of their room.
* Click on an object to send a Nav2 `PoseStamped` goal to the robot, directing it to the precise location of that object as reported by the DSG.

The project consists of two main components:
* **`SemanticNavigationNode`**: A ROS2 Python node that acts as a client to the `DsgMapServer`'s services (for listing rooms and objects) and publishes `PoseStamped` messages to Nav2's `/goal_pose` topic.
* **`SemanticNavigationUI`**: A Tkinter-based GUI that provides the interactive elements for room and object selection. It asynchronously communicates with the `SemanticNavigationNode` to fetch dynamic data and dispatch navigation commands.

## Dependencies

This project relies on the following:

* **ROS2 (Humble Hawksbill or later recommended)**: The core robotic operating system. This project was developed on ROS2 Iron.
* **`rclpy`**: ROS2 client library for Python.
* **`tkinter`**: Python's standard GUI toolkit (usually comes with Python installation).
* **`geometry_msgs`**: ROS2 message type for `PoseStamped`.
* **Your Custom ROS2 Package (e.g., `dsg_query`)**: This package must contain the definitions for the following custom messages and services:
    * **Messages:**
        * `dsg_query/msg/Room.msg`
        * `dsg_query/msg/RoomObject.msg`
    * **Services:**
        * `dsg_query/srv/ListRooms.srv`
        * `dsg_query/srv/ListRoomObjects.srv`
        * `dsg_query/srv/MoveToObject.srv` (used by `DsgMapServer`, not directly by UI for now)
* **`DsgMapServer` (C++ ROS2 Node)**: The backend node that provides the Dynamic Scene Graph information via ROS2 services. This node is expected to publish its services under `/hydra/backend/...` topics. Click [here](https://github.com/JoaoLucasMBC/dsg-query-ros2) to visit the DsgMapServer repository.

## Installation

1.  **Set up your ROS2 Workspace**:
    If you don't have a ROS2 workspace, create one:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2.  **Clone or Create Your Custom ROS2 Package**:
    Ensure your custom ROS2 package
   

3.  **Build Your ROS2 Workspace**:
    Navigate to your workspace root and build, ensuring your custom message/service package is built.
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

## Topics and Services Used/Expected

The `Semantic Navigation UI` interacts with the following ROS2 interfaces:

* **Published Topic:**
    * `/goal_pose` (`geometry_msgs/msg/PoseStamped`): Publishes the target 2D pose for Nav2 navigation.

* **Subscribed Services (from `DsgMapServer`):**
    * `/hydra/backend/rooms` (`dsg_query/srv/ListRooms`): Used to fetch the list of available rooms.
    * `/hydra/backend/all_objects` (`dsg_query/srv/ListRoomObjects`): Used to fetch a list of all objects across all rooms.
    * `/hydra/backend/objects` (`dsg_query/srv/ListRoomObjects`): Used to fetch a list of objects within a specific room.

* **Expected `DsgMapServer` Node:**
    * A ROS2 node (like the C++ `DsgMapServer` provided) is expected to be running and advertising these services.

## Usage

1.  **Start Your ROS2 Environment**:
    Open a terminal and source your ROS2 workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2.  **Launch the `DsgMapServer`**:
    Start your C++ `DsgMapServer` node. The exact command depends on your package and executable name.
    ```bash
    ros2 run dsg_query dsg_map_node
    ```

3.  **Run the Semantic Navigation UI**:
    In a new terminal, source your ROS2 workspace again and then execute the Python script:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run hydra_navigation_ui semantic_navigation_ui
    ```

4.  **Interact with the UI**:
    * The UI will display buttons for available rooms (fetched from the `DsgMapServer`).
    * Click a room button to see objects within that room.
    * Click the "All Objects" button to see all detected objects in the environment.
    * Click an object button to send its coordinates as a navigation goal to `/goal_pose`. A confirmation pop-up will appear with the goal details.
    * Use the "Back to Rooms" button to return to the main room selection screen.