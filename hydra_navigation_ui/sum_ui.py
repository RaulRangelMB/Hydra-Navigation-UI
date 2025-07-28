import tkinter as tk
from tkinter import messagebox
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class SimpleCalculatorNode(Node):
    def __init__(self):
        super().__init__('simple_calculator_node')
        # This publisher is just an example for ROS2 integration.
        # In a real scenario, you'd publish the sum or other processed data.
        self.publisher_ = self.create_publisher(Int64, 'sum_result', 10)
        self.get_logger().info('Simple Calculator Node started.')

    def publish_sum(self, num1, num2):
        sum_val = num1 + num2
        msg = Int64()
        msg.data = sum_val
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published sum: {sum_val}')
        return sum_val

class CalculatorApp:
    def __init__(self, root, ros_node):
        self.root = root
        self.root.title("Simple ROS2 Calculator")
        self.ros_node = ros_node

        # Entry widgets for numbers
        self.label1 = tk.Label(root, text="Number 1:")
        self.label1.grid(row=0, column=0, padx=10, pady=10)
        self.entry1 = tk.Entry(root)
        self.entry1.grid(row=0, column=1, padx=10, pady=10)

        self.label2 = tk.Label(root, text="Number 2:")
        self.label2.grid(row=1, column=0, padx=10, pady=10)
        self.entry2 = tk.Entry(root)
        self.entry2.grid(row=1, column=1, padx=10, pady=10)

        # Button to calculate sum
        self.sum_button = tk.Button(root, text="Calculate Sum", command=self.calculate_sum)
        self.sum_button.grid(row=2, column=0, columnspan=2, pady=10)

        # Label to display result
        self.result_label = tk.Label(root, text="Sum: ")
        self.result_label.grid(row=3, column=0, columnspan=2, pady=10)

    def calculate_sum(self):
        try:
            num1 = int(self.entry1.get())
            num2 = int(self.entry2.get())

            # Call the ROS2 node method to calculate and publish the sum
            calculated_sum = self.ros_node.publish_sum(num1, num2)

            self.result_label.config(text=f"Sum: {calculated_sum}")
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid integers for both numbers.")
        except Exception as e:
            messagebox.showerror("Error", f"An error occurred: {e}")

def main(args=None):
    rclpy.init(args=args)
    ros_node = SimpleCalculatorNode()

    root = tk.Tk()
    app = CalculatorApp(root, ros_node)

    # Use root.after to spin the ROS2 node periodically
    def ros_spin():
        rclpy.spin_once(ros_node, timeout_sec=0.1)
        root.after(100, ros_spin) # Call itself after 100ms

    root.after(100, ros_spin) # Start the ROS2 spinning

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()