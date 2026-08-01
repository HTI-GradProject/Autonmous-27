#!/usr/bin/env python3
import sys
import threading
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopGUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Speed values
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        
        self.root.title("Rover Teleop Control Center")
        self.root.geometry("400x430")
        self.root.configure(bg="#1e272e")
        
        # Style
        style = ttk.Style()
        style.theme_use('clam')
        
        # Title Label
        title_label = tk.Label(root, text="🛸 Rover Control Center", font=("Helvetica", 16, "bold"), fg="#d2dae2", bg="#1e272e")
        title_label.pack(pady=15)
        
        # Controls Frame
        ctrl_frame = tk.Frame(root, bg="#1e272e")
        ctrl_frame.pack(pady=10)
        
        # Buttons layout
        btn_opts = {"font": ("Helvetica", 11, "bold"), "width": 9, "height": 2, "bd": 0, "relief": "flat"}
        
        self.btn_up = tk.Button(ctrl_frame, text="▲ Forward\n(Up Arrow)", bg="#05c46b", fg="#ffffff", activebackground="#04a75b", command=self.move_forward, **btn_opts)
        self.btn_up.grid(row=0, column=1, padx=5, pady=5)
        
        self.btn_left = tk.Button(ctrl_frame, text="◀ Left\n(Left Arrow)", bg="#3c40c6", fg="#ffffff", activebackground="#2f32a7", command=self.turn_left, **btn_opts)
        self.btn_left.grid(row=1, column=0, padx=5, pady=5)
        
        self.btn_stop = tk.Button(ctrl_frame, text="⏹ STOP\n(Space)", bg="#ff3f34", fg="#ffffff", activebackground="#e62e24", command=self.stop, **btn_opts)
        self.btn_stop.grid(row=1, column=1, padx=5, pady=5)
        
        self.btn_right = tk.Button(ctrl_frame, text="Right ▶\n(Right Arrow)", bg="#3c40c6", fg="#ffffff", activebackground="#2f32a7", command=self.turn_right, **btn_opts)
        self.btn_right.grid(row=1, column=2, padx=5, pady=5)
        
        self.btn_down = tk.Button(ctrl_frame, text="▼ Reverse\n(Down Arrow)", bg="#ffc048", fg="#1e272e", activebackground="#ffa801", command=self.move_backward, **btn_opts)
        self.btn_down.grid(row=2, column=1, padx=5, pady=5)
        
        # Keyboard bindings
        self.root.bind("<Up>", lambda event: self.move_forward())
        self.root.bind("<Down>", lambda event: self.move_backward())
        self.root.bind("<Left>", lambda event: self.turn_left())
        self.root.bind("<Right>", lambda event: self.turn_right())
        self.root.bind("<space>", lambda event: self.stop())
        
        # Speed Sliders Frame
        speed_frame = tk.Frame(root, bg="#2f3542")
        speed_frame.pack(pady=15, fill="x", padx=30, ipady=10)
        
        lbl_lin = tk.Label(speed_frame, text="Linear Speed (m/s):", fg="#f1f2f6", bg="#2f3542", font=("Helvetica", 9, "bold"))
        lbl_lin.grid(row=0, column=0, sticky="w", padx=10, pady=5)
        self.scale_lin = tk.Scale(speed_frame, from_=0.1, to=2.0, resolution=0.1, orient="horizontal", bg="#2f3542", fg="#f1f2f6", highlightthickness=0, command=self.update_speeds)
        self.scale_lin.set(self.linear_speed)
        self.scale_lin.grid(row=0, column=1, sticky="we", padx=10)
        
        lbl_ang = tk.Label(speed_frame, text="Angular Speed (rad/s):", fg="#f1f2f6", bg="#2f3542", font=("Helvetica", 9, "bold"))
        lbl_ang.grid(row=1, column=0, sticky="w", padx=10, pady=5)
        self.scale_ang = tk.Scale(speed_frame, from_=0.2, to=3.0, resolution=0.1, orient="horizontal", bg="#2f3542", fg="#f1f2f6", highlightthickness=0, command=self.update_speeds)
        self.scale_ang.set(self.angular_speed)
        self.scale_ang.grid(row=1, column=1, sticky="we", padx=10)
        
        speed_frame.columnconfigure(1, weight=1)
        
        # Status Label
        self.lbl_status = tk.Label(root, text="Status: IDLE", font=("Helvetica", 10, "italic"), fg="#808e9b", bg="#1e272e")
        self.lbl_status.pack(pady=5)
        
    def update_speeds(self, event=None):
        self.linear_speed = float(self.scale_lin.get())
        self.angular_speed = float(self.scale_ang.get())
        
    def publish_twist(self, linear, angular, status_text):
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.publisher.publish(twist)
        self.lbl_status.config(text=f"Status: {status_text}", fg="#d2dae2")
        
    def move_forward(self):
        self.publish_twist(self.linear_speed, 0.0, f"MOVING FORWARD ({self.linear_speed} m/s)")
        
    def move_backward(self):
        self.publish_twist(-self.linear_speed, 0.0, f"REVERSING ({-self.linear_speed} m/s)")
        
    def turn_left(self):
        self.publish_twist(0.0, self.angular_speed, f"TURNING LEFT ({self.angular_speed} rad/s)")
        
    def turn_right(self):
        self.publish_twist(0.0, -self.angular_speed, f"TURNING RIGHT ({-self.angular_speed} rad/s)")
        
    def stop(self):
        self.publish_twist(0.0, 0.0, "STOPPED")

def ros2_spin(node):
    rclpy.spin(node)

def main():
    rclpy.init(args=sys.argv)
    node = Node('teleop_gui_node')
    
    # Threading to spin ROS 2 in the background
    spin_thread = threading.Thread(target=ros2_spin, args=(node,), daemon=True)
    spin_thread.start()
    
    root = tk.Tk()
    app = TeleopGUI(root, node)
    
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
