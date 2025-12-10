import sys 
from threading import Thread
import rclpy
from rclpy.node import Node
import rclpy.executors 

import ros_nodes
import numpy as np 
import cv2

from PIL import Image, ImageTk


import tkinter as tk
import os
os.environ['DISPLAY'] = ':23'

class Panel:
    def __init__(self, root, publisher_node, listener_node):
        self.root = root
        self.publisher_node = publisher_node
        self.listener_node = listener_node

        self.root.title("ROV Control Panel")
        self.root.geometry("900x550") 

        # --- Trajectory State ---
        self.map_size = 300     
        self.map_scale = 20     # [px/m]
        
        # map background
        self.map_background = np.ones((self.map_size, self.map_size, 3), dtype=np.uint8) * 255
        
        self.last_pos_px = None 
        self.origin_pos = None  # start pose

        # --- Control State ---
        self.shift_state = [0.0, 0.0, 0.0]
        self.rotate_state = [0.0, 0.0, 0.0]
        self.power = 40.0 # Scale factor - power

        # --- Layout Configuration ---
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1) 
        self.root.columnconfigure(2, weight=1)
        self.root.rowconfigure(1, weight=1) 
        
        # Labels
        tk.Label(root, text="TRAJEKTORIA (Top-Down)", font=('Arial', 10, 'bold')).grid(row=0, column=0, pady=5)
        tk.Label(root, text="STEROWANIE", font=('Arial', 10, 'bold')).grid(row=0, column=1, pady=5)
        tk.Label(root, text="SONAR FLS", font=('Arial', 10, 'bold')).grid(row=0, column=2, pady=5)
        
        # 1. Trjectory map
        self.ImgPlaceholder_Trajectory = tk.Label(root, bg="white", relief=tk.SUNKEN)
        self.ImgPlaceholder_Trajectory.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        # 2. Controls
        control_frame = tk.Frame(root, bd=2, relief=tk.GROOVE)
        control_frame.grid(row=1, column=1, sticky="nsew", padx=5, pady=10)
        
        # Control panel grid - layout 
        control_frame.columnconfigure(0, weight=1)
        control_frame.columnconfigure(1, weight=1)

        # Buttons
        btn_fwd = tk.Button(control_frame, text='▲\nForward', bg="#d1e7dd", height=3)
        btn_fwd.grid(row=0, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        btn_fwd.bind('<ButtonPress-1>', lambda e: self.set_shift_x(self.power))
        btn_fwd.bind('<ButtonRelease-1>', lambda e: self.set_shift_x(0.0))

        btn_left = tk.Button(control_frame, text='◀ Left', bg="#d1e7dd", height=3)
        btn_left.grid(row=1, column=0, sticky="ew", padx=5, pady=5)
        btn_left.bind('<ButtonPress-1>', lambda e: self.set_shift_y(self.power)) # Y+ to lewo w ROS
        btn_left.bind('<ButtonRelease-1>', lambda e: self.set_shift_y(0.0))

        btn_right = tk.Button(control_frame, text='Right ▶', bg="#d1e7dd", height=3)
        btn_right.grid(row=1, column=1, sticky="ew", padx=5, pady=5)
        btn_right.bind('<ButtonPress-1>', lambda e: self.set_shift_y(-self.power)) # Y- to prawo w ROS
        btn_right.bind('<ButtonRelease-1>', lambda e: self.set_shift_y(0.0))

        btn_back = tk.Button(control_frame, text='▼\nBackward', bg="#d1e7dd", height=3)
        btn_back.grid(row=2, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        btn_back.bind('<ButtonPress-1>', lambda e: self.set_shift_x(-self.power))
        btn_back.bind('<ButtonRelease-1>', lambda e: self.set_shift_x(0.0))
        
        # Info label
        self.lbl_status = tk.Label(control_frame, text="Waiting for telemetry...", fg="gray")
        self.lbl_status.grid(row=3, column=0, columnspan=2, pady=20)

        # Sonar display
        self.ImgPlaceholder_Sonar = tk.Label(root, bg="black", relief=tk.SUNKEN)
        self.ImgPlaceholder_Sonar.grid(row=1, column=2, padx=10, pady=10, sticky="nsew")

        # Start loop
        self.update_all_displays()

    # --- Control Logic ---
    def new_event(self):
       # send ross cmd
        self.publisher_node.send_cmd(self.shift_state, self.rotate_state)
        
    def set_shift_x(self, val):
        self.shift_state[0] = val
        self.new_event()
    def set_shift_y(self, val):
        self.shift_state[1] = val
        self.new_event()

    # --- Display Logic ---
    def update_all_displays(self):
        if self.listener_node:
            self.show_img_Sonar()
            self.show_img_Trajectory()
        
        # Refreshing
        self.root.after(100, self.update_all_displays)
        
    def show_img_Sonar(self):
        if hasattr(self.listener_node, 'FLS') and self.listener_node.FLS is not None:
            fls_data = self.listener_node.FLS
            
            # Scale
            fls_resized = cv2.resize(fls_data, (300, 300), interpolation=cv2.INTER_NEAREST)
            
            # What is that? idk - chat did it
            if fls_resized.ndim == 2:
                fls_color = cv2.applyColorMap(fls_resized, cv2.COLORMAP_JET)
                fls_final = cv2.cvtColor(fls_color, cv2.COLOR_BGR2RGB)
            else:
                fls_final = fls_resized

            img_tk = ImageTk.PhotoImage(Image.fromarray(fls_final))
            self.ImgPlaceholder_Sonar.configure(image=img_tk)
            self.ImgPlaceholder_Sonar.image = img_tk

    def show_img_Trajectory(self):
        # Check if data available
        if not hasattr(self.listener_node, 'POSITION') or self.listener_node.POSITION is None:
            self.lbl_status.configure(text="Waiting for ROS connection...", fg="red")
            return
        # Get pose [x, y, z]
        pos = self.listener_node.POSITION

        if len(pos) < 2: return
        
        # Check if it non zero pose
        if np.all(pos == 0) and self.origin_pos is None:
            self.lbl_status.configure(text="Waiting for movement...", fg="orange")
            # return # Odkomentuj jeśli chcesz czekać na ruch

        # --- Set origin to center of map ---
        if self.origin_pos is None:
            self.origin_pos = (pos[0], pos[1])
            self.lbl_status.configure(text="System ARMED. Origin set.", fg="green")
            print(f"Origin set at: {self.origin_pos}")

        # Calc relative pose
        rel_x = pos[0] - self.origin_pos[0]
        rel_y = pos[1] - self.origin_pos[1]

        # Convert to pixels
        # Centrum mapy to (map_size/2, map_size/2)
        # ROS X+ (Przód) -> UI Y- (Góra)
        # ROS Y+ (Lewo)  -> UI X- (Lewo)  <- Tutaj zależy od konwencji, zazwyczaj Y w lewo to X w lewo na ekranie
        
        center = self.map_size // 2
        
        # map X axis <=> Y axis of robot (left/right)
        px_x = int(center - (rel_y * self.map_scale)) 
        
        # map Y axis <=> X axis of robot (forward/backward)
        px_y = int(center - (rel_x * self.map_scale))

        current_px = (px_x, px_y)

        # 1. Draw line
        if self.last_pos_px is not None:
            cv2.line(self.map_background, self.last_pos_px, current_px, (0, 0, 0), 1)
        
        self.last_pos_px = current_px

        display_img = self.map_background.copy()

        # draw robot actual pose
        cv2.circle(display_img, current_px, 5, (0, 0, 255), -1) # BGR: Red

        # Convert to PIL
        display_rgb = cv2.cvtColor(display_img, cv2.COLOR_BGR2RGB)
        traj_tk = ImageTk.PhotoImage(Image.fromarray(display_rgb))

        self.ImgPlaceholder_Trajectory.configure(image=traj_tk)
        self.ImgPlaceholder_Trajectory.image = traj_tk
        
        # Debug info
        self.lbl_status.configure(text=f"Pos: X={rel_x:.2f}m Y={rel_y:.2f}m")

        
def main():
    rclpy.init()
    
    control_node = ros_nodes.StonefishPublisher()
    listener_node = ros_nodes.StonefishSubscriber() 

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(control_node)
    executor.add_node(listener_node)

    # Start ROS w osobnym wątku
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Init GUI
    root = tk.Tk()
    master = Panel(root, control_node, listener_node) 
    
    try:
        root.mainloop()
    finally:
        # Graceful shutdown
        control_node.destroy_node()
        listener_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

