import sys 
from threading import Thread
import rclpy
from rclpy.node import Node
import rclpy.executors 
import math 

import ros_nodes
import numpy as np 
import cv2

from PIL import Image, ImageTk
import tkinter as tk
import os

# --- Environment Setup ---
# Optional: Set display for XServer (if running remotely)
os.environ['DISPLAY'] = ':23'

# --- Helper Function: Quaternion to Euler ---
def euler_from_quaternion(x, y, z, w):
    """
    Converts quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw) in radians.
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # returns radians

class Panel:
    def __init__(self, root, publisher_node, listener_node):
        self.root = root
        self.publisher_node = publisher_node
        self.listener_node = listener_node

        self.root.title("ROV Control Panel")
        self.root.geometry("900x700") # Increased height for new buttons

        # --- Trajectory State ---
        self.map_size = 300     
        self.map_scale = 20     # [px/m]
        self.map_background = np.ones((self.map_size, self.map_size, 3), dtype=np.uint8) * 255
        self.last_pos_px = None 
        self.origin_pos = None  # Start pose reference

        # --- Control State ---
        self.shift_state = [0.0, 0.0, 0.0]  # Linear: x, y, z
        self.rotate_state = [0.0, 0.0, 0.0] # Angular: r, p, y
        self.power = 40.0 

        # --- Layout Configuration ---
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1) 
        self.root.columnconfigure(2, weight=1)
        self.root.rowconfigure(1, weight=1) 
        
        # --- Headers ---
        tk.Label(root, text="TRAJECTORY (Top-Down)", font=('Arial', 10, 'bold')).grid(row=0, column=0, pady=5)
        tk.Label(root, text="CONTROL & TELEMETRY", font=('Arial', 10, 'bold')).grid(row=0, column=1, pady=5)
        tk.Label(root, text="SONAR FLS", font=('Arial', 10, 'bold')).grid(row=0, column=2, pady=5)
        
        # 1. Trajectory Widget (Left Column)
        self.ImgPlaceholder_Trajectory = tk.Label(root, bg="white", relief=tk.SUNKEN)
        self.ImgPlaceholder_Trajectory.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        # 2. Control & Telemetry (Middle Column)
        middle_frame = tk.Frame(root)
        middle_frame.grid(row=1, column=1, sticky="nsew", padx=5, pady=10)
        middle_frame.columnconfigure(0, weight=1)

        # --- A. Control Buttons ---
        control_frame = tk.Frame(middle_frame, bd=2, relief=tk.GROOVE)
        control_frame.grid(row=0, column=0, sticky="ew", pady=(0, 20))
        control_frame.columnconfigure(0, weight=1)
        control_frame.columnconfigure(1, weight=1)

        # Row 0: Forward (Surge +)
        btn_fwd = tk.Button(control_frame, text='▲\nForward', bg="#d1e7dd", height=2)
        btn_fwd.grid(row=0, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        btn_fwd.bind('<ButtonPress-1>', lambda e: self.set_shift_x(self.power))
        btn_fwd.bind('<ButtonRelease-1>', lambda e: self.set_shift_x(0.0))

        # Row 1: Left / Right (Sway +/-)
        btn_left = tk.Button(control_frame, text='◀ Left', bg="#d1e7dd", height=2)
        btn_left.grid(row=1, column=0, sticky="ew", padx=5, pady=5)
        btn_left.bind('<ButtonPress-1>', lambda e: self.set_shift_y(self.power)) 
        btn_left.bind('<ButtonRelease-1>', lambda e: self.set_shift_y(0.0))

        btn_right = tk.Button(control_frame, text='Right ▶', bg="#d1e7dd", height=2)
        btn_right.grid(row=1, column=1, sticky="ew", padx=5, pady=5)
        btn_right.bind('<ButtonPress-1>', lambda e: self.set_shift_y(-self.power))
        btn_right.bind('<ButtonRelease-1>', lambda e: self.set_shift_y(0.0))

        # Row 2: Backward (Surge -)
        btn_back = tk.Button(control_frame, text='▼\nBackward', bg="#d1e7dd", height=2)
        btn_back.grid(row=2, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        btn_back.bind('<ButtonPress-1>', lambda e: self.set_shift_x(-self.power))
        btn_back.bind('<ButtonRelease-1>', lambda e: self.set_shift_x(0.0))
        
        # Row 3: Up / Down (Heave +/-)
        btn_up = tk.Button(control_frame, text='⇪ UP (Z+)', bg="#fff3cd", height=2)
        btn_up.grid(row=3, column=0, sticky="ew", padx=5, pady=5)
        btn_up.bind('<ButtonPress-1>', lambda e: self.set_shift_z(self.power))
        btn_up.bind('<ButtonRelease-1>', lambda e: self.set_shift_z(0.0))

        btn_down = tk.Button(control_frame, text='⇩ DOWN (Z-)', bg="#fff3cd", height=2)
        btn_down.grid(row=3, column=1, sticky="ew", padx=5, pady=5)
        btn_down.bind('<ButtonPress-1>', lambda e: self.set_shift_z(-self.power))
        btn_down.bind('<ButtonRelease-1>', lambda e: self.set_shift_z(0.0))

        # Row 4: Yaw Left / Yaw Right (Yaw +/-)
        btn_yaw_left = tk.Button(control_frame, text='↺ Turn Left', bg="#e2e3e5", height=2)
        btn_yaw_left.grid(row=4, column=0, sticky="ew", padx=5, pady=5)
        btn_yaw_left.bind('<ButtonPress-1>', lambda e: self.set_rotate_z(self.power)) # +Yaw = Left
        btn_yaw_left.bind('<ButtonRelease-1>', lambda e: self.set_rotate_z(0.0))

        btn_yaw_right = tk.Button(control_frame, text='Turn Right ↻', bg="#e2e3e5", height=2)
        btn_yaw_right.grid(row=4, column=1, sticky="ew", padx=5, pady=5)
        btn_yaw_right.bind('<ButtonPress-1>', lambda e: self.set_rotate_z(-self.power)) # -Yaw = Right
        btn_yaw_right.bind('<ButtonRelease-1>', lambda e: self.set_rotate_z(0.0))

        # --- B. Telemetry Table ---
        telemetry_frame = tk.LabelFrame(middle_frame, text=" Live Data ", font=('Arial', 9))
        telemetry_frame.grid(row=1, column=0, sticky="ew")
        telemetry_frame.columnconfigure(0, weight=1) # Label Col
        telemetry_frame.columnconfigure(1, weight=1) # Value Col
        telemetry_frame.columnconfigure(2, weight=1) # Label Col
        telemetry_frame.columnconfigure(3, weight=1) # Value Col

        # Position Data
        tk.Label(telemetry_frame, text="Pos X:").grid(row=0, column=0, sticky="e")
        self.val_x = tk.Label(telemetry_frame, text="0.00", fg="blue")
        self.val_x.grid(row=0, column=1, sticky="w")

        tk.Label(telemetry_frame, text="Pos Y:").grid(row=1, column=0, sticky="e")
        self.val_y = tk.Label(telemetry_frame, text="0.00", fg="blue")
        self.val_y.grid(row=1, column=1, sticky="w")

        tk.Label(telemetry_frame, text="Depth Z:").grid(row=2, column=0, sticky="e")
        self.val_z = tk.Label(telemetry_frame, text="0.00", fg="blue")
        self.val_z.grid(row=2, column=1, sticky="w")

        # Orientation Data
        tk.Label(telemetry_frame, text="Roll:").grid(row=0, column=2, sticky="e")
        self.val_roll = tk.Label(telemetry_frame, text="0°", fg="red")
        self.val_roll.grid(row=0, column=3, sticky="w")

        tk.Label(telemetry_frame, text="Pitch:").grid(row=1, column=2, sticky="e")
        self.val_pitch = tk.Label(telemetry_frame, text="0°", fg="red")
        self.val_pitch.grid(row=1, column=3, sticky="w")

        tk.Label(telemetry_frame, text="Yaw:").grid(row=2, column=2, sticky="e")
        self.val_yaw = tk.Label(telemetry_frame, text="0°", fg="red")
        self.val_yaw.grid(row=2, column=3, sticky="w")

        # 3. Sonar Widget (Right Column)
        self.ImgPlaceholder_Sonar = tk.Label(root, bg="black", relief=tk.SUNKEN)
        self.ImgPlaceholder_Sonar.grid(row=1, column=2, padx=10, pady=10, sticky="nsew")

        # Start Update Loop
        self.update_all_displays()

    # --- Control Logic ---
    def new_event(self):
       # Send command to ROS node
       self.publisher_node.send_cmd(self.shift_state, self.rotate_state)
       
    def set_shift_x(self, val):
        self.shift_state[0] = val
        self.new_event()
    def set_shift_y(self, val):
        self.shift_state[1] = val
        self.new_event()
    def set_shift_z(self, val):
        self.shift_state[2] = val
        self.new_event()
    
    def set_rotate_z(self, val):
        self.rotate_state[2] = val # Yaw index in [r, p, y]
        self.new_event()

    # --- Display Logic ---
    def update_all_displays(self):
        if self.listener_node:
            self.show_img_Sonar()
            self.show_img_Trajectory()
            self.update_telemetry_text()
        
        # Refresh rate: 100ms
        self.root.after(100, self.update_all_displays)

    def update_telemetry_text(self):
        """Updates GUI labels with ROS data."""
        # Update Position
        if hasattr(self.listener_node, 'POSITION') and self.listener_node.POSITION is not None:
            p = self.listener_node.POSITION
            if len(p) >= 3:
                self.val_x.configure(text=f"{p[0]:.2f} m")
                self.val_y.configure(text=f"{p[1]:.2f} m")
                self.val_z.configure(text=f"{p[2]:.2f} m")

        # Update Orientation
        if hasattr(self.listener_node, 'ORIENTATION') and self.listener_node.ORIENTATION is not None:
            q = self.listener_node.ORIENTATION
            if len(q) >= 4:
                # Quaternion -> Euler (rad)
                r, p, y = euler_from_quaternion(q[0], q[1], q[2], q[3])
                
                # rad -> degrees
                r_deg = math.degrees(r)
                p_deg = math.degrees(p)
                y_deg = math.degrees(y)

                self.val_roll.configure(text=f"{r_deg:.1f}°")
                self.val_pitch.configure(text=f"{p_deg:.1f}°")
                self.val_yaw.configure(text=f"{y_deg:.1f}°")

    def show_img_Sonar(self):
        """Displays FLS data with false-color mapping."""
        if hasattr(self.listener_node, 'FLS') and self.listener_node.FLS is not None:
            fls_data = self.listener_node.FLS
            fls_resized = cv2.resize(fls_data, (300, 300), interpolation=cv2.INTER_NEAREST)
            
            if fls_resized.ndim == 2:
                fls_color = cv2.applyColorMap(fls_resized, cv2.COLORMAP_JET)
                fls_final = cv2.cvtColor(fls_color, cv2.COLOR_BGR2RGB)
            else:
                fls_final = fls_resized

            img_tk = ImageTk.PhotoImage(Image.fromarray(fls_final))
            self.ImgPlaceholder_Sonar.configure(image=img_tk)
            self.ImgPlaceholder_Sonar.image = img_tk

    def show_img_Trajectory(self):
        """Draws top-down trajectory map."""
        if not hasattr(self.listener_node, 'POSITION') or self.listener_node.POSITION is None:
            return
        
        pos = self.listener_node.POSITION
        if len(pos) < 2: return
        
        # Origin Setup
        if self.origin_pos is None:
            self.origin_pos = (pos[0], pos[1])
            print(f"Origin set at: {self.origin_pos}")

        rel_x = pos[0] - self.origin_pos[0]
        rel_y = pos[1] - self.origin_pos[1]

        center = self.map_size // 2
        
        # Coordinate Mapping:
        # Map X (Screen Horizontal) <-> Robot Y (Left/Right)
        px_x = int(center - (rel_y * self.map_scale)) 
        # Map Y (Screen Vertical)   <-> Robot X (Fwd/Back)
        px_y = int(center - (rel_x * self.map_scale))
        
        current_px = (px_x, px_y)

        # Draw Line (History)
        if self.last_pos_px is not None:
            cv2.line(self.map_background, self.last_pos_px, current_px, (0, 0, 0), 1)
        
        self.last_pos_px = current_px
        
        # Draw Cursor (Current)
        display_img = self.map_background.copy()
        cv2.circle(display_img, current_px, 5, (0, 0, 255), -1) 

        display_rgb = cv2.cvtColor(display_img, cv2.COLOR_BGR2RGB)
        traj_tk = ImageTk.PhotoImage(Image.fromarray(display_rgb))

        self.ImgPlaceholder_Trajectory.configure(image=traj_tk)
        self.ImgPlaceholder_Trajectory.image = traj_tk
        
def main():
    rclpy.init()
    control_node = ros_nodes.StonefishPublisher()
    listener_node = ros_nodes.StonefishSubscriber() 

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(control_node)
    executor.add_node(listener_node)

    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    master = Panel(root, control_node, listener_node) 
    
    try:
        root.mainloop()
    finally:
        control_node.destroy_node()
        listener_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
#____

# import sys 
# from threading import Thread
# import rclpy
# from rclpy.node import Node
# import rclpy.executors 

# import ros_nodes
# import numpy as np 
# import cv2

# from PIL import Image, ImageTk


# import tkinter as tk
# import os
# os.environ['DISPLAY'] = ':23'

# class Panel:
#     def __init__(self, root, publisher_node, listener_node):
#         self.root = root
#         self.publisher_node = publisher_node
#         self.listener_node = listener_node

#         self.root.title("ROV Control Panel")
#         self.root.geometry("900x600") # Zwiększyłem nieco wysokość okna

#         # --- Trajectory State ---
#         self.map_size = 300     
#         self.map_scale = 20     # [px/m]
        
#         # map background
#         self.map_background = np.ones((self.map_size, self.map_size, 3), dtype=np.uint8) * 255
        
#         self.last_pos_px = None 
#         self.origin_pos = None  # start pose

#         # --- Control State ---
#         self.shift_state = [0.0, 0.0, 0.0]
#         self.rotate_state = [0.0, 0.0, 0.0]
#         self.power = 40.0 # Scale factor - power

#         # --- Layout Configuration ---
#         self.root.columnconfigure(0, weight=1)
#         self.root.columnconfigure(1, weight=1) 
#         self.root.columnconfigure(2, weight=1)
#         self.root.rowconfigure(1, weight=1) 
        
#         # Labels
#         tk.Label(root, text="TRAJEKTORIA (Top-Down)", font=('Arial', 10, 'bold')).grid(row=0, column=0, pady=5)
#         tk.Label(root, text="STEROWANIE", font=('Arial', 10, 'bold')).grid(row=0, column=1, pady=5)
#         tk.Label(root, text="SONAR FLS", font=('Arial', 10, 'bold')).grid(row=0, column=2, pady=5)
        
#         # 1. Trjectory map
#         self.ImgPlaceholder_Trajectory = tk.Label(root, bg="white", relief=tk.SUNKEN)
#         self.ImgPlaceholder_Trajectory.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

#         # 2. Controls
#         control_frame = tk.Frame(root, bd=2, relief=tk.GROOVE)
#         control_frame.grid(row=1, column=1, sticky="nsew", padx=5, pady=10)
        
#         # Control panel grid - layout 
#         control_frame.columnconfigure(0, weight=1)
#         control_frame.columnconfigure(1, weight=1)

#         # --- Forward ---
#         btn_fwd = tk.Button(control_frame, text='▲\nForward', bg="#d1e7dd", height=2)
#         btn_fwd.grid(row=0, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
#         btn_fwd.bind('<ButtonPress-1>', lambda e: self.set_shift_x(self.power))
#         btn_fwd.bind('<ButtonRelease-1>', lambda e: self.set_shift_x(0.0))

#         # --- Left / Right ---
#         btn_left = tk.Button(control_frame, text='◀ Left', bg="#d1e7dd", height=2)
#         btn_left.grid(row=1, column=0, sticky="ew", padx=5, pady=5)
#         btn_left.bind('<ButtonPress-1>', lambda e: self.set_shift_y(self.power)) # Y+ to lewo w ROS
#         btn_left.bind('<ButtonRelease-1>', lambda e: self.set_shift_y(0.0))

#         btn_right = tk.Button(control_frame, text='Right ▶', bg="#d1e7dd", height=2)
#         btn_right.grid(row=1, column=1, sticky="ew", padx=5, pady=5)
#         btn_right.bind('<ButtonPress-1>', lambda e: self.set_shift_y(-self.power)) # Y- to prawo w ROS
#         btn_right.bind('<ButtonRelease-1>', lambda e: self.set_shift_y(0.0))

#         # --- Backward ---
#         btn_back = tk.Button(control_frame, text='▼\nBackward', bg="#d1e7dd", height=2)
#         btn_back.grid(row=2, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
#         btn_back.bind('<ButtonPress-1>', lambda e: self.set_shift_x(-self.power))
#         btn_back.bind('<ButtonRelease-1>', lambda e: self.set_shift_x(0.0))
        
#         # --- UP / DOWN (NOWE PRZYCISKI) ---
#         # Zależnie od konfiguracji silników: Z+ to góra, Z- to dół (lub odwrotnie)
#         btn_up = tk.Button(control_frame, text='⇪ UP (Z+)', bg="#fff3cd", height=2)
#         btn_up.grid(row=3, column=0, sticky="ew", padx=5, pady=5)
#         btn_up.bind('<ButtonPress-1>', lambda e: self.set_shift_z(self.power))
#         btn_up.bind('<ButtonRelease-1>', lambda e: self.set_shift_z(0.0))

#         btn_down = tk.Button(control_frame, text='⇩ DOWN (Z-)', bg="#fff3cd", height=2)
#         btn_down.grid(row=3, column=1, sticky="ew", padx=5, pady=5)
#         btn_down.bind('<ButtonPress-1>', lambda e: self.set_shift_z(-self.power))
#         btn_down.bind('<ButtonRelease-1>', lambda e: self.set_shift_z(0.0))

#         # Info label
#         self.lbl_status = tk.Label(control_frame, text="Waiting for telemetry...", fg="gray")
#         self.lbl_status.grid(row=4, column=0, columnspan=2, pady=20) # Przesunięte do row 4

#         # Sonar display
#         self.ImgPlaceholder_Sonar = tk.Label(root, bg="black", relief=tk.SUNKEN)
#         self.ImgPlaceholder_Sonar.grid(row=1, column=2, padx=10, pady=10, sticky="nsew")

#         # Start loop
#         self.update_all_displays()

#     # --- Control Logic ---
#     def new_event(self):
#        # send ross cmd
#        self.publisher_node.send_cmd(self.shift_state, self.rotate_state)
       
#     def set_shift_x(self, val):
#         self.shift_state[0] = val
#         self.new_event()
#     def set_shift_y(self, val):
#         self.shift_state[1] = val
#         self.new_event()
    
#     # NOWA FUNKCJA DO OSI Z
#     def set_shift_z(self, val):
#         self.shift_state[2] = val
#         self.new_event()

#     # --- Display Logic ---
#     def update_all_displays(self):
#         if self.listener_node:
#             self.show_img_Sonar()
#             self.show_img_Trajectory()
        
#         # Refreshing
#         self.root.after(100, self.update_all_displays)
        
#     def show_img_Sonar(self):
#         if hasattr(self.listener_node, 'FLS') and self.listener_node.FLS is not None:
#             fls_data = self.listener_node.FLS
            
#             # Scale
#             fls_resized = cv2.resize(fls_data, (300, 300), interpolation=cv2.INTER_NEAREST)
            
#             # What is that? idk - chat did it
#             if fls_resized.ndim == 2:
#                 fls_color = cv2.applyColorMap(fls_resized, cv2.COLORMAP_JET)
#                 fls_final = cv2.cvtColor(fls_color, cv2.COLOR_BGR2RGB)
#             else:
#                 fls_final = fls_resized

#             img_tk = ImageTk.PhotoImage(Image.fromarray(fls_final))
#             self.ImgPlaceholder_Sonar.configure(image=img_tk)
#             self.ImgPlaceholder_Sonar.image = img_tk

#     def show_img_Trajectory(self):
#         # Check if data available
#         if not hasattr(self.listener_node, 'POSITION') or self.listener_node.POSITION is None:
#             self.lbl_status.configure(text="Waiting for ROS connection...", fg="red")
#             return
#         # Get pose [x, y, z]
#         pos = self.listener_node.POSITION

#         if len(pos) < 2: return
        
#         # Check if it non zero pose
#         if np.all(pos == 0) and self.origin_pos is None:
#             self.lbl_status.configure(text="Waiting for movement...", fg="orange")
#             # return # Odkomentuj jeśli chcesz czekać na ruch

#         # --- Set origin to center of map ---
#         if self.origin_pos is None:
#             self.origin_pos = (pos[0], pos[1])
#             self.lbl_status.configure(text="System ARMED. Origin set.", fg="green")
#             print(f"Origin set at: {self.origin_pos}")

#         # Calc relative pose
#         rel_x = pos[0] - self.origin_pos[0]
#         rel_y = pos[1] - self.origin_pos[1]

#         # Convert to pixels
#         # Centrum mapy to (map_size/2, map_size/2)
#         # ROS X+ (Przód) -> UI Y- (Góra)
#         # ROS Y+ (Lewo)  -> UI X- (Lewo)  <- Tutaj zależy od konwencji, zazwyczaj Y w lewo to X w lewo na ekranie
        
#         center = self.map_size // 2
        
#         # map X axis <=> Y axis of robot (left/right)
#         px_x = int(center - (rel_y * self.map_scale)) 
        
#         # map Y axis <=> X axis of robot (forward/backward)
#         px_y = int(center - (rel_x * self.map_scale))

#         current_px = (px_x, px_y)

#         # 1. Draw line
#         if self.last_pos_px is not None:
#             cv2.line(self.map_background, self.last_pos_px, current_px, (0, 0, 0), 1)
        
#         self.last_pos_px = current_px

#         display_img = self.map_background.copy()

#         # draw robot actual pose
#         cv2.circle(display_img, current_px, 5, (0, 0, 255), -1) # BGR: Red

#         # Convert to PIL
#         display_rgb = cv2.cvtColor(display_img, cv2.COLOR_BGR2RGB)
#         traj_tk = ImageTk.PhotoImage(Image.fromarray(display_rgb))

#         self.ImgPlaceholder_Trajectory.configure(image=traj_tk)
#         self.ImgPlaceholder_Trajectory.image = traj_tk
        
#         # Debug info
#         self.lbl_status.configure(text=f"Pos: X={rel_x:.2f}m Y={rel_y:.2f}m")

        
# def main():
#     rclpy.init()
    
#     control_node = ros_nodes.StonefishPublisher()
#     listener_node = ros_nodes.StonefishSubscriber() 

#     executor = rclpy.executors.SingleThreadedExecutor()
#     executor.add_node(control_node)
#     executor.add_node(listener_node)

#     # Start ROS w osobnym wątku
#     spin_thread = Thread(target=executor.spin, daemon=True)
#     spin_thread.start()

#     # Init GUI
#     root = tk.Tk()
#     master = Panel(root, control_node, listener_node) 
    
#     try:
#         root.mainloop()
#     finally:
#         # Graceful shutdown
#         control_node.destroy_node()
#         listener_node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()