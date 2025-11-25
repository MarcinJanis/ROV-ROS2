
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

        # --- Trajectory State ---
        # Map size 200x200 (pixels) with white background (RGB)
        self.trajectory = np.ones((200, 200, 3), dtype = np.uint8) * 255 
        self.last_pos_px = None 
        self.map_scale = 20     # Scale: 20 pixels per 1 meter ROS
        self.map_size = 200     

        self.root.title("ROV Control Panel and Telemetry")
        self.root.geometry("800x500")

        # --- Control ---
        self.shift_state = [0.0, 0.0, 0.0]
        self.rotate_state = [0.0, 0.0, 0.0]
        self.power = 0.5

        # --- Grid configuration ---
        # Column 0: Trajectory (Weight 2)
        # Column 1: Controls (Weight 1)
        # Column 2: Sonar (Weight 2)
        self.root.columnconfigure(0, weight=2)
        self.root.columnconfigure(1, weight=1) 
        self.root.columnconfigure(2, weight=2)
        
        # Row 0: Labels (Low weight)
        # Row 1: Displays and Controls (High weight)
        self.root.rowconfigure(0, weight=0) 
        self.root.rowconfigure(1, weight=1) 
        
        # --- Etykiety nad wyświetlaczami (Row 0) ---
        tk.Label(root, text="TRAJEKTORIA (2D)", font=('Arial', 10, 'bold')).grid(row=0, column=0, pady=(5, 0))
        tk.Label(root, text="SONAR FLS", font=('Arial', 10, 'bold')).grid(row=0, column=2, pady=(5, 0))
        
        
        # --- TRAJECTORY DISPLAY (Kolumna 0, Row 1) ---
        self.ImgPlaceholder_Trajectory = tk.Label(
            root, 
            text="Trajectory Map - Waiting for Data", 
            bg="gray", 
            width=200,
            height=200,
            relief=tk.SUNKEN 
        )
        self.ImgPlaceholder_Trajectory.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")


        # --- FLS measurements display (Kolumna 2, Row 1) ---
        self.ImgPlaceholder_Sonar = tk.Label(
            root, 
            text="Forward Looking Sonar - Waiting for Data", 
            bg="gray", 
            width=200,
            height=200,
            relief=tk.SUNKEN
        )
        self.ImgPlaceholder_Sonar.grid(row=1, column=2, padx=10, pady=10, sticky="nsew")


        # --- PRZYCISKI W KONTENERZE (Kolumna 1, Row 1) ---
        control_frame = tk.Frame(root, padx=10, pady=10)
        # Umieszczamy ramkę w rzędzie 1 i kolumnie 1
        control_frame.grid(row=1, column=1, sticky="nsew") 
        control_frame.columnconfigure(0, weight=1)
        control_frame.columnconfigure(1, weight=1)

        # Forward (Row 0 wewnątrz control_frame)
        btn_forward = tk.Button(control_frame, text='Forward (X+)', bg="#E0E0FF")
        btn_forward.grid(row=0, column=0, columnspan=2, sticky="ew", pady=5)
        btn_forward.bind('<ButtonPress-1>', lambda event: self.set_shift_x(self.power))
        btn_forward.bind('<ButtonRelease-1>', lambda event: self.set_shift_x(0.0))

        # Left / Right (Row 1 wewnątrz control_frame)
        btn_left = tk.Button(control_frame, text='Left (Y+)', bg="#E0E0FF")
        btn_left.grid(row=1, column=0, sticky="ew", padx=(0, 2), pady=5)
        btn_left.bind('<ButtonPress-1>', lambda event: self.set_shift_y(self.power)) 
        btn_left.bind('<ButtonRelease-1>', lambda event: self.set_shift_y(0.0))

        btn_right = tk.Button(control_frame, text='Right (Y-)', bg="#E0E0FF")
        btn_right.grid(row=1, column=1, sticky="ew", padx=(2, 0), pady=5)
        btn_right.bind('<ButtonPress-1>', lambda event: self.set_shift_y(-self.power))
        btn_right.bind('<ButtonRelease-1>', lambda event: self.set_shift_y(0.0))

        # Backward (Row 2 wewnątrz control_frame)
        btn_backward = tk.Button(control_frame, text='Backward (X-)', bg="#E0E0FF")
        btn_backward.grid(row=2, column=0, columnspan=2, sticky="ew", pady=5)
        btn_backward.bind('<ButtonPress-1>', lambda event: self.set_shift_x(-self.power))
        btn_backward.bind('<ButtonRelease-1>', lambda event: self.set_shift_x(0.0))


        # --- URUCHOMIENIE PĘTLI ODŚWIEŻANIA ---
        self.update_all_displays()


    # --- Control Fcns ---
    def new_event(self):
        self.publisher_node.send_cmd(self.shift_state, self.rotate_state)
    def set_shift_x(self, val:float):
        self.shift_state[0] = val
        self.new_event()
    def set_shift_y(self, val:float):
        self.shift_state[1] = val
        self.new_event()
    def set_shift_z(self, val:float):
        self.shift_state[2] = val
        self.new_event()
    def set_rotate_x(self, val:float):
        self.rotate_state[0] = val
        self.new_event()
    def set_rotate_y(self, val:float):
        self.rotate_state[1] = val
        self.new_event()
    def set_rotate_z(self, val:float):
        self.rotate_state[2] = val
        self.new_event()
        
    
    # --- Odświeżanie i Wyświetlanie ---

    def update_all_displays(self):
        """
        Główna pętla odświeżająca GUI.
        Wywoływana przez root.after() raz na sekundę (1000 ms).
        """
        if self.listener_node:
            self.show_img_Sonar()
            self.show_img_Trajectory()

        # Zaplanuj kolejne wywołanie za 1000 milisekund (1 sekunda)
        self.root.after(1000, self.update_all_displays)
        
    def show_img_Sonar(self):
        """Pobiera i wyświetla obraz z Sonaru (FLS), obsługując skalę szarości."""
        
        if hasattr(self.listener_node, 'FLS') and isinstance(self.listener_node.FLS, np.ndarray):
            fls_np = self.listener_node.FLS
            
            # Wymuszamy rozmiar 200x200
            fls_resized = cv2.resize(fls_np, (200, 200))
            
            # POPRAWKA: Konwersja kolorów dla różnych typów obrazów
            if fls_resized.ndim == 2:
                # Obraz 1-kanałowy (skala szarości) -> BGR
                fls_resized = cv2.cvtColor(fls_resized, cv2.COLOR_GRAY2BGR)
            
            if fls_resized.ndim == 3 and fls_resized.shape[2] == 3:
                # BGR -> RGB dla PIL/Tkinter
                fls_resized = cv2.cvtColor(fls_resized, cv2.COLOR_BGR2RGB)
            
            fls_pil = Image.fromarray(fls_resized) 
            fls_tk = ImageTk.PhotoImage(fls_pil)
            
            self.ImgPlaceholder_Sonar.configure(image=fls_tk, text="")
            self.ImgPlaceholder_Sonar.image = fls_tk # Utrzymanie referencji

    def show_img_Trajectory(self):
        """Rysuje trajektorię na mapie i ją wyświetla."""
        
        # 1. POBRANIE POZYCJI
        if not (hasattr(self.listener_node, 'POSITION') and 
                isinstance(self.listener_node.POSITION, (list, tuple, np.ndarray)) and
                len(self.listener_node.POSITION) >= 2):
             return

        x_m, y_m, *rest = self.listener_node.POSITION
        
        # 2. KONWERSJA METRY -> PIKSELE (Mapowanie X/Y ROS na wiersze/kolumny mapy)
        x_px = int(self.map_size / 2 - y_m * self.map_scale) 
        y_px = int(self.map_size / 2 - x_m * self.map_scale) 
        
        current_pos_px = (x_px, y_px)
        
        # Ograniczanie do granic mapy
        if 0 <= x_px < self.map_size and 0 <= y_px < self.map_size:
            # 3. Rysowanie linii
            if self.last_pos_px is not None:
                cv2.line(self.trajectory, self.last_pos_px, current_pos_px, color=(0, 0, 0), thickness=1)
            
            # 4. Rysowanie aktualnej pozycji (Czerwony punkt - BGR)
            cv2.circle(self.trajectory, 
                       current_pos_px, 
                       radius = 3, 
                       color = (0, 0, 255), 
                       thickness = -1 )
            
            # 5. Aktualizacja ostatniej pozycji
            self.last_pos_px = current_pos_px
        
        # 6. Wyświetlanie obrazu
        traj_pil = Image.fromarray(self.trajectory, 'RGB')
        traj_tk = ImageTk.PhotoImage(traj_pil)
        
        self.ImgPlaceholder_Trajectory.configure(image = traj_tk, text="")
        self.ImgPlaceholder_Trajectory.image = traj_tk

        
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

