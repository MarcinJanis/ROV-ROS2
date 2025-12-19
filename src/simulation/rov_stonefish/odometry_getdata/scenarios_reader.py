import numpy as np
import os
import random
import csv
import cv2
import time
import xml.etree.ElementTree as ET
import rclpy

class MasterController:
    '''
    Reads test scenario from xml or generates random moves and sends to control node.
    '''
    def __init__(self, publisher_node, listener_node, general_dir):
        self.determinist = False

        self.pub_node = publisher_node
        self.sub_node = listener_node
        
        # Control loop configuration
        self.control_rate = 20.0  
        self.dt = 1.0 / self.control_rate

        self.root_dir = general_dir
        self.act_dir = None
        self.seq_id = None
        self.img_dir = None
        
        # Trajectories list
        self.durations = []     # Duration of each action [s] (ZMIANA: przechowujemy czas trwania, a nie harmonogram)
        self.actions = []       # Actions
        self.values = []        # Values (force/torque)

        os.makedirs(self.root_dir, exist_ok=True)
        
    def setup(self, seq_id, determinist: bool = False, scenario_pth: str = None, 
              mv_count: int = 0, boundaries: dict = None):
        """
        Generate trajectory
        """

        self.seq_id = seq_id
        self.act_dir = os.path.join(self.root_dir, f'seq_{seq_id}')
        self.img_dir = os.path.join(self.act_dir, 'fls')
        
        os.makedirs(self.act_dir, exist_ok=True)
        os.makedirs(self.img_dir, exist_ok=True)
    
        self.determinist = determinist

        if self.determinist:
            # Read from XML
            if not scenario_pth or not os.path.exists(scenario_pth):
                raise FileNotFoundError("Scenario file not found provided.")
            tree = ET.parse(scenario_pth)
            root = tree.getroot()
            
            self.durations = [float(act.find('time').text) for act in root.findall('cmd')]
            self.actions = [act.find('action').text for act in root.findall('cmd')]
            self.values = [float(act.find('value').text) for act in root.findall('cmd')]

        else: 
            available_actions = [
                'forward', 'backward', 
                'slide_left', 'slide_right', 
                'rotate_left', 'rotate_right', 
                'depth_change',
                'circle_left', 'circle_right'
            ]
            
            # Generate duration time for each move
            self.durations = [random.uniform(boundaries['t_min'], boundaries['t_max']) 
                              for _ in range(mv_count)]
            
            self.actions = [random.choice(available_actions) for _ in range(mv_count)]
            self.values = []

            for action in self.actions:
                if 'rotate' in action:
                    val = random.uniform(boundaries['T_min'], boundaries['T_max'])
                elif 'circle' in action:
                    val = random.uniform(boundaries['F_min'], boundaries['F_max'])
                else:
                    val = random.uniform(boundaries['F_min'], boundaries['F_max'])
                
                self.values.append(val)

        # CSV init
        self.csv_file_path = os.path.join(self.act_dir, 'sequence.csv')
        with open(self.csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['step_idx', 'timestamp', 
                             'pos_x', 'pos_y', 'pos_z', 
                             'quat_x', 'quat_y', 'quat_z', 'quat_w',
                            ])

    def _map_action_to_wrench(self, action, value):

        # cmd_shift: [surge (x), sway (y), heave (z)]
        # cmd_rotate: [roll, pitch, yaw]
        cmd_shift = [0.0, 0.0, 0.0] 
        cmd_rotate = [0.0, 0.0, 0.0] 

        val = float(value)

        # Simple movements
        if action == 'forward':
            cmd_shift[0] = val
        elif action == 'backward':
            cmd_shift[0] = -val
        elif action == 'slide_left':
            cmd_shift[1] = val # Check coordinate system (NED vs ENU)
        elif action == 'slide_right':
            cmd_shift[1] = -val
        elif action == 'depth_change':
            cmd_shift[2] = val 
        elif action == 'rotate_right':
            cmd_rotate[2] = val
        elif action == 'rotate_left':
            cmd_rotate[2] = -val
            
        elif action == 'circle_right':
            cmd_shift[0] = val         
            cmd_rotate[2] = boundaries['T_max']
        elif action == 'circle_left':
            cmd_shift[0] = val
            cmd_rotate[2] = boundaries['T_max']

        return cmd_shift, cmd_rotate
    
    def get_obs(self):
        if self.sub_node.POSITION is None:
            return None
        
        return {
            'position_full': self.sub_node.POSITION, # [x,y,z, qx,qy,qz,qw]
            'fls': self.sub_node.FLS,                # fls img
            'timestamp': self.sub_node.TIME_STAMP
        }

    def save_step_data(self, step_idx, action_type, shift, rotate, obs_data):
        
        # Unpacked data from dict
        timestamp = obs_data.get('timestamp', 0.0)
        pos_full = obs_data.get('position_full', np.zeros(7))
        fls_img = obs_data.get('fls', None)
        
        # Separate pos (quaterions)
        pos_x, pos_y, pos_z = pos_full[0], pos_full[1], pos_full[2]
        quat_x, quat_y, quat_z, quat_w = pos_full[3], pos_full[4], pos_full[5], pos_full[6]

        # 1. Write data to csv
        with open(self.csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                step_idx,
                timestamp, 
                pos_x, pos_y, pos_z,                     # Pozycja
                quat_x, quat_y, quat_z, quat_w           # Orientacja
                # action_type,                       
                # shift[0], shift[1], shift[2], rotate[2]  
            ])
            
        # FLS image save
        if fls_img is not None and fls_img.size > 0:
            fls_img_name = f"{step_idx}.png"
            full_img_path = os.path.join(self.img_dir, fls_img_name)
            try:
                cv2.imwrite(full_img_path, fls_img)
            except Exception as e:
                print(f"[Error] Failed to save image: {e}")

    def sequence_exec(self, target_samples_num):
        """
        Main execution loop based on sample count.
        Cycles through actions if they run out before target_samples_num is reached.
        """

        samples_collected = 0
        action_idx_pointer = 0 
        
        print(f"[Sequence: {self.seq_id}] Starting execution. Target samples: {target_samples_num}")
        
        # Main loop
        while samples_collected < target_samples_num:
            
            current_action_idx = action_idx_pointer % len(self.actions)
            
            action_type = self.actions[current_action_idx]
            action_val = self.values[current_action_idx]
            action_duration = self.durations[current_action_idx]

            print(f"[Action Loop] Action: {action_type} (val={action_val:.2f}) for {action_duration:.2f}s. "
                  f"Samples: {samples_collected}/{target_samples_num}")

          
            action_start_time = time.time()
            
            while (time.time() - action_start_time) < action_duration:
                loop_start = time.time()
                
                # 1. Map and send command
                shift, rotate = self._map_action_to_wrench(action_type, action_val)
                self.pub_node.send_cmd(shift, rotate)

                # 2. Get data
                obs = self.get_obs()
                
                # 3. Write data & Increment sample counter
                if obs:
                    self.save_step_data(samples_collected, action_type, shift, rotate, obs)
                    samples_collected += 1
                    
                    # Log postępu co 100 próbek (opcjonalnie)
                    if samples_collected % 100 == 0:
                         print(f" -> Collected {samples_collected} samples...")

                    # Exit condition
                    if samples_collected >= target_samples_num:
                        break
                  
                # Wait for next iter to keep update frequecny 
                elapsed = time.time() - loop_start
                if elapsed < self.dt:
                    time.sleep(self.dt - elapsed)

            action_idx_pointer += 1
        
        # Finish task
        print(f"[Sequence {self.seq_id}] Finished. Stopping robot.")
        self.pub_node.send_cmd([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        print(f"[Sequence {self.seq_id}] Total samples collected: {samples_collected}.")