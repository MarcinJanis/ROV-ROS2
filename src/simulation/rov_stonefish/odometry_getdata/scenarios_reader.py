import numpy as np
import os
import random
import csv
import cv2
import time
import xml.etree.ElementTree as ET

class MasterController:
    '''
    Reads test scenario from xml or generates random moves and sends to control node.
    '''
    def __init__(self, publisher_node, listener_node, general_dir):
        self.determinist = False
        self.pub_node = publisher_node
        self.sub_node = listener_node
        
        # Control loop configuration
        # Zwiększamy rate, 1Hz to bardzo wolno dla sterowania, ale OK dla zbierania danych statycznych
        self.control_rate = 10.0  
        self.dt = 1.0 / self.control_rate

        self.root_dir = general_dir
        self.act_dir = None
        self.seq_id = None
        self.img_dir = None
        
        # Trajectories list
        self.durations = []    
        self.actions = []      
        self.values = []       

        os.makedirs(self.root_dir, exist_ok=True)
        
    def setup(self, seq_id, determinist: bool = False, scenario_pth: str = None, 
              mv_count: int = 0, boundaries: dict = None):
        """
        Generate trajectory
        """
        self.boundaries = boundaries
        self.seq_id = seq_id
        self.act_dir = os.path.join(self.root_dir, f'seq_{seq_id}')
        self.img_dir = os.path.join(self.act_dir, 'fls')
        
        os.makedirs(self.act_dir, exist_ok=True)
        os.makedirs(self.img_dir, exist_ok=True)
    
        self.determinist = determinist

        if self.determinist:
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
            
            self.durations = [random.uniform(self.boundaries['t_min'], self.boundaries['t_max']) 
                              for _ in range(mv_count)]
            self.actions = [random.choice(available_actions) for _ in range(mv_count)]
            self.values = []

            for action in self.actions:
                if 'rotate' in action:
                    val = random.uniform(self.boundaries['T_min'], self.boundaries['T_max'])
                elif 'circle' in action:
                    val = random.uniform(self.boundaries['F_min'], self.boundaries['F_max'])
                else:
                    val = random.uniform(self.boundaries['F_min'], self.boundaries['F_max'])
                
                self.values.append(val)

        # CSV init - dodajemy nagłówki dla akcji sterujących
        self.csv_file_path = os.path.join(self.act_dir, 'sequence.csv')
        with open(self.csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['step_idx', 'timestamp', 
                             'pos_x', 'pos_y', 'pos_z', 
                             'quat_x', 'quat_y', 'quat_z', 'quat_w'
                            ])

    def _map_action_to_wrench(self, action, value):
        # cmd_shift: [surge (x), sway (y), heave (z)]
        # cmd_rotate: [roll, pitch, yaw]
        cmd_shift = [0.0, 0.0, 0.0] 
        cmd_rotate = [0.0, 0.0, 0.0] 
        val = float(value)

        if action == 'forward':
            cmd_shift[0] = val
        elif action == 'backward':
            cmd_shift[0] = -val
        elif action == 'slide_left':
            cmd_shift[1] = val 
        elif action == 'slide_right':
            cmd_shift[1] = -val
        elif action == 'depth_change':
            cmd_shift[2] = val 
        elif action == 'rotate_right':
            cmd_rotate[2] = -val # Check sign convention
        elif action == 'rotate_left':
            cmd_rotate[2] = val
        elif action == 'circle_right':
            cmd_shift[0] = val         
            cmd_rotate[2] = -self.boundaries['T_max'] # Check sign
        elif action == 'circle_left':
            cmd_shift[0] = val
            cmd_rotate[2] = self.boundaries['T_max']

        return cmd_shift, cmd_rotate
    
    def get_obs(self):
        # Bezpieczny dostęp do danych z ROS
        if self.sub_node.POSITION is None:
            return None
        
        return {
            'position_full': self.sub_node.POSITION, # [x,y,z, qx,qy,qz,qw]
            'fls': self.sub_node.FLS,                
            'timestamp': self.sub_node.TIME_STAMP
        }

    # POPRAWIONA DEFINICJA: przyjmuje argumenty, które przekazujesz
    def save_step_data(self, step_idx, action_name, shift, rotate, obs_data):
        
        timestamp = obs_data.get('timestamp', 0.0)
        pos_full = obs_data.get('position_full', np.zeros(7))
        fls_img = obs_data.get('fls', None)
        
        pos_x, pos_y, pos_z = pos_full[0], pos_full[1], pos_full[2]
        quat_x, quat_y, quat_z, quat_w = pos_full[3], pos_full[4], pos_full[5], pos_full[6]

        # Zapis do CSV
        with open(self.csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                step_idx,
                f"{timestamp:.3f}", 
                f"{pos_x:.3f}", f"{pos_y:.3f}", f"{pos_z:.3f}",
                f"{quat_x:.3f}", f"{quat_y:.3f}", f"{quat_z:.3f}", f"{quat_w:.3f}"
            ])
            
        # Zapis obrazu
        if fls_img is not None and fls_img.size > 0:
            fls_img_name = f"{step_idx}.png"
            full_img_path = os.path.join(self.img_dir, fls_img_name)
            try:
                cv2.imwrite(full_img_path, fls_img)
            except Exception as e:
                print(f"[Error] Failed to save image: {e}")

    def sequence_exec(self, target_samples_num):
        """
        Main execution loop.
        """
        samples_collected = 0
        action_idx_pointer = 0 
        
        print(f"[Sequence: {self.seq_id}] Starting execution. Target samples: {target_samples_num}")
        
        while samples_collected < target_samples_num:
            
            # Pobierz parametry bieżącej akcji
            current_action_idx = action_idx_pointer % len(self.actions)
            action_type = self.actions[current_action_idx]
            action_val = self.values[current_action_idx]
            action_duration = self.durations[current_action_idx]

            print(f"[Action Loop] {action_type} (val={action_val:.1f}) for {action_duration:.1f}s. "
                  f"Samples: {samples_collected}/{target_samples_num}")

            # Wyznacz siły
            shift, rotate = self._map_action_to_wrench(action_type, action_val)
            
            # Pętla czasowa dla pojedynczej akcji
            action_start_time = time.time()
            
            while (time.time() - action_start_time) < action_duration:
                loop_start = time.time()
                
                # 1. Pobierz obserwację (ROS DATA)
                obs = self.get_obs()
                
                # Jeśli brak danych (np. ROS nie połączył), czekaj i ponów
                if obs is None:
                    time.sleep(0.05) # Ważne: oddaj procesor wątkowi ROS!
                    continue

                # 2. Sprawdzenie bezpieczników (Safety Limits)
                current_z = obs['position_full'][2]
                
                # Zabezpieczenie przed uderzeniem w dno/zbyt dużą głębokością
                # Nadpisuje sterowanie w osi Z, jeśli jest za głęboko
                safe_shift = list(shift) # Kopia, żeby nie modyfikować oryginału permanentnie
                if current_z > self.boundaries['max_depth']:   
                    safe_shift[2] = 80.0 # Wymuszenie wypłynięcia (siła w górę)
                    print(f"Warning: Max depth reached ({current_z:.2f}m). Surfacing force applied.")

                # 3. Zapis danych
                self.save_step_data