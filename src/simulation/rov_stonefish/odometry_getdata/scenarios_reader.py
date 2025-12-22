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
        
        # self.control_rate = 2  
        # self.dt = 1.0 / self.control_rate

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

        # CSV init 
        self.csv_file_path = os.path.join(self.act_dir, 'sequence.csv')
        with open(self.csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['step_idx', 'timestamp', 
                             'pos_x', 'pos_y', 'pos_z', 
                             'quat_x', 'quat_y', 'quat_z', 'quat_w', 
                             'dvl_vel_x', 'dvl_vel_y', 'dvl_vel_z', 'dvl_alt',
                             'imu_orient_x', 'imu_orient_y', 'imu_orient_z', 'imu_orient_w',
                             'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z', 
                             'imu_accel_x', 'imu_accel_y', 'imu_accel_z'
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
            cmd_rotate[2] = -val 
        elif action == 'rotate_left':
            cmd_rotate[2] = val
        elif action == 'circle_right':
            cmd_shift[0] = val         
            cmd_rotate[2] = -self.boundaries['T_max'] 
        elif action == 'circle_left':
            cmd_shift[0] = val
            cmd_rotate[2] = self.boundaries['T_max']

        return cmd_shift, cmd_rotate
    
    def get_obs(self):
        if self.sub_node.POSITION is None:
            return None
        return {
            'timestamp': self.sub_node.TIME_STAMP, 
            'position_full': self.sub_node.POSITION, # [x,y,z, qx,qy,qz,qw]
            'fls': self.sub_node.FLS,                
            'dvl':self.sub_node.DVL_VEL,
            'altitude':self.sub_node.DVL_ALTITUDE,
            'imu':self.sub_node.IMU
        }

    def save_step_data(self, step_idx, obs_data):
        
        timestamp = obs_data.get('timestamp', 0.0)
        pos_full = obs_data.get('position_full', np.zeros(7))
        fls_img = obs_data.get('fls', None)
        dvl_vel = obs_data.get('dvl', np.zeros(3))
        dvl_alt = obs_data.get('altitude', 0.0)
        imu = obs_data.get('imu', np.zeros(10))
        
        if pos_full is None: pos_full = np.zeros(7)
        dvl_vel = obs_data.get('dvl')
        if dvl_vel is None: dvl_vel = np.zeros(3)
        dvl_alt = obs_data.get('altitude')
        if dvl_alt is None: dvl_alt = 0.0
        imu = obs_data.get('imu')
        if imu is None: imu = np.zeros(10)

        pos_x, pos_y, pos_z = pos_full[0], pos_full[1], pos_full[2]
        quat_x, quat_y, quat_z, quat_w = pos_full[3], pos_full[4], pos_full[5], pos_full[6]


        # CSV Write
        with open(self.csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                step_idx,
                f"{timestamp:.3f}", 
                f"{pos_x:.3f}", f"{pos_y:.3f}", f"{pos_z:.3f}",
                f"{quat_x:.3f}", f"{quat_y:.3f}", f"{quat_z:.3f}", f"{quat_w:.3f}", 
                f"{dvl_vel[0]:.3f}", f"{dvl_vel[1]:.3f}", f"{dvl_vel[2]:.3f}", f"{dvl_alt:.3f}",
                f"{imu[0]:.3f}", f"{imu[1]:.3f}", f"{imu[2]:.3f}", f"{imu[3]:.3f}", f"{imu[4]:.3f}", f"{imu[5]:.3f}", f"{imu[6]:.3f}", f"{imu[7]:.3f}", f"{imu[8]:.3f}", f"{imu[9]:.3f}"
            ])
            
        # FLS img write
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
        obs = None

        while samples_collected < target_samples_num:
        
            current_action_idx = action_idx_pointer % len(self.actions)
            action_type = self.actions[current_action_idx]
            action_val = self.values[current_action_idx]
            action_duration = self.durations[current_action_idx]

            action_idx_pointer += 1
            print(f"[Action Loop] {action_type} (val={action_val:.1f}) for {action_duration:.1f}s. "
                  f"Samples: {samples_collected}/{target_samples_num}")

            shift, rotate = self._map_action_to_wrench(action_type, action_val)
            self.pub_node.send_cmd(shift, rotate)
           
            action_start_time = time.time()

            while (time.time() - action_start_time) < action_duration:

                # loop_start = time.time()
                if self.sub_node.new_data_event.is_set():

                    obs = self.get_obs()
                    self.save_step_data(samples_collected, obs)
                    samples_collected += 1

                    self.sub_node.new_data_event.clear()
                    if samples_collected >= target_samples_num:
                        break

                    # Depth limitation
                    current_z = obs['position_full'][2]
                    safe_shift = list(shift) 
                    if current_z > self.boundaries['max_depth']:   
                        safe_shift[2] = 80.0 
                        self.pub_node.send_cmd(safe_shift, rotate)
                    elif current_z < self.boundaries['min_depth']:   
                        safe_shift[2] = -40
                        self.pub_node.send_cmd(safe_shift, rotate)
                    else:
                        self.pub_node.send_cmd(shift, rotate)

                else:
                    time.sleep(0.002)

                
                  