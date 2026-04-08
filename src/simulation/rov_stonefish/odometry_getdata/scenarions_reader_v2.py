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

        self.depth_regulator = PID(P=5, I=0.05, D=12, out_min=-40, out_max=40, dt=0.5)
        self.t1 = time.time() # for regulator purpose
        
        # Init depth target
        self.current_target_depth = (self.boundaries['min_depth'] + self.boundaries['max_depth']) / 2.0

        if self.determinist:
            if not scenario_pth or not os.path.exists(scenario_pth):
                raise FileNotFoundError("Scenario file not found provided.")
            tree = ET.parse(scenario_pth)
            root = tree.getroot()
            
            self.durations = [float(act.find('time').text) for act in root.findall('cmd')]
            self.actions = [act.find('action').text for act in root.findall('cmd')]
            self.values = [float(act.find('value').text) for act in root.findall('cmd')]

        else: 
            self.durations = []
            self.actions = []
            self.values = [] 

            # Lista konkretnych, nazwanych manewrów
            maneuvers = ['straight', 'curve', 'circle', 'pure_turn', 'slide']

            for _ in range(mv_count):
                # Losujemy manewr z przypisanymi wagami (prawdopodobieństwem)
                maneuver = random.choices(
                    maneuvers, 
                    weights=[0.15, 0.30, 0.4, 0.10, 0.05], 
                    k=1
                )[0]
                
                surge = 0.0
                sway = 0.0
                yaw_torque = 0.0
                
                if maneuver == 'straight':
                    surge = random.uniform(-self.boundaries['F_max'], self.boundaries['F_max'])
                    duration = random.uniform(10.0, 20.0)
                
                elif maneuver == 'curve':
                    surge = random.uniform(-self.boundaries['F_min'], self.boundaries['F_max'])
                    yaw_torque = random.uniform(-self.boundaries['T_max'], self.boundaries['T_max']) * 0.4 
                    duration = random.uniform(10.0, 15.0)
                    
                elif maneuver == 'circle':
                    
                    surge = 9.0  
                    
                    yaw_dir = random.choice([-1, 1])
                    yaw_torque = yaw_dir * self.boundaries['T_max'] * 0.2 # Używa pełnych 2.0 Nm z configu
                    
                    duration = random.uniform(3.0, 10.0)

                elif maneuver == 'pure_turn':
                    yaw_torque = random.choice([-1, 1]) * random.uniform(self.boundaries['T_max']*0.6, self.boundaries['T_max'])
                    duration = random.uniform(2.0, 4.0)
                    
                elif maneuver == 'slide':
                    sway = random.choice([-1, 1]) * random.uniform(self.boundaries['F_min'], self.boundaries['F_max'])
                    duration = random.uniform(5.0, 10.0)

                # 20 % szans na zmianę docelowej głębokości podczas nowego manewru
                if random.random() > 0.8:
                    target_depth = random.uniform(self.boundaries['max_depth'], self.boundaries['min_depth']) 
                else:
                    target_depth = None # Utrzymaj poprzednią głębokość
                
                self.durations.append(duration)
                self.actions.append({
                    'type': maneuver,
                    'surge': surge,
                    'sway': sway,
                    'yaw': yaw_torque,
                    'depth': target_depth
                })
                self.values.append(0) 

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
        """Detrministic mode"""
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
            cmd_shift[0] = 25      
            cmd_rotate[2] = -0.2
        elif action == 'circle_left':
            cmd_shift[0] = 30
            cmd_rotate[2] = 0.2

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
        if dvl_vel is None: dvl_vel = np.zeros(3)
        if dvl_alt is None: dvl_alt = 0.0
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

        # Target state
        current_shift = [0.0, 0.0, 0.0]
        current_rotate = [0.0, 0.0, 0.0]
        alpha = 0.1 # coefficient for low-pass filter for low changes rate, make moves more smooth 

        while samples_collected < target_samples_num:
        
            current_action_idx = action_idx_pointer % len(self.actions)
            action_duration = self.durations[current_action_idx]

            # Logger and target updates
            if not self.determinist:
                action_data = self.actions[current_action_idx]
                if action_data['depth'] is not None:
                    self.current_target_depth = action_data['depth']
                print(f"[Action Loop] Executing '{action_data['type']}' for {action_duration:.1f}s. Target Depth: {self.current_target_depth:.2f}m. Samples: {samples_collected}/{target_samples_num}")
            else:
                action_type = self.actions[current_action_idx]
                action_val = self.values[current_action_idx]
                print(f"[Action Loop] {action_type} (val={action_val:.1f}) for {action_duration:.1f}s. Samples: {samples_collected}/{target_samples_num}")

            action_idx_pointer += 1
            action_start_time = time.time()

            while (time.time() - action_start_time) < action_duration:

                if self.sub_node.new_data_event.is_set():

                    obs = self.get_obs()
                    if obs is None:
                        continue
                    
                    self.save_step_data(samples_collected, obs)
                    samples_collected += 1

                    self.sub_node.new_data_event.clear()
                    if samples_collected >= target_samples_num:
                        break

                    current_z = obs['position_full'][2]

                    if not self.determinist:
                        target_shift = [action_data['surge'], action_data['sway'], 0.0]
                        target_rotate = [0.0, 0.0, action_data['yaw']]
                    else:
                        target_shift, target_rotate = self._map_action_to_wrench(action_type, action_val)

         

                    # 2. Movement smoothing - low-pass filter
                    current_shift[0] += alpha * (target_shift[0] - current_shift[0])
                    current_shift[1] += alpha * (target_shift[1] - current_shift[1])
                    current_rotate[2] += alpha * (target_rotate[2] - current_rotate[2])

                    # 3. Depth regulator
                    if time.time() - self.t1 > self.depth_regulator.dt: 
                        self.t1 = time.time()
                        current_shift[2] = self.depth_regulator.out(current_z, self.current_target_depth)

                    self.pub_node.send_cmd(current_shift, current_rotate)

                else:
                    time.sleep(0.002)

                
class PID:
    def __init__(self, P, I, D, out_min, out_max, dt):
        self.P, self.I, self.D = P, I, D
        self.out_min, self.out_max = out_min, out_max
        self.sum = 0 
        self.prev_e = 0 
        self.dt = dt

    def out(self, current_val, target):
        e = current_val - target
        
        out_P = e * self.P
        
        # Anti-windup 
        if abs(e) < 2.0: 
            self.sum += e * self.I * self.dt
            self.sum = max(min(self.sum, 10), -10)

        out_D = (e - self.prev_e) / self.dt if self.dt > 0 else 0
        self.prev_e = e

        output = out_P + (out_D * self.D) + self.sum
        return max(min(output, self.out_max), self.out_min)