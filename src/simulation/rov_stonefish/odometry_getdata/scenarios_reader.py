import numpy as np
import math
import os 
import random 

import time
import rclpy
from threading import Thread

import ros_nodes
import xml.etree.ElementTree as ET

class MasterController:
  '''
  Reads test scenario from xml or generates random moves and send to control node. 
  '''
  def __init__(self, publisher_node, listener_node, general_dir):
    self.determinist = False

    self.pub_node = publisher_node
    self.sub_node = listener_node
    
    # control loop config 
    self.control_rate = 20.0  
    self.dt = 1.0 / self.control_rate

    self.time = []
    self.action = []
    self.value = []

    self.root_dir = general_directory
    self.act_dir = None
    self.seq_id = None
    
    os.makedirs(self.root_dir, exist_ok=True)
    
  def setup(self, determinist = False: bool, scenario_pth = None: str, mv_count = 0: int, boundaries = None: dict, seq_id):
      self.seq_id = seq_id
      self.act_dir = os.path.join(self.root_dir, f'seq_{seq_id}')
  
      self.determinist = determinist
      self.boundaries = boundaries 

      if self.determinist:
        ''' to read elements from xml with struct
        <cmd>
          <time>10</time> <!-- after 10 s from simulation start -->
          <action> forward </action> <!-- forward -> forward/backward; side -> left/right; rotation -> rotate right/left; depth -->
          <value> 5 </value> <!-- value in [N] / [Nm]
        </cmd>

        '''
        tree = ET.parse(scenario_pth)
        root = tree.getroot()
        self.time = [act.get('time') for act in root.findall('Cmd')]
        self.action = [act.get('action') for act in root.findall('Cmd')]
        self.value = [act.get('value') for act in root.findall('Cmd')]

      else: 
        '''
        boundaries = {
          't_min':1.0 # [s] min time of duration of some action
          't_max':10.0 # [s] max time of duration of some action
          'v_min': 1 # min value for some action
          'v_max': 5 # max value for some action
        }
        '''
        time = np.array([random.uniform(boundaries['t_min'], boundaries['t_max']) for _ in range(mv_count)], dtype = np.float32)
        actions_list = ['forward', 'side', 'rotation', 'depth']
        
        self.time = np.cumsum(time)
        self.action = [action_list[random.randint(0, len(action_list)] for _ in range(mv_count)]
        self.value = [random.uniform(boundaries['v_min'], boundaries['v_max']) for _ in range(mv_count)]
       
  def _map_action_to_wrench(self, action, value):
        """Pomocnicza funkcja zamieniająca 'forward' na wektory sił"""
        cmd_shift = [0.0, 0.0, 0.0]  # x, y, z
        cmd_rotate = [0.0, 0.0, 0.0] # roll, pitch, yaw

        if action == 'forward':
            cmd_shift[0] = float(value) # Forward/ backward
        elif action == 'side':
            cmd_shift[1] = float(value) # Right/Left
        elif action == 'depth':
            cmd_shift[2] = float(value) # Up/down
        elif action == 'rotation':
            cmd_rotate[2] = float(value) # Yaw
        
        return cmd_shift, cmd_rotate
    
def sequence_exec(self):
        """main exec loop"""

        data_idx = 0
        #TODO:
        # > init csv to save pos, time steps etc
        # > init dir to save fls
        # > make save mechanizm (in self.act_dir )



        print(f"[Sequence: {self.seq_id}] Starting sequence execution...")
        
        start_time_global = time.time()
        action_num = len(self.time)
        
        current_step_idx = 0
        
        while current_step_idx < action_num:
            target_end_time = self.time[current_step_idx] 
            action_type = self.action[current_step_idx]
            action_val = self.value[current_step_idx]

            print(f"[Action: {current_step_idx}]: {action_type} with val {action_val} until t={target_end_time}")

            # Keep action until timeout
            while (time.time() - start_time_global) < target_end_time:
                
                # 1. Send cmd cyclic
                shift, rotate = self._map_action_to_wrench(action_type, action_val)
                self.pub_node.send_cmd(shift, rotate)

                # 2. Get obs
                result_obs = self.get_obs()
                if result_obs:
                  data_idx += 1
                  
                # 3. Wait for nect iter
                time.sleep(self.dt)

            # Next step
            current_step_idx += 1
        
        # Finish execution
        print(f"[Sequence {self.seq_id}] Sequence {self.seq} finished. Stopping robot.")
        self.pub_node.send_cmd([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        print(f"[Sequence {self.seq_id}] Time stamps amount: {data_idx}.")
  
    def get_obs(self):
        if self.sub_node.POSITION is None:
            return None
        else 
            return True
        # --- Tu masz dostęp do danych z Listenera ---
        # Listener actualise data in seperate thread        
        # pos = self.sub_node.POSITION
        # vel = self.sub_node.VELOCITY
        # img = self.sub_node.FLS
        # ts = self.sub_node.TIME_STAMP

        # Dodać flagę zeby zapisywało tylko jak przyjdą nowe pomiary 




#   def main(args=None):
#     rclpy.init(args=args)

#     # 1. Tworzenie Node'ów
#     pub_node = StonefishPublisher()
#     sub_node = StonefishSubscriber()

#     # 2. Executor do obsługi ROSa w tle
#     # MultiThreadedExecutor jest bezpieczniejszy przy wielu callbackach
#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(pub_node)
#     executor.add_node(sub_node)

#     # 3. Uruchomienie ROSa w osobnym wątku
#     # Dzięki daemon=True wątek zamknie się sam, gdy zamknie się program główny
#     ros_thread = Thread(target=executor.spin, daemon=True)
#     ros_thread.start()

#     # 4. Inicjalizacja Twojego Master Controllera
#     controller = MasterController(pub_node, sub_node)
    
#     # Przykładowy setup (symulacja danych z setupu deterministycznego/losowego)
#     # Ręcznie ustawiam dane, żeby pokazać jak to zadziała z pętlą:
#     # Akcja 1: do 3 sekundy, Forward
#     # Akcja 2: do 6 sekundy, Rotate
#     controller.time = [3.0, 6.0] 
#     controller.action = ['forward', 'rotation']
#     controller.value = [20.0, 5.0]

#     try:
#         # 5. Uruchomienie głównej logiki scenariusza
#         # To zablokuje główny wątek dopóki scenariusz się nie skończy
#         # W TYM SAMYM CZASIE 'ros_thread' w tle odbiera dane i aktualizuje sub_node
#         controller.sequence_exec()

#     except KeyboardInterrupt:
#         print("Interrupted by user.")
    
#     finally:
#         # Sprzątanie
#         print("Cleaning up...")
#         # Zatrzymanie robota przed wyjściem
#         pub_node.send_cmd([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        
#         executor.shutdown()
#         pub_node.destroy_node()
#         sub_node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
# Jak to działa (Wyjaśnienie):
  



