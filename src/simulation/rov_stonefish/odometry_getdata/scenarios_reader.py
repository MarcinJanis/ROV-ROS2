import numpy as np
import math
import os 
import random 


import xml.etree.ElementTree as ET

class MasterController:
  '''
  Reads test scenario from xml or generates random moves and send to control node. 
  '''
  def __init__(self):
    self.determinist = False
    #TODO: Init ros publisher 
    #TODO: some smart mechanizm how to deal with map baoundaries

  def setup(self, determinist = False: bool, scenario_pth = None: str, mv_count = 0: int, boundaries = None: dict):
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
       
  
  def sequence_exec(self):
    action_num = len(self.time)
    for idx in range(action_num):
      t = self.time[idx]
      act = self.time[idx]
      v = self.time[idx]
      
    pass


  def get_obs(self):
    # counter for const wait time
    # get obs
    # save to file
    pass

  



