import time


class SimulationData:
    
    def __init__(self, dt):
        self.time = 0.0
        self.dt = dt
        self.grav = 9.81
        
    def step(self):
        self.time += self.dt
        
        
class SimulationStreamer:
    
    """
    SimulationStreamer streams data from offline data file as inputs to the simulation
    
    attributes:
    
    state_dict: dictionary of body velocities
    {
        <step_num>: [vx, vy, omega]
    }
    a key value pair where k=0 must be included in the dictionary
    
    """
    
    def __init__(self, state_dict):
        self.state_dict = state_dict
        self.step_num = 0
        self.prev_input = self.state_dict[self.step_num]
        
    def next_input(self):
        if self.step_num in self.state_dict:
            self.prev_input = self.state_dict[self.step_num]
            
        vel = self.prev_input[0:2]
        omega = self.prev_input[2]
        self.step_num += 1
        
        return vel, omega
        
    
class Simulator:
    
    def __init__(self, sim_data):
        self.robot = None
        self.sim_data = sim_data
        
    def step(self):
        res = self.robot.step()
        self.sim_data.step()
        
        success = res["success"]
        if not success:
            raise Exception("Robot MPC failed")
        
        return res
        
    def add_robot(self, robot):
        self.robot = robot