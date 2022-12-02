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
    TODO
    """
    
    def __init__(self):
        pass
    
class Simulator:
    
    def __init__(self, sim_data):
        self.robot = None
        self.sim_data = sim_data
        
    def step(self):
        res = self.robot.step()
        self.sim_data.step()
        
        return res
        
    def add_robot(self, robot):
        self.robot = robot