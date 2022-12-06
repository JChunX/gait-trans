import matplotlib.pyplot as plt
import numpy as np

from gait_trans.robot import Quadruped
from gait_trans.simulator import SimulationData, SimulationStreamer, Simulator
from gait_trans.utils import plot_com_traj, plot_contact_forces

state_dict = {0: [1.5, 0.0, 0.0], 
              15: [3.0, 0.0, 0.0]}

n_steps = 20
results = []

def main():

    sim_data = SimulationData(dt=0.05)
    sim_stream = SimulationStreamer(state_dict)
    sim = Simulator(sim_data)
    robot = Quadruped(sim_data)
    sim.add_robot(robot)
    
    for i in range(n_steps):
        print("Step: {}".format(i))
        vel, omega = sim_stream.next_input()
        robot.set_body_cmd_vel(vel)
        robot.set_omega(omega)
        res = sim.step()
        results.append(res)    

if __name__ == "__main__":
    main()