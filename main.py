import matplotlib.pyplot as plt
import numpy as np
import glob
from gait_trans.robot import Quadruped
from gait_trans.simulator import SimulationData, SimulationStreamer, Simulator
from gait_trans.utils import *

#state_dict = {0: [1.5, 0.0, np.deg2rad(50)], 
#              15: [3.0, 0.0, np.deg2rad(50)]}

state_dict = {0: [1.5, 0.0, np.deg2rad(0)]}
dt=0.05
n_steps = 50
results_raw = []

def main():

    sim_data = SimulationData(dt)
    sim_stream = SimulationStreamer(state_dict)
    sim = Simulator(sim_data)
    robot = Quadruped(sim_data)
    sim.add_robot(robot)
    
    for i in range(n_steps):
        #print("Step: {}".format(i))
        vel, omega = sim_stream.next_input()
        robot.set_body_cmd_vel(vel)
        robot.set_omega(omega)
        res = sim.step()
        results_raw.append(res)    
        
    sim_results = SimulationResults(results_raw)
    plot_com_traj(sim_results.x_ref, sim_results.x_mpc)
    plot_contact_forces(sim_results.f_mpc)
    plot_mpc_solve_time(sim_results.iter_times)
    plot_fsm(sim_results.fsm)
    
    for i in range(n_steps):
        print(i)
        # plot_com_traj(results_raw[i]["x_ref"], results_raw[i]["x_mpc"])
        #plot_contact_forces(results_raw[i]["f_mpc"])
        #plot_fsm(results_raw[i]["fsm"])
        plot_footstep_locations(results_raw[i], i)
    
    # glob all pngs from gait_trans/notebook_outputs
    frames = glob.glob("gait_trans/notebook_outputs/*.png")
    
    
    plt.show()

if __name__ == "__main__":
    main()