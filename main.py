import glob

import matplotlib.pyplot as plt
import numpy as np
import cv2

from gait_trans.robot import Quadruped
from gait_trans.simulator import SimulationData, SimulationStreamer, Simulator
from gait_trans.utils import *
from gait_trans.videowriter import make_video

dt=0.04
n_steps = int(1/dt * 10)

state_dict = {0: [1.5, 0.0, np.deg2rad(0)], 
              int(1/dt * 5): [2.5, 0.0, np.deg2rad(0)],
              int(1/dt * 7): [3.2, 0.0, np.deg2rad(0)]}

#state_dict = {0: [3, 0.0, np.deg2rad(0)]}

results_raw = []

def main():
    skips = 6
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
    
    
    
    for i in range(int(n_steps / skips)):
        #plot_com_traj(results_raw[i]["x_ref"], results_raw[i]["x_mpc"])
        #plot_contact_forces(results_raw[i]["f_mpc"])
        #plot_fsm(results_raw[i]["fsm"])
        plot_footstep_locations(results_raw[i], i * skips)

    
    # glob all pngs from gait_trans/notebook_outputs
    frame_dirs = glob.glob("gait_trans/notebook_outputs/*.png")
    frames = []
    for frame_dir in frame_dirs:
        frames.append(cv2.imread(frame_dir))

    make_video("out.mp4", frames, fps=int((1/dt) / skips))
    
    plt.show()

if __name__ == "__main__":
    main()