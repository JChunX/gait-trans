import matplotlib.pyplot as plt
import numpy as np

from gait_trans.robot import Quadruped
from gait_trans.simulator import SimulationData, Simulator
from gait_trans.utils import plot_com_traj, plot_contact_forces


def main():

    sim_data = SimulationData(dt=0.05)
    sim = Simulator(sim_data)
    robot = Quadruped(sim_data)
    sim.add_robot(robot)
    
    robot.set_body_cmd_vel(np.array([1.5, 0.0]))
    robot.set_omega(np.pi/20)
    res = sim.step()
    
    plot_com_traj(res["x_ref"], res["x_mpc"])
    plot_contact_forces(res["f_mpc"])
    
    plt.show()

if __name__ == "__main__":
    main()