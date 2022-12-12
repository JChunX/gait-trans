import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

def rotation_z_mat(psi_k):
    """
    Returns the rotation matrix for a rotation about the z axis
    """
    return np.array([[np.cos(psi_k), -np.sin(psi_k), 0],
                     [np.sin(psi_k), np.cos(psi_k), 0],
                     [0, 0, 1]])
    
def skew(x):
    """
    Computes the skew symmetric matrix of a vector
    """
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])
    
def plot_fsm(fsm):
    N = fsm.shape[0]
    plt.figure(figsize=(N/4, 10))
    plt.imshow(fsm.T)
    # label y axis. from bottom to top it should be [FR, FL, BR, BL]
    plt.yticks(np.arange(4), ["BL", "BR", "FL", "FR"])
    plt.xlabel("time step")
    plt.ylabel("leg")
    plt.title("Finite State Machine")
    
def plot_contact_forces(force):
    
    N = force.shape[0]
    # keep every 3rd column
    #force = force[:, 2::3]
    #force = force.T.astype(np.uint8)
    plt.figure(figsize=(N/4, 10))
    plt.imshow(force.T)
    # add colorbar but make it shorter
    plt.colorbar(shrink=0.2)
    plt.xlabel("time step")
    plt.ylabel("leg")
    plt.title("Contact forces")
    
def plot_mpc_solve_time(solve_times):
    plt.figure(figsize=(20,5))
    plt.plot(solve_times)
    plt.xlabel("time step")
    plt.ylabel("solve time (s))")
    plt.title("MPC solve time")

def plot_com_traj(x_ref, x_mpc, new_fig=True, label_postfix="", dt=1):
    """
    Plots the center of mass trajectory for the reference and the MPC solution
    
    input:
    
    x_ref - (N, 12) array of reference body states
    
    x_mpc - (N, 12) array of MPC body states
    
    
    x is organized as: [phi, theta, psi, x, y, z, phi_dot, theta_dot, psi_dot, x_dot, y_dot, z_dot]
    
    """
    if new_fig:
        plt.figure(figsize=(20,15))
        
    N = x_ref.shape[0]
    t = np.arange(N) * dt
    plt.subplot(4,3,1)
    plt.plot(t, x_ref[:,0], label="x_ref_phi {}".format(label_postfix))
    plt.plot(t, x_mpc[:,0], label="x_mpc_phi {}".format(label_postfix))
    # set y scale to be between -pi and pi
    plt.ylim(-np.pi, np.pi)
    # but set tiks to degrees
    plt.yticks(np.arange(-np.pi, np.pi, np.pi/2), np.arange(-180, 180, 90))
    plt.xlabel("time (s)")
    plt.ylabel("angle (deg)")
    plt.legend()
    plt.subplot(4,3,2)
    plt.plot(t, x_ref[:,1], label="x_ref_theta".format(label_postfix))
    plt.plot(t, x_mpc[:,1], label="x_mpc_theta".format(label_postfix))
    plt.ylim(-np.pi, np.pi)
    plt.yticks(np.arange(-np.pi, np.pi, np.pi/2), np.arange(-180, 180, 90))
    plt.xlabel("time (s)")
    plt.ylabel("angle (deg)")
    plt.legend()
    plt.subplot(4,3,3)
    plt.plot(t, x_ref[:,2], label="x_ref_psi".format(label_postfix))
    plt.plot(t, x_mpc[:,2], label="x_mpc_psi".format(label_postfix))
    plt.ylim(np.mean(x_ref[:,2]) - np.pi, np.mean(x_ref[:,2]) + np.pi)
    plt.yticks(np.arange(-np.pi, np.pi, np.pi/2), np.arange(-180, 180, 90))
    plt.xlabel("time (s)")
    plt.ylabel("angle (deg)")
    plt.legend()
    plt.subplot(4,3,4)
    plt.plot(t, x_mpc[:,3] - x_ref[:,3], label="x_error {}".format(label_postfix))
    plt.ylim(-0.5, 0.5)
    plt.legend()
    plt.xlabel("time (s)")
    plt.ylabel("error (m)")
    plt.subplot(4,3,5)
    plt.plot(t, x_ref[:,4] - x_mpc[:,4], label="y_error {}".format(label_postfix))
    plt.ylim(-0.5, 0.5)
    plt.xlabel("time (s)")
    plt.ylabel("error (m)")
    plt.legend()
    plt.subplot(4,3,6)
    plt.plot(t, x_ref[:,5] - x_mpc[:,5], label="z_error {}".format(label_postfix))
    plt.ylim(-0.2, 0.2)
    plt.xlabel("time (s)")
    plt.ylabel("error (m)")
    plt.legend()
    plt.subplot(4,3,7)
    plt.plot(t, x_ref[:,6], label="x_ref_phi_dot {}".format(label_postfix))
    plt.plot(t, x_mpc[:,6], label="x_mpc_phi_dot {}".format(label_postfix))
    plt.ylim(-np.pi, np.pi)
    plt.yticks(np.arange(-np.pi, np.pi, np.pi/2), np.arange(-180, 180, 90))
    plt.xlabel("time (s)")
    plt.ylabel("angular velocity (deg/s)")
    plt.legend()
    plt.subplot(4,3,8)
    plt.plot(t, x_ref[:,7], label="x_ref_theta_dot {}".format(label_postfix))
    plt.plot(t, x_mpc[:,7], label="x_mpc_theta_dot {}".format(label_postfix))
    plt.ylim(-np.pi, np.pi)
    plt.yticks(np.arange(-np.pi, np.pi, np.pi/2), np.arange(-180, 180, 90))
    plt.xlabel("time (s)")
    plt.ylabel("angular velocity (deg/s)")
    plt.legend()
    plt.subplot(4,3,9)
    plt.plot(t, x_ref[:,8], label="x_ref_psi_dot {}".format(label_postfix))
    plt.plot(t, x_mpc[:,8], label="x_mpc_psi_dot {}".format(label_postfix))
    plt.ylim(-np.pi, np.pi)
    plt.yticks(np.arange(-np.pi, np.pi, np.pi/2), np.arange(-180, 180, 90))
    plt.xlabel("time (s)")
    plt.ylabel("angular velocity (deg/s)")
    plt.legend()
    plt.subplot(4,3,10)
    plt.plot(t, x_ref[:,9], label="x_ref_x_dot {}".format(label_postfix))
    plt.plot(t, x_mpc[:,9], label="x_mpc_x_dot {}".format(label_postfix))
    plt.ylim(np.mean(x_ref[:,9]) - 2.0, np.mean(x_ref[:,9]) + 2.0)
    plt.xlabel("time (s)")
    plt.ylabel("velocity (m/s)")
    plt.legend()
    plt.subplot(4,3,11)
    plt.plot(t, x_ref[:,10], label="x_ref_y_dot {}".format(label_postfix))
    plt.plot(t, x_mpc[:,10], label="x_mpc_y_dot {}".format(label_postfix))
    plt.ylim(np.mean(x_ref[:,10]) - 2.0, np.mean(x_ref[:,10]) + 2.0)
    plt.xlabel("time (s)")
    plt.ylabel("velocity (m/s)")
    plt.legend()
    plt.subplot(4,3,12)
    plt.plot(t, x_ref[:,11], label="x_ref_z_dot {}".format(label_postfix))
    plt.plot(t, x_mpc[:,11], label="x_mpc_z_dot {}".format(label_postfix))
    plt.ylim(np.mean(x_ref[:,11]) - 2.0, np.mean(x_ref[:,11]) + 2.0)
    plt.xlabel("time (s)")
    plt.ylabel("velocity (m/s)")
    plt.legend()
    
def plot_footstep_locations(results, step_num):
    """Plot the footstep locations, for each iteration of the MPC"""
    r_ref = results["r_ref"]
    x_ref = results["x_ref"]
    fsm = results["fsm"]
    r_world =  r_ref + x_ref[:, np.newaxis, 3:6]
    fig = plt.figure()
    plt.plot(x_ref[:,3], x_ref[:,4], label="x_ref")
    plt.axis("equal")
    
    colors = ['r', 'g', 'b', 'k']
    i=0
    contacts = fsm[0]
    for j in range(4):
        if contacts[j]:
            plt.plot(r_world[i, j, 0], r_world[i, j, 1], 'o', color=colors[j])
            
    # draw the convex hull of foot positions in contact, if number of contacts > 2
    if np.sum(contacts) > 2:
        hull = ConvexHull(r_world[i, contacts == 1, :2])
        for simplex in hull.simplices:
            plt.plot(r_world[i, contacts == 1, 0][simplex], r_world[i, contacts == 1, 1][simplex], 'k-')
            
    # save figure to image
    fig.savefig('gait_trans/notebook_outputs/frame_{}.png'.format(step_num))    
    plt.close(fig)   
    
    
class SimulationResults:
    
    def __init__(self, results):
        self.results = results
        self.parse_results()
        
    def parse_results(self):
        N = len(self.results)
        self.iter_times = []
        self.f_mpc = np.zeros((N, 12))
        self.x_mpc = np.zeros((N, 12))
        self.x_ref = np.zeros((N, 12))
        self.fsm = np.zeros((N, 4))
        self.r_ref = np.zeros((N, 4, 3))
        self.mpc_success = []
        
        for i, result_dict in enumerate(self.results):
            self.iter_times.append(result_dict["iter_time"])
            self.f_mpc[i,:] = result_dict["f_mpc"][0]
            self.x_mpc[i,:] = result_dict["x_mpc"][0]
            self.x_ref[i,:] = result_dict["x_ref"][0]
            self.fsm[i,:] = result_dict["fsm"][0]
            self.r_ref[i,:,:] = result_dict["r_ref"][0]
            self.mpc_success.append(result_dict["success"])
            