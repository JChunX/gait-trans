import numpy as np
import matplotlib.pyplot as plt


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
    
def plot_contact_forces(force):
    
    N = force.shape[0]
    #force = force.T.astype(np.uint8)
    print(force[:5])
    plt.figure(figsize=(N/2, 5))
    plt.imshow(force.T)

def plot_com_traj(x_ref, x_mpc):
    """
    Plots the center of mass trajectory for the reference and the MPC solution
    
    input:
    
    x_ref - (N, 12) array of reference body states
    
    x_mpc - (N, 12) array of MPC body states
    
    
    x is organized as: [phi, theta, psi, x, y, z, phi_dot, theta_dot, psi_dot, x_dot, y_dot, z_dot]
    
    """
    
    plt.figure(figsize=(20,15))
    plt.subplot(4,3,1)
    plt.plot(x_ref[:,0], label="x_ref_phi")
    plt.plot(x_mpc[:,0], label="x_mpc_phi")
    # set y scale to be between -pi and pi
    plt.ylim(-np.pi, np.pi)
    # but set tiks to degrees
    plt.yticks(np.arange(-np.pi, np.pi, np.pi/2), np.arange(-180, 180, 90))
    plt.legend()
    plt.subplot(4,3,2)
    plt.plot(x_ref[:,1], label="x_ref_theta")
    plt.plot(x_mpc[:,1], label="x_mpc_theta")
    plt.ylim(-np.pi, np.pi)
    plt.yticks(np.arange(-np.pi, np.pi, np.pi/2), np.arange(-180, 180, 90))
    plt.legend()
    plt.subplot(4,3,3)
    plt.plot(x_ref[:,2], label="x_ref_psi")
    plt.plot(x_mpc[:,2], label="x_mpc_psi")
    plt.ylim(np.mean(x_ref[:,2]) - np.pi, np.mean(x_ref[:,2]) + np.pi)
    plt.yticks(np.arange(-np.pi, np.pi, np.pi/2), np.arange(-180, 180, 90))
    plt.legend()
    plt.subplot(4,3,4)
    plt.plot(x_mpc[:,3] - x_ref[:,3], label="x_error")
    plt.ylim(-2.0, 2.0)
    plt.legend()
    plt.subplot(4,3,5)
    plt.plot(x_ref[:,4], label="x_ref_y")
    plt.plot(x_mpc[:,4], label="x_mpc_y")
    plt.ylim(np.mean(x_ref[:,4]) - 2.0, np.mean(x_ref[:,4]) + 2.0)
    plt.legend()
    plt.subplot(4,3,6)
    plt.plot(x_ref[:,5], label="x_ref_z")
    plt.plot(x_mpc[:,5], label="x_mpc_z")
    plt.ylim(np.mean(x_ref[:,5]) - 0.5, np.mean(x_ref[:,5]) + 0.5)
    plt.legend()
    plt.subplot(4,3,7)
    plt.plot(x_ref[:,6], label="x_ref_phi_dot")
    plt.plot(x_mpc[:,6], label="x_mpc_phi_dot")
    plt.ylim(-np.pi, np.pi)
    plt.yticks(np.arange(-np.pi, np.pi, np.pi/2), np.arange(-180, 180, 90))
    plt.legend()
    plt.subplot(4,3,8)
    plt.plot(x_ref[:,7], label="x_ref_theta_dot")
    plt.plot(x_mpc[:,7], label="x_mpc_theta_dot")
    plt.ylim(-np.pi, np.pi)
    plt.yticks(np.arange(-np.pi, np.pi, np.pi/2), np.arange(-180, 180, 90))
    plt.legend()
    plt.subplot(4,3,9)
    plt.plot(x_ref[:,8], label="x_ref_psi_dot")
    plt.plot(x_mpc[:,8], label="x_mpc_psi_dot")
    plt.ylim(-np.pi, np.pi)
    plt.yticks(np.arange(-np.pi, np.pi, np.pi/2), np.arange(-180, 180, 90))
    plt.legend()
    plt.subplot(4,3,10)
    plt.plot(x_ref[:,9], label="x_ref_x_dot")
    plt.plot(x_mpc[:,9], label="x_mpc_x_dot")
    plt.ylim(np.mean(x_ref[:,9]) - 2.0, np.mean(x_ref[:,9]) + 2.0)
    plt.legend()
    plt.subplot(4,3,11)
    plt.plot(x_ref[:,10], label="x_ref_y_dot")
    plt.plot(x_mpc[:,10], label="x_mpc_y_dot")
    plt.ylim(np.mean(x_ref[:,10]) - 2.0, np.mean(x_ref[:,10]) + 2.0)
    plt.legend()
    plt.subplot(4,3,12)
    plt.plot(x_ref[:,11], label="x_ref_z_dot")
    plt.plot(x_mpc[:,11], label="x_mpc_z_dot")
    plt.ylim(np.mean(x_ref[:,11]) - 2.0, np.mean(x_ref[:,11]) + 2.0)
    plt.legend()
