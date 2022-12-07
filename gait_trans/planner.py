import numpy as np

from gait_trans.utils import rotation_z_mat

g = 9.81

class GaitPlanner:

    def get_foot_positions(p_k, psi_k, l, v=0, v_cmd=0, omega_cmd=0, h=0.5, t_stance=2, k=0.03):
        """
        Foot step planner
        returns the foot positions for a given step k
        
        inputs:
        
        p_k - body position at step k, wrt world
        
        psi_k - body orientation at step k
        
        l - i_th leg shoulder location wrt body frame, (4,) array
            - convention: [FL, FR, BL, BR]
            
        v - body velocity at step k
        
        v_cmd - commanded body velocity
        
        omega_cmd - commanded body angular velocity
        
        t_stance - stance time
        
        k - velocity gain
        
        h - step height ????
        """
        positions = []
        p_symmetry = 0#t_stance * v / 2 + k * (v - v_cmd) 
        p_centrifugal = 0#0.5 * np.sqrt(h/g) * np.cross(v, omega_cmd)
        for i in range(4):
            # p_shoulder is the i-th shoulder location wrt global frame
            p_shoulder = p_k + rotation_z_mat(psi_k) @ l[i]
            p_shoulder[2] = 0
            positions.append(p_shoulder + p_symmetry + p_centrifugal)
            
        return positions

    def gen_body_trajectory(v, omega, dt, x_0, N):
        """
        Generates a body trajectory for a given velocity and angular velocity
        
        input:
        
        v - x, y velocity in body frame
        
        omega - body angular velocity (yaw), scalar
        
        dt - time step
        
        x_0 - initial state, (12,) array
                
        N - number of steps to generate
        
        returns:
        Nx12 trajectory vector: (phi=0, theta=0, psi, 
                                 x, y, z, 
                                 phi_dot, theta_dot, psi_dot=omega, 
                                 x_dot, y_dot, z_dot=0)
        """
        # TODO: directly fill Nx12 array
        body_trajectory = np.zeros((N, 8))
        body_trajectory[0, :3] = x_0[3:6]
        body_trajectory[0, 3:6] = x_0[9:12]
        body_trajectory[0, 6] = x_0[2]
        body_trajectory[0, 7] = omega
        rot_mat = rotation_z_mat(omega * dt)
        v = np.append(v, 0)
        for i in range(N):
            if i > 0:
                # increment body position
                body_trajectory[i, :3] = rot_mat @ (body_trajectory[i-1, :3] + v * dt)
                # compute body velocity via finite difference
                body_trajectory[i, 3:6] = (body_trajectory[i, :3] - body_trajectory[i-1, :3]) / dt
            else:
                body_trajectory[i, 3:6] = v
            body_trajectory[i, 6] = body_trajectory[i-1, 6] + omega * dt
            body_trajectory[i, 7] = omega
        
        body_trajectory[:, 2] = x_0[5]
        
        # reformat to (N, 12) array for MPC controller
        body_trajectory_mpc = np.zeros((N, 12))
        body_trajectory_mpc[:,:2] = np.zeros((N,2))
        body_trajectory_mpc[:,2] = body_trajectory[:,6]
        body_trajectory_mpc[:,3:6] = body_trajectory[:,:3]
        body_trajectory_mpc[:,6:8] = np.zeros((N,2))
        body_trajectory_mpc[:,8] = body_trajectory[:,7]
        body_trajectory_mpc[:,9:12] = body_trajectory[:,3:6]
        
        return body_trajectory_mpc

    def gen_foot_positions(body_traj, fsm, leg_shoulder_pos, N, prev_contacts=np.zeros(4)):
        """
        Generate foot positions for a given body trajectory
        
        input:
        
        body_traj - Nx12 body trajectory
        
        fsm - Nx4 contact sequence boolean array
        
        leg_shoulder_pos - i_th leg shoulder location wrt body frame, (4,) array
        
        returns:
        
        foot_positions - Nx4x3 foot position array
        """
        foot_positions = np.zeros((N, 4, 3))

        for i in range(N-1):
            body_state = body_traj[i]
            contacts = fsm[i]
            # get the indices where leg went from not in contact to in contact
            new_in_contact = np.where(np.logical_and(prev_contacts == 0, contacts == 1))[0]
            # get the indices where leg went from in contact to not in contact
            new_out_contact = np.where(np.logical_and(prev_contacts == 1, contacts == 0))[0]
            
            new_foot_positions = GaitPlanner.get_foot_positions(
                body_state[3:6], 
                body_state[2], 
                leg_shoulder_pos)
            
            # foot_positions[i] is the prev foot position, unless it just went out of contact
            foot_positions[i] = foot_positions[i-1]
            for j in new_in_contact:
                foot_positions[i, j] = new_foot_positions[j]
            
            prev_contacts = contacts
            
        return foot_positions


class ContactScheduler:
    """
    Contact Scheduler assigns contact sequence for next N steps
    """
    
    trot_params = {
        "gait_phase_offsets": [0.5, 0.0, 0.0, 0.5],
        "stance_fraction": 0.6
    }
    
    gallop_params = {
        "gait_phase_offsets": [0.2, 0.6, 0.4, 0.0],
        "stance_fraction": 0.25
    }
    
    param_dict = {"trot": trot_params,
                  "gallop": gallop_params}
    
    def make_fsm(period, t, dt, N, gait_params, phase_offset=0):
        """
        Makes a trot contact sequence for the next N steps, starting at time t
        
        inputs:
        
        N - number of steps
        period - gait period
        t - current time
        dt - time step
        
        phase_offset - phase offset for the first step, in [0, 1]
        
        output:
        
        fsm - (N, 4) array, where each row is a contact sequence for a step
                            1 means in contact, 0 means not in contact
        """
        # trot_phase_offsets are the percentage offset where contact should start
        fsm = np.zeros((N, 4))
        gait_phase_offsets = gait_params["gait_phase_offsets"]
        # t_stance is how long the robot spends in stance
        stance_fraction = gait_params['stance_fraction']

        for i in range(N):
            time = t + i * dt
            for j in range(4):
                phase = ContactScheduler.time_to_phase(time, gait_phase_offsets[j] + phase_offset, period)
                if phase < stance_fraction:
                    fsm[i, j] = 1
            
        return fsm
    
    def time_to_phase(time, offset, period):
        """
        Converts time to phase
        """
        return ((time % period) / period + offset) % 1
