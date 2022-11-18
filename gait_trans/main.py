import numpy as np

from gait_trans.mpc import QuadrupedMPC

def main():
    leg_shoulder_pos = [np.array([0.5,0.5,0]),
    np.array([0.5,-0.5,0]),
    np.array([-0.5,0.5,0]),
    np.array([-0.5,-0.5,0])]

    body_cmd_vel = np.array([0.5, 0])
    omega = np.pi/200
    dt = 0.1
    N = 50
    x_ref = np.zeros((N, 12))
    x_ref_tmp = gen_body_trajectory(body_cmd_vel, omega, dt, N)
    x_ref[:,:2] = np.zeros((N,2))
    x_ref[:,2] = x_ref_tmp[:,6]
    x_ref[:,3:6] = x_ref_tmp[:,:3]
    x_ref[:,6:8] = np.zeros((N,2))
    x_ref[:,8] = x_ref_tmp[:,7]
    x_ref[:,9:12] = x_ref_tmp[:,3:6]

    contact_scheduler = ContactScheduler(2, dt)
    fsm = contact_scheduler.make_trot_contact_sequence(N, 0)
    r_ref = gen_foot_positions(x_ref_tmp, fsm, leg_shoulder_pos)
    # convert r_ref from world frame to body frame
    r_ref = r_ref - x_ref[:, np.newaxis, 3:6]

    mpc = QuadrupedMPC(N, dt)
    mpc.compute_mpc(x_ref, r_ref[:N-1], fsm[:N-1])

if __name__ == "__main__":
    main()