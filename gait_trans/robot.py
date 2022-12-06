import time

import numpy as np

from gait_trans.mpc import QuadrupedMPC
from gait_trans.planner import ContactScheduler, GaitPlanner


class Quadruped:
    """
    Robot class for quadruped
    
    The robot knows where it is at all times. It knows this because it knows where it isn't. By subtracting where it is from where it isn't, or where it isn't from where it is (whichever is greater), it obtains a difference, or deviation. The guidance subsystem uses deviations to generate corrective commands to drive the robot from a position where it is to a position where it isn't, and arriving at a position where it wasn't, it now is. Consequently, the position where it is, is now the position that it wasn't, and it follows that the position that it was, is now the position that it isn't.
    
    In the event that the position that it is in is not the position that it wasn't, the system has acquired a variation, the variation being the difference between where the robot is, and where it wasn't. If variation is considered to be a significant factor, it too may be corrected by the GEA. However, the robot must also know where it was.
    
    The robot guidance computer scenario works as follows. Because a variation has modified some of the information the robot has obtained, it is not sure just where it is. However, it is sure where it isn't, within reason, and it knows where it was. It now subtracts where it should be from where it wasn't, or vice-versa, and by differentiating this from the algebraic sum of where it shouldn't be, and where it was, it is able to obtain the deviation and its variation, which is called error.
    """
    leg_shoulder_pos = [np.array([0.3,0.1,0]),
                        np.array([0.3,-0.1,0]),
                        np.array([-0.3,0.1,0]),
                        np.array([-0.3,-0.1,0])]
    height = 0.34
    
    speed_to_gait_idx = [0, 1, 2, 3]
    gait_idx_to_name = ["trot", "trot", "gallop", "gallop"]
    gait_to_period = {'trot': 0.6, 'bound': 0.6, 'pace': 0.6, 'gallop': 0.6}
    
    
    def __init__(self, sim_data):
        self.body_cmd_vel = np.array([0,0])
        self.omega = 0
        self.x = np.zeros(12)
        self.x[5] = self.height
        self.gait_period = 0.6
        self.sim_data = sim_data
        
        self.mpc_Q =  1000 * np.diag(
            [1,   1,   1,
             1,   1,   1,
             20,  5,   5,
             100, 100, 100])
    
        self.mpc_R = 1 * np.identity(12)
        
        self.planning_horizon = 100
        self.prev_contacts = np.zeros(4)
        
        self.cur_gait_type = None
        self.prev_gait_type = None
        self.mpc_fail_count = 0
        self.mpc_fail_threshold = 5
        self.f_mpc_prev = None
        self.x_mpc_prev = None
        
    def set_body_cmd_vel(self, body_cmd_vel):
        self.body_cmd_vel = body_cmd_vel
        
    def set_omega(self, omega):
        self.omega = omega
        
    def set_gait_freq(self, gait_freq):
        self.gait_freq = gait_freq
        
    def step(self):
        """
        In each step, the robot should update its current plan and run its controllers to determine the next body state
        """
        print("Generating new plan...")
        x_ref = GaitPlanner.gen_body_trajectory(self.body_cmd_vel, self.omega, self.sim_data.dt, self.x, self.planning_horizon)
        
        print("Finding contact forces...")
        mpc_time, f_mpc, x_mpc, fsm, r_ref, cost, success = self.find_contact_forces(x_ref)
        
        if not success:
            if self.mpc_fail_count >= self.mpc_fail_threshold:
                print("MPC failed too many times. Stopping.")
                return {}
            self.x_mpc = self.x_mpc_prev[self.mpc_fail_count, :]
            self.f_mpc = self.f_mpc_prev[self.mpc_fail_count, :]
            self.mpc_fail_count += 1
            success = True
        else:
            self.mpc_fail_count = 0
            self.f_mpc_prev = f_mpc
            self.x_mpc_prev = x_mpc
            
        self.x = x_mpc[1]
        
        step_dict = {
            "iter_time": mpc_time,
            "f_mpc": f_mpc,
            "x_mpc": x_mpc,
            "x_ref": x_ref,
            "fsm": fsm,
            "r_ref": r_ref,
            "success": success
        }
        
        # save previous contacts the ensure correct foot placements in next iteration
        self.prev_contacts = fsm[0]
        self.prev_gait_type = self.cur_gait_type
        
        return step_dict
    
    def find_contact_forces(self, x_ref):
        """
        Given a reference trajectory, 
        find the forces that will be applied to the robot's feet to achieve that trajectory
        """
        t0 = time.time()
        self.select_gait()
        gait_params = ContactScheduler.param_dict[self.cur_gait_type]
        success = False
        # gait is transitioning.
        # we need to vary the fsm phase offsets and compute mpc costs
        if self.cur_gait_type != self.prev_gait_type:
            best_cost = np.inf
            best_fsm = None
            best_r_ref = None
            best_f_mpc = None
            best_x_mpc = None
            for _, offset in enumerate(np.unique(gait_params["gait_phase_offsets"])):
                fsm = ContactScheduler.make_fsm(
                    self.gait_period, self.sim_data.time, 
                    self.sim_data.dt, self.planning_horizon, 
                    gait_params, phase_offset = offset)
                r_ref = GaitPlanner.gen_foot_positions(
                    x_ref, fsm, 
                    self.leg_shoulder_pos, 
                    self.planning_horizon, 
                    self.prev_contacts)
                r_ref = r_ref - x_ref[:, np.newaxis, 3:6]
                
                mpc = QuadrupedMPC(self.planning_horizon, 
                                   self.sim_data.dt, 
                                   self.mpc_Q, self.mpc_R)
                f_mpc, x_mpc, cost, success = mpc.compute_mpc(x_ref, 
                                        r_ref[:self.planning_horizon-1], 
                                        fsm[:self.planning_horizon-1])
                print("MPC cost for offset {0}: {1}".format(offset, cost))
                if success and cost < best_cost:
                    best_cost = cost
                    best_fsm = fsm
                    best_r_ref = r_ref
                    best_f_mpc = f_mpc
                    best_x_mpc = x_mpc
                    
            f_mpc = best_f_mpc
            x_mpc = best_x_mpc
            fsm = best_fsm
            r_ref = best_r_ref
            cost = best_cost
                    
        else:
            fsm = ContactScheduler.make_fsm(
                    self.gait_period, self.sim_data.time, 
                    self.sim_data.dt, self.planning_horizon, 
                    gait_params)
            r_ref = GaitPlanner.gen_foot_positions(
                x_ref, fsm, 
                self.leg_shoulder_pos, 
                self.planning_horizon, self.prev_contacts)
            r_ref = r_ref - x_ref[:, np.newaxis, 3:6]
            
            mpc = QuadrupedMPC(self.planning_horizon, 
                               self.sim_data.dt, 
                               self.mpc_Q, self.mpc_R)

            f_mpc, x_mpc, cost, success = mpc.compute_mpc(x_ref, 
                                        r_ref[:self.planning_horizon-1], 
                                        fsm[:self.planning_horizon-1])
        mpc_time = time.time() - t0
            
        return mpc_time, f_mpc, x_mpc, fsm, r_ref, cost, success
    
    def select_gait(self):
        # use speed_to_gait_idx to get gait type, based on current speed
        speed = np.linalg.norm(self.body_cmd_vel)
        gait_idx = 0
        for i in range(len(self.speed_to_gait_idx)):
            if speed > self.speed_to_gait_idx[i]:
                gait_idx = i
            else:
                break
            
        self.cur_gait_type = self.gait_idx_to_name[gait_idx]
        print("Selected gait: {0}".format(self.cur_gait_type))