import os

import numpy as np
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.solvers import (CommonSolverOption, 
                             MathematicalProgram,
                             SolverOptions)
from pydrake.solvers.osqp import OsqpSolver
from pydrake.systems.framework import Context, LeafSystem

from gait_trans.utils import rotation_z_mat, skew

# MPC control class for quadruped

class QuadrupedMPC(LeafSystem):

    def __init__(self, N, dt, Q, R):
        """
        Model-Predictive Control for approximated quadruped
        
        Goal:
        Find reaction forces given pre-defined contacts and body trajectory
        """
        
        LeafSystem.__init__(self)
        """
        self.plant = MultibodyPlant(0.0)
        self.parser = Parser(self.plant)
        self.parser.AddModelFromFile("../models/xml/mini_cheetah.xml")
        self.plant.WeldFrames(self.plant.world_frame(), 
                              self.plant.GetFrameByName("body"))
        self.plant.Finalize()
        self.plant_context = self.plant.CreateDefaultContext()
        """
        self.N = N
        self.dt = dt

        # inertia tensor of the body
        # approximated as a rectangular prism with dimensions w, l, h
        l = 0.7
        w = 0.3
        h = 0.2
        self.m = 8.852
        self.I_B = np.diag([11253, 36203, 42673]) * 1e-6
        self.g = np.zeros((12,))
        self.g[9:] = np.array([0, 0, -9.81 * self.dt])
        
        self.Q = Q
        self.R = R

    def discrete_time_dynamics(self, x_ref_k, r_k):
        """
        Returns A_k, B_k simplified discrete time dynamics for quadruped
        
        x_ref_k is the reference state of the body:
        [Theta, p, omega, p_dot] 
        
        Theta [phi, theta, psi] - roll, pitch, yaw of the body in world frame
        p [x, y, z] - the position of the body in the world frame
        omega - angular velocity of the body in local frame
        p_dot - velocity of the body in world frame
        shape is (12,1)
        
        r_k - positions of contact points relative to the body
        shape is (num_contacts, 3)
        
        """
        I3 = np.eye(3)
        Z3 = np.zeros((3, 3))
        
        psi_k = x_ref_k[2]
        Rz_k = rotation_z_mat(psi_k)
        A_k = np.zeros((12, 12))
        A_k[:3, :] = np.hstack((I3, Z3, Rz_k * self.dt, Z3))
        A_k[3:6, :] = np.hstack((Z3, I3, Z3, I3 * self.dt))
        A_k[6:9, :] = np.hstack((Z3, Z3, I3, Z3))
        A_k[9:, :] = np.hstack((Z3, Z3, Z3, I3))
        
        I_G = Rz_k @ self.I_B @ Rz_k.T
        B_k = np.zeros((12, 3 * r_k.shape[0]))
        
        for i in range(r_k.shape[0]):
            B_k[6:9, 3 * i:3 * (i + 1)] = np.linalg.inv(I_G) @ skew(r_k[i, :]) * self.dt
            B_k[9:, 3 * i:3 * (i + 1)] = I3 * self.dt / self.m
        
        return A_k, B_k
        
    def add_initial_constraints(self, x, x_ref):
        """
        Add initial constraints to the program
        
        inputs:
        
        x - (N, 12) array of body states
        
        x_ref - (N, 12) array of reference body states
        """
        init_constraint = self.prog.AddLinearEqualityConstraint(x[0, :], x_ref[0, :])
        init_constraint.evaluator().set_description("initial constraint")
        
    def get_fk(self, indices, f):
        """ 
        Get reaction force decision vars that have contacts
        
        inputs: 
        
        indices - list of indices for contact forces
        
        f - (12,) array of reaction forces for current time step
        
        example:
        
        f = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])
        indices = [0, 2]
        f_k = np.array([1, 2, 3, 7, 8, 9])
        
        output:
        
        f_k - (3*num_contacts,1) array of reaction forces for current time step
        """
        f_k = np.zeros((len(indices)*3, 1), dtype="object")
        for i, idx in enumerate(indices):
            f_k[3*i:3*(i+1), :] = f[3*idx:3*(idx+1)].reshape(3,1)
            
        return f_k
    
    def add_dynamic_constraints(self, x, f, x_ref, r_ref, fsm):
        """
        Add dynamic constraints to the program
        
        inputs:
        
        x - (N, 12) array of body states
        
        f - (N-1, 3*4) array of contact forces
        
        x_ref - (N, 12) array of reference body states
        
        r_ref - (N-1, 4, 3) array of reference foot positions
        
        fsm - (N-1, 4) array of contact sequences
        
        for each time step in the horizon,
            we need to identify which legs are in contact
            then fix dynamics
        
        """
        
        for i in range(0, self.N - 1):
            in_contact = np.where(fsm[i, :] == 1)[0]
            r_k = r_ref[i, in_contact, :]
            f_k = self.get_fk(in_contact, f[i, :]).flatten()
            x_k = x[i, :]
            
            x_kp1 = x[i + 1, :]

            A_k, B_k = self.discrete_time_dynamics(x_ref[i, :], r_k)
            dyn = A_k @ x_k + B_k @ f_k + self.g - x_kp1

            dyn_constraint = self.prog.AddLinearEqualityConstraint(dyn, np.zeros((12,)))
            dyn_constraint.evaluator().set_description("dynamics constraint")
    
    def add_contact_constraints(self, f, fsm, mu=0.7):
        """
        Add ground reaction force constraints to the program
        
        inputs:
        
        f - (N-1, 3*4) array of contact forces
        
        fsm - (N-1, 4) array of contact sequences
        
        for each foot in contact,
            abs(fx) <= mu * fz
            abs(fy) <= mu * fz
            fz >= 0
            
        else, fix to zero
        
        """
        
        for i in range(self.N-1):
            
            
            in_contact = np.where(fsm[i, :] == 1)[0]
            not_in_contact = np.where(fsm[i, :] == 0)[0]
            f_k = self.get_fk(in_contact, f[i, :])
            f_k_not = self.get_fk(not_in_contact, f[i, :])

            num_contacts = len(in_contact)
            epsilon = 2e-3
            
            for j in range(num_contacts):
                A = np.array( [[1, 0, -mu], 
                               [-1, 0, -mu], 
                               [0, 1, -mu], 
                               [0, -1, -mu], 
                               [0, 0, -1]])
                f_k_j = f_k[3 * j:3 * (j + 1)]
                ub = np.array([0, 0, 0, 0, 0])
                lb = np.array([-np.inf, -np.inf, -np.inf, -np.inf, -np.inf])
                f_contact = self.prog.AddLinearConstraint(A@f_k_j,lb,ub)
                f_contact.evaluator().set_description("font contact constraint j={}".format(j))
                
    def add_cost(self, x, x_ref, f):
        """
        Add cost to the program
        
        inputs:
        
        x - (N, 12) array of body states
        
        x_ref - (N, 12) array of reference body states
        
        f - (N-1, 3*4) array of contact forces
        """

        for i in range(0, self.N-1):
            x_kp1 = x[i+1, :]
            x_ref_kp1 = x_ref[i+1, :]
            x_dk = x_kp1 - x_ref_kp1
            self.prog.AddQuadraticCost(x_dk.T @ self.Q @ x_dk)
            f_k = f[i, :]
            self.prog.AddQuadraticCost(f_k.T @ self.R @ f_k)
            
    def compute_mpc(self, x_ref, r_ref, fsm, verbose=False):
        """
        Compute the optimal control input
        
        inputs:
        
        x_ref - (N, 12) array of reference body states
        
        r_ref - (N-1, 4, 3) array of reference foot positions
        
        fsm - (N-1, 4) array of contact sequences
        
        returns:
        
        u - (N, 3*4) array of contact forces
        """
        print("Computing MPC...")
        self.prog = MathematicalProgram()
        x = np.zeros((self.N, 12), dtype="object")
        for i in range(self.N):
            x[i] = self.prog.NewContinuousVariables(12, "x_" + str(i))
        f = np.zeros((self.N-1, 3*4), dtype="object")
        for i in range(self.N-1):
            in_contact = np.where(fsm[i, :] == 1)[0] # TODO: eliminate decision variables for feet not in contact
            for idx in in_contact:
                f[i, 3 * idx:3 * (idx + 1)] = self.prog.NewContinuousVariables(3, "f_" + str(i) + "_" + str(idx))
        
        print("Adding initial condition constraint...")
        self.add_initial_constraints(x, x_ref)
        print("Adding dynamic constraints...")
        self.add_dynamic_constraints(x, f, x_ref, r_ref, fsm)
        print("Adding contact constraints...")
        self.add_contact_constraints(f, fsm)
        
        self.add_cost(x, x_ref, f)
        
        print("Solving...")
        solver = OsqpSolver()
        logfile = "logs/debug.txt"
        # if logfile exists, delete it
        if os.path.exists(logfile):
            os.remove(logfile)
        solver_options = SolverOptions()
        solver_options.SetOption(CommonSolverOption.kPrintFileName, logfile)
        result = solver.Solve(self.prog, solver_options=solver_options)
        infeasible_constraints = result.GetInfeasibleConstraints(self.prog)
        f_mpc = result.GetSolution(f)
        x_mpc = result.GetSolution(x)
        f_mpc = np.vectorize(lambda x: x.Evaluate())(f_mpc)

        if verbose:
            for c in infeasible_constraints:
                print(f"infeasiable: {c}")
        
        if result.is_success():
            print("Success!")
            cost = result.get_optimal_cost()
            return (f_mpc, x_mpc, cost, True)
        else:
            print("MPC failed to solve")
            return (None, None, None, False)
            
            