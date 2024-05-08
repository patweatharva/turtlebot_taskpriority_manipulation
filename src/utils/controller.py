from .common import *
class Controller():
    def __init__(self, tasks, robot, weight_matrix):
        self.tasks          = tasks
        self.robot          = robot
        self.weight_matrix  = weight_matrix


    def compute(self):
        ### Recursive Task-Priority algorithm (w/set-based tasks)
        # The algorithm works in the same way as in Lab4. 
        # The only difference is that it checks if a task is active.
        # Initialize null-space projector
        P   = np.eye(self.robot.dof, self.robot.dof)
        # Initialize output vector (joint velocity)
        dq  = np.zeros((self.robot.dof, 1))
        for i in range(len(self.tasks)):      
            # Update task state
            self.tasks[i].update(self.robot)

            # Compute augmented Jacobian
            a = self.tasks[i].J
            Jbar    = self.tasks[i].J @ P 
            # Compute task velocity
            # Accumulate velocity
            a = self.tasks[i].K
            b = self.tasks[i].err
            c = weighted_DLS(Jbar, 0.1, self.weight_matrix)
            d = self.tasks[i].J
            dq      = dq + weighted_DLS(Jbar, 0.1, self.weight_matrix) @ (self.tasks[i].K @ self.tasks[i].err - self.tasks[i].J @ dq) 
            # Update null-space projector
            P       = P - DLS(Jbar, 0.001) @ Jbar  

        return dq
