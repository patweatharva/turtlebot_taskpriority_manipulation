import numpy as np


class Task:
    """
    Task class represents a generic task for a manipulator.

    Attributes:
        name (str): Title of the task.
        sigma_d (numpy array): Desired sigma (goal).
        FeedForward: Feedforward component for the task.
        K: Gain for the task.
        err_hist (list): History of task errors.
        activation (int): Activation status of the task (1 for active, 0 for inactive).
    """

    def __init__(self, name: str, desired, feedforward, gain):
        """
        Initializes a Task object with the given parameters.

        Args:
            name (str): Title of the task.
            desired: Desired sigma (goal).
            feedforward: Feedforward component for the task.
            gain: Gain for the task.
        """
        self.name = name
        self.sigma_d = desired
        self.FeedForward = feedforward
        self.K = gain
        self.err_hist = []
        self.activation = 1

    def update(self, robot):
        """
        Abstract method for updating the task variables.

        Args:
            robot (Manipulator): Reference to the manipulator object.
        """
        pass

    def setDesired(self, value):
        """
        Sets the desired sigma for the task.

        Args:
            value: Value of the desired sigma (goal).
        """
        self.sigma_d = value

    def getDesired(self):
        """
        Gets the desired sigma for the task.

        Returns:
            numpy array: Desired sigma (goal).
        """
        return self.sigma_d

    def getJacobian(self):
        """
        Gets the task Jacobian.

        Returns:
            object: Task Jacobian.
        """
        return self.J

    def getError(self):
        """
        Gets the task error (tilde sigma).

        Returns:
            object: Task error.
        """
        return self.err

    def track_err(self):
        """ 
        Tracks the task error by appending it to the error history.
        """
        self.err_hist.append(self.getError())

    def setFeedForward(self, value):
        """
        Sets the feedforward component for the task.

        Args:
            value: Feedforward component value.
        """
        self.FeedForward = value

    def getFeedForward(self):
        """
        Gets the feedforward component for the task.

        Returns:
            object: Feedforward component value.
        """
        return self.FeedForward

    def setGain(self, value):
        """
        Sets the gain for the task.

        Args:
            value: Gain value.
        """
        self.K = value

    def getGain(self):
        """
        Gets the gain for the task.

        Returns:
            object: Gain value.
        """
        return self.K

    def isActive(self):
        """
        Checks if the task is active.

        Returns:
            int: Activation status (1 for active, 0 for inactive).
        """
        return self.activation

    def updateActivation(self, robot):
        """
        Abstract method for updating the activation status of the task.

        Args:
            robot (Manipulator): Reference to the manipulator object.
        """
        pass  # Overridden in child class

    def setActivation(self, value):
        """
        Sets the activation status of the task.

        Args:
            value: Activation status (1 for active, 0 for inactive).
        """
        self.activation = value



