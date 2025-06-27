# motion_predictor.py

import matplotlib.pyplot as plt
import numpy as np


class MotionPredictor:
    def __init__(self, prediction_horizon=10, delta_t=1.0):
        """
        Initialize the motion predictor.

        Args:
            prediction_horizon (int): Number of steps to predict into the future
            delta_t (float): Time step between frames (seconds)
        """
        self.prediction_horizon = prediction_horizon
        self.delta_t = delta_t
        self.history = {}  # Store positions and velocities for each object

    def update(self, object_id, position, velocity):
        """
        Update the history for a given object with new position and velocity.

        Args:
            object_id (int): Unique identifier for the object
            position (np.ndarray): Current position of the object [x, y]
            velocity (np.ndarray): Current velocity of the object [vx, vy]
        """
        self.history[object_id] = {"position": position, "velocity": velocity}

    def predict(self, object_id):
        """
        Predict the future trajectory of an object using the kinematic model.

        Args:
            object_id (int): Unique identifier for the object

        Returns:
            np.ndarray: Predicted positions for the next `prediction_horizon` steps
        """
        if object_id not in self.history:
            raise ValueError(f"Object {object_id} not found in history")

        # Retrieve the last known position and velocity
        position = self.history[object_id]["position"]
        velocity = self.history[object_id]["velocity"]

        # Generate predicted positions for the future
        predicted_positions = []
        for i in range(self.prediction_horizon):
            position = position + velocity * self.delta_t
            predicted_positions.append(position)

        return np.array(predicted_positions)

    def plot_trajectory(self, object_id, ax=None):
        """
        Plot the predicted trajectory for an object.

        Args:
            object_id (int): Unique identifier for the object
            ax (matplotlib.axes.Axes): Axes object to plot on (optional)
        """
        if ax is None:
            fig, ax = plt.subplots()

        # Get the predicted trajectory
        predicted_positions = self.predict(object_id)

        # Extract x and y coordinates for plotting
        x_pred = predicted_positions[:, 0]
        y_pred = predicted_positions[:, 1]

        # Plot the predicted trajectory
        ax.plot(x_pred, y_pred, label=f"Object {object_id} Prediction")
        ax.scatter(x_pred[0], y_pred[0], color="red", label=f"Object {object_id} Start")

        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")
        ax.set_title("Predicted Trajectories")
        ax.legend()
        plt.show()
