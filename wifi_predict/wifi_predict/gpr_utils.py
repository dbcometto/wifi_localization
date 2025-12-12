from sklearn.gaussian_process import GaussianProcessRegressor
import numpy as np

class MultiGPR:
    """Wrap multiple GPRs (e.g., for x and y) into one object with a unified predict method."""

    def __init__(self, gps_list):
        self.gps_list = gps_list

    def predict(self, X):
        # Predict each GPR and stack as columns
        return np.column_stack([gp.predict(X) for gp in self.gps_list])


class ProgressGPR(GaussianProcessRegressor):
    def _constrained_optimization(self, obj_func, initial_theta, bounds):
        n_restarts = self.n_restarts_optimizer
        if n_restarts > 0:
            print(f"[INFO] Optimizing kernel with {n_restarts} restarts...")
        return super()._constrained_optimization(obj_func, initial_theta, bounds)