from sklearn.gaussian_process import GaussianProcessRegressor
import numpy as np

class MultiGPR:
    """Wrap multiple GPRs (e.g., for x and y) into one object with a unified predict method."""

    def __init__(self, gps_list):
        self.gps_list = gps_list  # list of GaussianProcessRegressor (one per output dimension)

    def predict(self, X, return_cov=False):
        """
        Predict with multiple GPRs.

        Args:
            X: array-like, shape (n_samples, n_features)
            return_cov: bool, if True, also return 2x2 covariance for each sample

        Returns:
            If return_cov=False: array, shape (n_samples, n_outputs)
            If return_cov=True: tuple (mean, covariances)
                - mean: shape (n_samples, n_outputs)
                - covariances: shape (n_samples, n_outputs, n_outputs)
        """
        n_samples = X.shape[0]
        n_outputs = len(self.gps_list)
        mean = np.zeros((n_samples, n_outputs))

        if return_cov:
            covs = np.zeros((n_samples, n_outputs, n_outputs))
            for i, gp in enumerate(self.gps_list):
                # Predict mean and variance
                mu, var = gp.predict(X, return_cov=False, return_std=True) if hasattr(gp, "predict") else gp.predict(X, return_std=True)
                mean[:, i] = mu
                covs[:, i, i] = var ** 2  # variance on the diagonal
            return mean, covs

        # Standard predict without covariance
        for i, gp in enumerate(self.gps_list):
            mean[:, i] = gp.predict(X)
        return mean


class ProgressGPR(GaussianProcessRegressor):
    """GPR with progress logging during kernel optimization."""

    def _constrained_optimization(self, obj_func, initial_theta, bounds):
        n_restarts = self.n_restarts_optimizer
        if n_restarts > 0:
            print(f"[INFO] Optimizing kernel with {n_restarts} restarts...")
        return super()._constrained_optimization(obj_func, initial_theta, bounds)
