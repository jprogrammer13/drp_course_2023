import numpy as np
import pandas as pd
from scipy.linalg import block_diag


class Model():

    def __init__(self,  theta=np.zeros((3, 1))) -> None:
        self.theta = theta

    def predict(self, X):
        X = X[None]
        X = np.column_stack((np.ones(len(X)), X))
        return X @ self.theta


class WLSRegressor():
    def __init__(self, F=None, a=None, theta=np.zeros((3, 3))) -> None:

        self.F = F
        self.a = a
        self.theta = theta


class MapSlippageLocalWLSEstimator():

    def __init__(self, height, width, id) -> None:

        self.height = height
        self.width = width

        self.id = id

        self.map_wls_regressors = [
            [WLSRegressor() for j in range(width)] for i in range(height)]

    def local_first_estimate(self, data):
        # Example data
        omega_l = data.wheel_l.values
        omega_r = data.wheel_r.values
        beta_l = data.beta_l.values
        beta_r = data.beta_r.values
        alpha = data.alpha.values
        Y = np.column_stack((beta_l, beta_r, alpha))

        # Initial OLS regression to get residuals
        X = np.column_stack((np.ones(len(omega_l)), omega_l, omega_r))
        theta_hat_ols = np.linalg.inv(X.T @ X) @ (X.T @ Y)
        y_hat_ols = X @ theta_hat_ols
        residuals_ols = Y - y_hat_ols

        # Estimate weights as the inverse of the squared residuals
        weights_l = 1 / (residuals_ols[:, 0] ** 2 + 1e-10)
        weights_r = 1 / (residuals_ols[:, 1] ** 2 + 1e-10)
        weights_alpha = 1 / (residuals_ols[:, 2] ** 2 + 1e-10)

        # Create individual diagonal weight matrices for each output
        W_l = np.diag(weights_l)
        W_r = np.diag(weights_r)
        W_alpha = np.diag(weights_alpha)

        # Combine these into a block diagonal matrix
        W = block_diag(W_l, W_r, W_alpha)

        # Expand X to match the block diagonal structure
        # Kronecker product to create the expanded X matrix
        X_expanded = np.kron(np.eye(3), X)

        F_0 = X_expanded.T @ W @ X_expanded
        a_0 = X_expanded.T @ W @ Y.flatten(order='F')
        # Now compute WLS using the expanded X and Y
        XTWX_inv = np.linalg.inv(F_0)
        XTWy = a_0
        beta_hat_wls_flat = XTWX_inv @ XTWy

        # Reshape the result back to the original coefficient shape
        beta_hat_wls = beta_hat_wls_flat.reshape(3, -1).T

        # print("WLS Estimated coefficients:", beta_hat_wls)
        return F_0, a_0, beta_hat_wls

    def compute_wls_regressor(self, data):
        for i in range(self.width):
            for j in range(self.height):
                filtered = data[(data.i == i) & (data.j == j)]
                if len(filtered) > 3:
                    self.map_wls_regressors[i][j].F, self.map_wls_regressors[i][j].a, self.map_wls_regressors[i][j].theta = self.local_first_estimate(
                        filtered)

    def generate_msg(self):
        msg = {self.id: {}}
        for i in range(self.width):
            msg[self.id][i] = {}
            for j in range(self.height):
                msg[self.id][i][j] = {
                    "F": self.map_wls_regressors[i][j].F, "a": self.map_wls_regressors[i][j].a}

        return msg


class MapSlippageDistributedWLSEstimator():

    def __init__(self, height, width) -> None:

        self.height = height
        self.width = width

        self.map_wls_regressors = [
            [WLSRegressor() for j in range(width)] for i in range(height)]

    def global_first_estimate(self, msg):

        sum_F = np.zeros((9, 9))
        sum_a = np.zeros((9,))

        for i in msg.keys():
            if isinstance(msg[i]["F"], np.ndarray):
                sum_F += msg[i]["F"]
                sum_a += msg[i]["a"]

        if np.all(sum_F == 0.):
            beta_hat_wls_global = np.zeros((9, 1))
        else:
            beta_hat_wls_global = np.linalg.inv(sum_F) @ sum_a

        return beta_hat_wls_global.reshape(3, -1).T

    def compute_weights(self, msg):
        n_robots = len(msg.keys())
        connection_matrix = np.array([
            [0, 1, 0, 0, 1],
            [1, 0, 1, 0, 0],
            [0, 1, 0, 1, 0],
            [0, 0, 1, 0, 1],
            [1, 0, 0, 1, 0]
        ])

        q_ij = np.zeros((n_robots, n_robots))
        for i in range(n_robots):
            for j in range(n_robots):
                if i == j:
                    q_ij[i, j] = 1 - (2/(n_robots))
                else:
                    if i != j and connection_matrix[i, j] == 1:
                        q_ij[i, j] = 1/n_robots
                    else:
                        q_ij[i, j] = 0
        return q_ij

    def global_new_estimate(self, msg, robot_id, theta_init):

        sum_F_next = msg[robot_id]["F"]
        sum_a_next = msg[robot_id]["a"]
        q_ij = self.compute_weights(msg)
        robot_id_num = int(robot_id[-1])

        for i in msg.keys():
            if isinstance(msg[i]["F"], np.ndarray) and isinstance(msg[robot_id]["F"], np.ndarray):
                i_num = int(i[-1])
                sum_F_next += q_ij[robot_id_num, i_num] * \
                    (msg[i]["F"] - msg[robot_id]["F"])
                sum_a_next += q_ij[robot_id_num, i_num] * \
                    (msg[i]["a"] - msg[robot_id]["a"])

        if not (isinstance(msg[i]["F"], np.ndarray) and isinstance(msg[robot_id]["F"], np.ndarray)):
            beta_hat_wls_global = theta_init
        else:
            beta_hat_wls_global = np.linalg.inv(sum_F_next) @ sum_a_next
            beta_hat_wls_global = beta_hat_wls_global.reshape(3, -1).T
        return beta_hat_wls_global

    def compute_wls_regressor(self, msgs):
        for i in range(self.width):
            for j in range(self.height):
                patch_msg = {}
                for msg in msgs:
                    id = list(msg.keys())[0]
                    # print(id, i, j)
                    patch_msg[id] = msg[id][i][j]

                # print(list(patch_msg.keys()))
                self.map_wls_regressors[i][j].theta = self.global_first_estimate(
                    patch_msg)

    def compute_wls_new_estimate_regressor(self, msgs, robot_name):
        for i in range(self.width):
            for j in range(self.height):
                patch_msg = {}
                for msg in msgs:
                    id = list(msg.keys())[0]
                    # print(id, i, j)
                    patch_msg[id] = msg[id][i][j]

                # print(list(patch_msg.keys()))
                self.map_wls_regressors[i][j].theta = self.global_new_estimate(
                    patch_msg, robot_name, self.map_wls_regressors[i][j].theta)
