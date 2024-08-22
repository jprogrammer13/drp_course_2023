import numpy as np
import pandas as pd
from scipy.linalg import block_diag


def local_first_estimate(data):
    data = data[(data.i == 0) & (data.j == 1)].reset_index()
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

    print("WLS Estimated coefficients:", beta_hat_wls)
    return F_0, a_0, beta_hat_wls

# msg = {"tractor0": {"F": [...], "a": [...]},
#            ...
#        "tractorn": {"F": [...], "a": [...]}
def global_estimate(msg):

    sum_F = np.zeros((9,9))
    sum_a = np.zeros((9,))

    for i in msg.keys():
        sum_F += msg[i]["F"]
        sum_a += msg[i]["a"]

    beta_hat_wls_global = np.linalg.inv(sum_F) @ sum_a
    
    return beta_hat_wls_global.reshape(3, -1).T