import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Class to perform Weighted Least Squares (WLS) estimation
class WLS_estimator:
    def __init__(self, file_name,i,j):
        # Constructor code here
        self.file_name = file_name
        self.i = i
        self.j = j
        
    def get_train_values(self):
        # Outputs the values for the selected i and j
        self.raw_data = pd.read_csv(self.file_name)
        self.data = self.raw_data[(self.raw_data['i'] == self.i) & (self.raw_data['j'] == self.j)]
        self.raw_data = pd.read_csv(self.file_name)
        sensor_readings=[]
        target_values=[]
        for index, row in self.data.iterrows():
            sensor_readings.append(row[['velocity_r', 'velocity_l']].values)  # Take velocity_r and velocity_l
            target_values.append(row[['beta_r','beta_l','alpha']].values)  # 
        self.sensor_readings_array=np.array(sensor_readings)
        self.target_values_array=np.array(target_values)
        return self.sensor_readings_array, self.target_values_array
    
    def wls_operation(self,X, R, y):
        """Perform WLS to predict the parameter."""
        # Compute the weights
        W = np.linalg.inv(R)  # Weight matrix is the inverse of the covariance matrix
        X_T_W_X = X.T @ W @ X
        X_T_W_y = X.T @ W @ y
        # Compute the WLS estimate
        beta = np.linalg.inv(X_T_W_X) @ X_T_W_y
        return beta
    
    def apply_wls(self):
        
        np.random.seed(0)
        X, y = self.get_train_values()
        if X.shape[0] == 0:
                    print("No data found for the selected i and j")
                    return        # Covariance matrix of the measurement noise
        sigma_eps = np.random.rand(X.shape[0])*1e-1  # Random noise for each measurement
        R_eps = np.diag(sigma_eps ** 2)  # Covariance matrix for a single measurement
        R=R_eps
        # Number of measurements
        n_meas = X.shape[0]  
        beta = self.wls_operation(X, R, y)
        self.beta = beta
        print("predicted beta: ", beta)
        return beta
    
    def wls_predict(self, X):
        if X.shape[0] == 0:
            print("No data found for the selected i and j")
            return
        predictions = X @ self.beta
        # print("predictions: ", predictions)
        return predictions
    
    def compute_error(self, predictions, y):
        errors=[]

        if len(predictions.shape)>1:
            for i in range(len( self.sensor_readings_array)):
                errors.append(abs(predictions[i])-abs(y[i]))
            errors=np.array(errors)
            
            for i in range(len(predictions)):
                print(f"Predicted: {predictions[i]}, Actual: {y[i]}, Error: {errors[i]}")
                print(f"Percentage error: {abs(errors[i])/abs(y[i])*100}%")
        else:
            errors = predictions - y  # This would give the error as mentioned
            print(f"Predicted: {predictions}, Actual: {y}, Error: {errors}")
            # Compute the absolute error
            absolute_error = np.abs(errors)
            # Compute the percentage error
            percentage_error = (absolute_error / np.abs(y)) * 100

            print(f"Percentage error: {percentage_error}%")



class MapWLS:
    def __init__(self, map_height, map_width):
        self.map_height = map_height
        self.map_width = map_width
        self.wls_estimators = [[WLS_estimator('robot_data.csv', i, j) for j in range(map_height)] for i in range(map_width)]

    def check_map(self):
        for i in range(self.map_width):
            for j in range(self.map_height):
                print(f"i: {i}, j: {j}")
                wls = self.wls_estimators[i][j]
                wls.apply_wls()
                if wls.get_train_values()[0].shape[0] == 0:
                    continue
                
                predictions = wls.wls_predict(wls.get_train_values()[0][-1])
                wls.compute_error(predictions, wls.get_train_values()[1][-1])
                print("\n")
    def get_estimate(self, i, j,value):
        return self.wls_estimators[i][j].wls_predict(value)
    
    
    def plot_predictios(self,i,j):
        print("Plotting the predictions")
        wls = self.wls_estimators[i][j]
        X, y = wls.get_train_values()
        predictions = wls.wls_predict(X)
        target_values_array_test = y
        n_columns = 3  # since the array has 3 columns
        names=['beta_r','beta_l','alpha']
        plt.figure(figsize=(12, 8))
        for i in range(n_columns):
            plt.subplot(3, 1, i+1)  # Create a subplot for each column (3 rows, 1 column)
            plt.plot(predictions[:, i], label=f'{names[i]} Predicted', color='blue', linestyle='--')
            plt.plot(target_values_array_test[:, i], label=f'{names[i]} Target', color='red',linestyle='dotted')
            plt.xlabel('Sample')
            plt.ylabel(f'{names[i]} value')
            plt.title(f'{names[i]} Values')
            plt.legend()
        plt.tight_layout()  # Adjust layout to prevent overlapping
        plt.show() 
