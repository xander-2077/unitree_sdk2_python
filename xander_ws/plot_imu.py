import time
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

import unitree_legged_const as go2

# Initialize lists to hold the data for plotting
rpy_data = []
acc_data = []

# Set up the plot
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Initial plot settings
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-2, 2])
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.set_title('RPY and Accelerometer Direction Vectors')
line_rpy, = ax.plot([], [], [], label='RPY Vector', color='b', lw=2)
line_acc, = ax.plot([], [], [], label='Accelerometer Vector', color='g', lw=2)
ax.legend()

def LowStateHandler(msg: LowState_):
    global rpy_data, acc_data
    
    # Extract data
    rpy = msg.imu_state.rpy
    acc = msg.imu_state.accelerometer
    
    # Append data to lists
    rpy_data.append(rpy)
    acc_data.append(acc)
    
    # Limit the number of data points for visualization
    if len(rpy_data) > 100:
        rpy_data.pop(0)
        acc_data.pop(0)

    # Debug output
    print(f"Received data - RPY: {rpy}, Accelerometer: {acc}")
    
    plot_data()

def plot_data():
    global rpy_data, acc_data
    
    # Convert lists to numpy arrays for easier manipulation
    rpy_np = np.array(rpy_data)
    acc_np = np.array(acc_data)

    # Update the RPY vector
    if len(rpy_np) > 0:
        rpy_vector = rpy_np[-1]
        line_rpy.set_data([0, np.cos(rpy_vector[1])], [0, np.sin(rpy_vector[1])])
        line_rpy.set_3d_properties([0, rpy_vector[2]])  # Using the yaw as height

    # Update the Accelerometer vector
    if len(acc_np) > 0:
        acc_vector = acc_np[-1]
        line_acc.set_data([0, acc_vector[0]], [0, acc_vector[1]])
        line_acc.set_3d_properties([0, acc_vector[2]])  # Using the z-axis for height

    plt.draw()
    plt.pause(0.1)  # Allow for dynamic updates

if __name__ == "__main__":
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)
    
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    
    try:
        sub.Init(LowStateHandler, 10)
        print("Subscriber initialized successfully.")
        
        plt.show(block=True)  # Keep the plot window open
        while True:
            # plot_data()  # Call plot_data() regularly
            time.sleep(0.1)  # Adjust the sleep time as needed for smoother updates
    except Exception as e:
        print(f"An error occurred: {e}")
