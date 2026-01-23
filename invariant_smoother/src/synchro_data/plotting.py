import pandas as pd
import matplotlib.pyplot as plt

# Read data from CSV file and specify column names
# IS with lidar
# df = pd.read_csv('/home/iit.local/ynistico/dls_ws_home/rosbags/FSC-Dataset/inekf_pose.csv', names=['x', 'y', 'z', 'q1', 'q2', 'q3', 'q4'])
# df2 = pd.read_csv('/home/iit.local/ynistico/dls_ws_home/rosbags/FSC-Dataset/inekf_vel.csv', names=['x', 'y', 'z'])
# df3 = pd.read_csv('/home/iit.local/ynistico/dls_ws_home/rosbags/FSC-Dataset/is_pose_ws15.csv', names=['x', 'y', 'z', 'q1', 'q2', 'q3', 'q4'])
# df4 = pd.read_csv('/home/iit.local/ynistico/dls_ws_home/rosbags/FSC-Dataset/gt_tum.csv', sep='\s+',  names=['dt','x', 'y', 'z', 'q1', 'q2', 'q3', 'q4'])

# df = pd.read_csv('/home/iit.local/ynistico/dls_ws_home/invariant_smoother_data/hound/indoor_new/new_indoor_synchro/linekf_pose.csv', names=['x', 'y', 'z', 'q1', 'q2', 'q3', 'q4'])

df = pd.read_csv('/home/iit.local/ynistico/dls_ws_home/invariant_smoother_data/hound/outdoor_with_gps/results_no_timestamp/accuracy/einekf_pose.csv', names=['x', 'y', 'z', 'q1', 'q2', 'q3', 'q4'])
# df2 = pd.read_csv('/home/iit.local/ynistico/dls_ws_home/invariant_smoother_data/hound/outdoor_with_gps/results_no_timestamp/lis_vel_lidar_15.csv', names=['x', 'y', 'z'])

# Vicon
# df3 = pd.read_csv('/home/iit.local/ynistico/dls_ws_home/invariant_smoother_data/hound/indoor_new/vicon_pose_indoor.csv', names=['t','x', 'y', 'z'])
# df3 = pd.read_csv('/home/iit.local/ynistico/dls_ws_home/invariant_smoother_data/hound/indoor_new/new_indoor_synchro/vicon_indoor_notime.csv', names=['x', 'y', 'z', 'q1', 'q2', 'q3', 'q4'])
# df4 = pd.read_csv('/home/iit.local/ynistico/dls_ws_home/invariant_smoother_data/hound/indoor_with_vicon/pos_rot/is_proprio_velocity.csv', names=['x', 'y', 'z'])

# GPS
df3 = pd.read_csv('/home/iit.local/ynistico/dls_ws_home/invariant_smoother_data/hound/outdoor_with_gps/gps_position.csv', names=['dt','x', 'y', 'z'])
# df4 = pd.read_csv('/home/iit.local/ynistico/dls_ws_home/invariant_smoother_data/hound/outdoor_with_gps/kiss_pose.csv', names=['dt','x', 'y', 'z', 'q1', 'q2', 'q3', 'q4'])


# Plot the data only if the DataFrame is not empty
# if not df.empty and not df2.empty:
if not df.empty:

    ####################### X-Y Trajectory #######################
    # Plot the data - indoor
    plt.plot(df['x'], df['y'], linestyle='-', label='inekf')
    plt.plot(df3['x'], df3['y'], linestyle='-', label='vicon')
    # plt.plot(df4['x'], df4['y'], linestyle='-', label='gt')
    # Add markers for starting and end points with larger size
    plt.scatter(df['x'].iloc[0], df['y'].iloc[0], color='red', label='inekf start', s=100)
    plt.scatter(df['x'].iloc[-1], df['y'].iloc[-1], color='green', label='inekf end', s=100)
    plt.scatter(df3['x'].iloc[0], df3['y'].iloc[0], color='blue', label='vicon start', s=100)
    plt.scatter(df3['x'].iloc[-1], df3['y'].iloc[-1], color='magenta', label='vicon end', s=100)

    # Plot the data - outdoor
    # plt.plot(df['x'], df['y'], linestyle='-', label='IS-LiDAR')
    # plt.plot(df3['x'], df3['y'], linestyle='-', label='GPS')
    # plt.plot(df4['x'], df4['y'], linestyle='-', label='KISS')
    # # Add markers for starting and end points with larger size
    # plt.scatter(df['x'].iloc[0], df['y'].iloc[0], color='red', label='IS-LiDAR start', s=100)
    # plt.scatter(df['x'].iloc[-1], df['y'].iloc[-1], color='green', label='IS-LiDAR end', s=100)
    # plt.scatter(df3['x'].iloc[0], df3['y'].iloc[0], color='blue', label='GPS start', s=100)
    # plt.scatter(df3['x'].iloc[-1], df3['y'].iloc[-1], color='magenta', label='GPS end', s=100)
    # plt.scatter(df4['x'].iloc[0], df4['y'].iloc[0], color='yellow', label='gt start', s=100)
    # plt.scatter(df4['x'].iloc[-1], df4['y'].iloc[-1], color='black', label='gt end', s=100)


    # Add labels and title
    plt.xlabel('X-axis [m]')
    plt.ylabel('Y-axis [m]')
    plt.legend()

    # plt.savefig('/home/ynistico/dls_ws_home/invariant_smoother_data/hound/indoor_with_lidar_kiss1/lid_obs_cost/trajectory.png')

    ####################### Position #######################
    # Plot the position data
    fig, (ax1, ax2,ax3) = plt.subplots(3, 1, figsize=(6, 4))

    # Plot x data - indoor
    ax1.set_title('Position')
    ax1.plot(df['x'], linestyle='-', label='x inekf')
    ax1.plot(df3['x'], linestyle='-', label='x vicon')
    ax1.set_xlabel('Iterations')
    ax1.set_ylabel('x [m]')

    # Plot y data - indoor
    ax2.plot(df['y'], linestyle='-', label='y inekf')
    ax2.plot(df3['y'], linestyle='-', label='y vicon')
    ax2.set_xlabel('Iterations')
    ax2.set_ylabel('y [m]')

    # Plot z data - indoor
    ax3.plot(df['z'], linestyle='-', label='z inekf')
    ax3.plot(df3['z'], linestyle='-', label='z vicon')
    ax3.set_xlabel('Iterations')
    ax3.set_ylabel('z [m]')

    # # Plot x data - outdoor
    # ax1.set_title('Position')
    # ax1.plot(df['x'], linestyle='-', label='x is_lidar')
    # ax1.plot(df3['x'], linestyle='-', label='x gps')
    # ax1.plot(df4['x'], linestyle='-', label='x kiss')
    # ax1.set_xlabel('Iterations')
    # ax1.set_ylabel('x [m]')

    # # Plot y data - outdoor
    # ax2.plot(df['y'], linestyle='-', label='y is_lidar')
    # ax2.plot(df3['y'], linestyle='-', label='y gps')
    # ax2.plot(df4['y'], linestyle='-', label='y kiss')
    # ax2.set_xlabel('Iterations')
    # ax2.set_ylabel('y [m]')

    # # Plot z data - outdoor
    # ax3.plot(df['z'], linestyle='-', label='z is_lidar')
    # ax3.plot(df3['z'], linestyle='-', label='z gps')
    # ax3.plot(df4['z'], linestyle='-', label='z kiss')
    # ax3.set_xlabel('Iterations')
    # ax3.set_ylabel('z [m]')

    # Add legend
    ax1.legend()
    ax2.legend()
    ax3.legend()

    # Adjust spacing between subplots
    plt.tight_layout()
    
    # Add legend
    plt.legend()


    # Get the range around the start and end points
    x_start, y_start = df['x'].iloc[0], df['y'].iloc[0]
    x_end, y_end = df['x'].iloc[-1], df['y'].iloc[-1]
    x_range = abs(x_end - x_start)
    y_range = abs(y_end - y_start)

    # plt.savefig('/home/ynistico/dls_ws_home/invariant_smoother_data/hound/indoor_with_lidar_kiss1/lid_obs_cost/position.png')

    ####################### Velocity #######################
    # Plot the velocity data
    # fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(6, 4))

    # # Plot x data
    # ax1.set_title('Linear Velocity')
    # # ax1.plot(df2['x'], linestyle='-', label='x inekf')
    # # ax1.plot(df4['x'], linestyle='-', label='x vicon')
    # ax1.set_xlabel('Iterations')
    # ax1.set_ylabel('x [m/s]')

    # # Plot y data
    # # ax2.plot(df2['y'], linestyle='-', label='y inekf')
    # # ax2.plot(df4['y'], linestyle='-', label='y vicon')
    # ax2.set_xlabel('Iterations')
    # ax2.set_ylabel('y [m/s]')

    # # Plot z data
    # # ax3.plot(df2['z'], linestyle='-', label='z inekf')
    # # ax3.plot(df4['z'], linestyle='-', label='z vicon')
    # ax3.set_xlabel('Iterations')
    # ax3.set_ylabel('z [m/s]')

    # # Add legend
    # ax1.legend()
    # ax2.legend()
    # ax3.legend()

    # # Add legend
    # plt.legend()

    # plt.savefig('/home/ynistico/dls_ws_home/invariant_smoother_data/hound/indoor_with_lidar_kiss1/lid_obs_cost/velocity.png')

    ################## Orientation ##################
    # Plot the orientation data
    # fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(6, 4))

    # # Plot q1 data 
    # ax1.set_title('Orientation')
    # ax1.plot(df['q1'], linestyle='-', label='q1 inekf')
    # ax1.plot(df3['q1'], linestyle='-', label='q1 vicon')
    # ax1.set_xlabel('Iterations')
    # ax1.set_ylabel('q1')

    # # Plot q2 data
    # ax2.plot(df['q2'], linestyle='-', label='q2 inekf')
    # ax2.plot(df3['q2'], linestyle='-', label='q2 vicon')
    # ax2.set_xlabel('Iterations')
    # ax2.set_ylabel('q2')

    # # Plot q3 data
    # ax3.plot(df['q3'], linestyle='-', label='q3 inekf')
    # ax3.plot(df3['q3'], linestyle='-', label='q3 vicon')
    # ax3.set_xlabel('Iterations')
    # ax3.set_ylabel('q3')

    # # Plot q4 data
    # ax4.plot(df['q4'], linestyle='-', label='q4 inekf')
    # ax4.plot(df3['q4'], linestyle='-', label='q4 vicon')
    # ax4.set_xlabel('Iterations')
    # ax4.set_ylabel('q4')

    # # Add legend
    # ax1.legend()
    # ax2.legend()
    # ax3.legend()
    # ax4.legend()

    # # Adjust spacing between subplots
    # plt.tight_layout()

    # plt.savefig('/home/ynistico/dls_ws_home/invariant_smoother_data/hound/indoor_with_lidar_kiss1/lid_obs_cost/orientation.png')

    # Show the plots
    plt.show()


else:
    print("DataFrame is empty. No data to plot.")