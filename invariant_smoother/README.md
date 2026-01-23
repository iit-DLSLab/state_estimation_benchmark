# Invariant smoother

## Prerequisites
- Eigen
- ROS Noetic (optional)

## To use the invariant smoother offline:
1. Clone the repo:

2. Build the package:
    ```
    cd invariant_smoother
    mkdir build
    cd build
    rm -rf *
    cmake ..
    make
    ```
3. Launch the script from the `build` folder:
   ```
   ./smoother_offline
    ```

To run the offline smoother you should save your data in the file measurements.csv in the path where you want to store your data, following this format:
| imu data          | joint states          | contact states| lidar odometry|
|-------------------|-----------------------|---------------|---------------|
| ang_vel - lin_acc | joint_pos - joint_vel | contact bool  | lid_pos - lid_quat - lid_update_bool|

Examples can be downloaded here:
- [slippage](https://drive.google.com/file/d/1IL7-E0HpRLwb7asQ-2M7M1kT2Ity8_LK/view?usp=drive_link): these data were collected using HOUND robot walking on slippery terrain.
- [outdoor](https://drive.google.com/file/d/1sYRbXb24iXZtXjRxGau31xw2_5jD3uY_/view?usp=drive_link): these data were collected using HOUND robot walking outdoor.
- [indoor with lidar](https://drive.google.com/file/d/1a0iN6gjHb41aEp_KLV6pQjTGmG3vAIfQ/view?usp=drive_link)

In the script [main.cpp](https://github.com/ylenianistico/invariant-smoother/blob/main/smoother_offline/src/main.cpp#L97) you will have to change the [path](https://github.com/ylenianistico/invariant-smoother/blob/main/smoother_offline/src/main.cpp#L97) where you store your data.

The script will save the estimated state in two files named estimate.csv (position & orientation) and velocity.csv (linear velocity). 
You can plot the results using the [plotting.py](https://github.com/ylenianistico/invariant-smoother/tree/main/smoother_offline/src/synchro_data) script.
If you want to benchmark your results, you can compare them with:
- slippage: invariant smoother running on the robot during experiments - [est_pose](https://drive.google.com/file/d/1y0pCAetfc86TaEmryaRV4HrB5GO2mawV/view?usp=drive_link) and [est_velocity](https://drive.google.com/file/d/1ZHckS29UPL5ee1guq0UCSXBhxT1c01A6/view?usp=drive_link)
- outdoor: invariant smoother running on the robot during experiments - [est_pose](https://drive.google.com/file/d/13fBW-4zvaeNzjm4ym9StShwCVg6tZw6O/view?usp=drive_link) and [est_velocity](https://drive.google.com/file/d/1NNLXuRTLCn3INMzxsO6ooeRVZV_BHi7f/view?usp=drive_link)
 


