clear all 
close all
clc

load("data_logging_20240523_test_ver1.mat")


dt = 0.0005;
T=8;

Time = dt*double(allData.SAVE_cnt_global);

time_length = size(Time,1);

leg_name{1} = "Rear Right";
leg_name{2} = "Rear Left";
leg_name{3} = "Front Right";
leg_name{4} = "Front Left";

%% Logging 
% LiDAR
sensors_lidar_odometry_t = allData.ros_lio_odometry_t;
sensors_lidar_odometry_R = allData.ros_lio_odometry_R;
lidar_update = allData.is_lidar_update;

for i=1:size(sensors_lidar_odometry_R,2)
    for j = 1:3
sensors_lidar_odometry_R_mat_temp(1:3,j) = sensors_lidar_odometry_R(3*(j-1)+1:3*(j-1)+3,i);
sensors_lidar_odometry_R_mat(1:3,j,i) = sensors_lidar_odometry_R(3*(j-1)+1:3*(j-1)+3,i);
    end
    sensors_lidar_odometry_R_eul(1:3,i) = rotm2eul(sensors_lidar_odometry_R_mat_temp(1:3,1:3),"xyz");
end

% Invariant Smoother
invariant_smoother_estimates_orientation = allData.x_estimated_X(1:9,:);
for i=1:size(invariant_smoother_estimates_orientation,2)
    for j = 1:3
invariant_smoother_estimates_orientation_mat_temp(1:3,j) = invariant_smoother_estimates_orientation(3*(j-1)+1:3*(j-1)+3,i);
invariant_smoother_estimates_orientation_mat(1:3,j,i) = invariant_smoother_estimates_orientation(3*(j-1)+1:3*(j-1)+3,i);
    end
    invariant_smoother_estimates_orientation_eul(1:3,i) = rotm2eul(invariant_smoother_estimates_orientation_mat_temp(1:3,1:3),"xyz");
end

invariant_smoother_estimates_position = allData.x_estimated_X(10:12,:);
invariant_smoother_estimates_angular_velocity = allData.x_estimated_X(13:15,:);
invariant_smoother_estimates_linear_velocity = allData.x_estimated_X(16:18,:);
invariant_smoother_estimates_contact = allData.x_estimated_contact(1:4,:);

% Encoders
sensors_joint_position = allData.sensor_data(7:18,:);
sensors_joint_velocity = allData.sensor_data(19:30,:);


% IMU
sensors_imu_gyro = allData.sensor_data(1:3,:);
sensors_imu_acc = allData.sensor_data(4:6,:);

% Controller - FSM
controller_fsm = allData.fsm_controller_output_FSM(1:4,:);

%% Removing bad initialization
time_init = 64001; %35001;
time_end = 300001;

Time = Time(1,time_init:time_end);
time_length = length(Time);

sensors_lidar_odometry_t = sensors_lidar_odometry_t(:,time_init:time_end);
sensors_lidar_odometry_R = sensors_lidar_odometry_R(:,time_init:time_end);
sensors_lidar_odometry_R_eul = sensors_lidar_odometry_R_eul(:,time_init:time_end);
lidar_update = lidar_update(:,time_init:time_end);

invariant_smoother_estimates_orientation = invariant_smoother_estimates_orientation(:,time_init:time_end);
invariant_smoother_estimates_orientation_eul = invariant_smoother_estimates_orientation_eul(:,time_init:time_end);
invariant_smoother_estimates_position = invariant_smoother_estimates_position(:,time_init:time_end);
invariant_smoother_estimates_angular_velocity = invariant_smoother_estimates_angular_velocity(:,time_init:time_end);
invariant_smoother_estimates_linear_velocity = invariant_smoother_estimates_linear_velocity(:,time_init:time_end);
invariant_smoother_estimates_contact = invariant_smoother_estimates_contact(:,time_init:time_end);

sensors_joint_position = sensors_joint_position(:,time_init:time_end);
sensors_joint_velocity = sensors_joint_velocity(:,time_init:time_end);

sensors_imu_gyro = sensors_imu_gyro(:,time_init:time_end);
sensors_imu_acc = sensors_imu_acc(:,time_init:time_end);

controller_fsm = controller_fsm(:,time_init:time_end);
%%
% 1 : LiDAR odometry position
% 2 : LiDAR odometry euler angles 
% 3 : Invariant Smoother - euler angles
% 4 : Invariant Smoother - position
% 5 : Invariant Smoother - angular_velocity
% 6 : Invariant Smoother - linear_velocity 
% 7 : Invariant Smoother - Contact
% 8 : joint position - linear_velocity 
% 9 : joint velocity - linear_velocity 
% 10 : imu - gyro
% 11 : imu - acc
% 12 : FSM - 
% e.g. when you want to check LiDAR odometry position and imu gyro : idx_plot = [1,10]
idx_plot = [1,2,3,4,5,6,7,8,9,10,11,12];


if(sum(ismember(idx_plot,1))>=1)

figure();
subplot(3,1,1)
plot(Time,sensors_lidar_odometry_t(1,:));
title("LiDAR Position X")

subplot(3,1,2)
plot(Time,sensors_lidar_odometry_t(2,:));
title("LiDAR Position Y")

subplot(3,1,3)
plot(Time,sensors_lidar_odometry_t(3,:));
title("LiDAR Position Z")

end


if(sum(ismember(idx_plot,2))>=1)

figure();
subplot(3,1,1)
plot(Time,180/pi*sensors_lidar_odometry_R_eul(1,:));
title("LiDAR Euler X")

subplot(3,1,2)
plot(Time,180/pi*sensors_lidar_odometry_R_eul(2,:));
title("LiDAR Euler Y")

subplot(3,1,3)
plot(Time,180/pi*sensors_lidar_odometry_R_eul(3,:));
title("LiDAR Euler Z")

end

if(sum(ismember(idx_plot,3))>=1)

figure();
plot(Time,lidar_update(1,:));
hold on;
title("LiDAR update")

end



if(sum(ismember(idx_plot,4))>=1)

figure();
subplot(3,1,1)
plot(Time,180/pi*invariant_smoother_estimates_orientation_eul(1,:));
title("IS Euler X")

subplot(3,1,2)
plot(Time,180/pi*invariant_smoother_estimates_orientation_eul(2,:));
title("IS Euler Y")

subplot(3,1,3)
plot(Time,180/pi*invariant_smoother_estimates_orientation_eul(3,:));
title("IS Euler Z")

end



if(sum(ismember(idx_plot,5))>=1)

figure();
subplot(3,1,1)
plot(Time,invariant_smoother_estimates_position(1,:));
title("IS Position X")

subplot(3,1,2)
plot(Time,invariant_smoother_estimates_position(2,:));
title("IS Position  Y")

subplot(3,1,3)
plot(Time,invariant_smoother_estimates_position(3,:));
title("IS Position  Z")

end


if(sum(ismember(idx_plot,6))>=1)

figure();
subplot(3,1,1)
plot(Time,invariant_smoother_estimates_angular_velocity(1,:));
title("IS Angular Velocity X")

subplot(3,1,2)
plot(Time,invariant_smoother_estimates_angular_velocity(2,:));
title("IS Angular Velocity  Y")

subplot(3,1,3)
plot(Time,invariant_smoother_estimates_angular_velocity(3,:));
title("IS Angular Velocity  Z")

end


if(sum(ismember(idx_plot,7))>=1)

figure();
subplot(3,1,1)
plot(Time,invariant_smoother_estimates_linear_velocity(1,:));
title("IS Linear Velocity X")

subplot(3,1,2)
plot(Time,invariant_smoother_estimates_linear_velocity(2,:));
title("IS Linear Velocity  Y")

subplot(3,1,3)
plot(Time,invariant_smoother_estimates_linear_velocity(3,:));
title("IS Linear Velocity  Z")

end



if(sum(ismember(idx_plot,8))>=1)

figure();
subplot(4,1,1)
plot(Time,invariant_smoother_estimates_contact(1,:));
title("Contact Estimates Rear Right leg")

subplot(4,1,2)
plot(Time,invariant_smoother_estimates_contact(2,:));
title("Contact Estimates Rear Left leg")

subplot(4,1,3)
plot(Time,invariant_smoother_estimates_contact(3,:));
title("Contact Estimates Front Right leg")

subplot(4,1,4)
plot(Time,invariant_smoother_estimates_contact(4,:));
title("Contact Estimates Front Left leg")

end




if(sum(ismember(idx_plot,9))>=1)

figure();

for idx_leg = 1 : 4
subplot(4,1,idx_leg)
plot(Time,sensors_joint_position(3*(idx_leg-1)+1,:));
hold on;
plot(Time,sensors_joint_position(3*(idx_leg-1)+2,:));
plot(Time,sensors_joint_position(3*(idx_leg-1)+3,:));
title(leg_name(idx_leg));
end

end




if(sum(ismember(idx_plot,10))>=1)

figure();

for idx_leg = 1 : 4
subplot(4,1,idx_leg)
plot(Time,sensors_joint_velocity(3*(idx_leg-1)+1,:));
hold on;
plot(Time,sensors_joint_velocity(3*(idx_leg-1)+2,:));
plot(Time,sensors_joint_velocity(3*(idx_leg-1)+3,:));
title(leg_name(idx_leg));
end

end




if(sum(ismember(idx_plot,11))>=1)

figure();
subplot(3,1,1)
plot(Time,sensors_imu_gyro(1,:));
title("IMU GYRO X")

subplot(3,1,2)
plot(Time,sensors_imu_gyro(2,:));
title("IMU GYRO Y")

subplot(3,1,3)
plot(Time,sensors_imu_gyro(3,:));
title("IMU GYRO Z")

end



if(sum(ismember(idx_plot,12))>=1)

figure();
subplot(3,1,1)
plot(Time,sensors_imu_acc(1,:));
title("IMU ACC X")

subplot(3,1,2)
plot(Time,sensors_imu_acc(2,:));
title("IMU ACC Y")

subplot(3,1,3)
plot(Time,sensors_imu_acc(3,:));
title("IMU ACC Z")

end



if(sum(ismember(idx_plot,13))>=1)

figure();
plot(Time,controller_fsm(1,:));
hold on;
plot(Time,controller_fsm(2,:));
plot(Time,controller_fsm(3,:));
plot(Time,controller_fsm(4,:));
title("FSM")

end

%% Proprioceptive measurements

imu_ang_vel = [sensors_imu_gyro'];
imu_lin_acc = [sensors_imu_acc'];

joint_pos = [sensors_joint_position'];
joint_vel = [sensors_joint_velocity'];

contacts = [invariant_smoother_estimates_contact'];

proprio_measurements = [imu_ang_vel imu_lin_acc joint_pos joint_vel contacts];
writematrix(proprio_measurements, 'proprio_measurements.csv')

%% Lidar odometry

lidar_position = [sensors_lidar_odometry_t'];
lidar_rot_mat = [sensors_lidar_odometry_R'];


lidar_position_x = lidar_position(1:end,1);
lidar_position_y = lidar_position(1:end,2);
lidar_position_z = lidar_position(1:end,3);

lidar_position_x = lidar_position_x - lidar_position_x(1,1);
lidar_position_y = lidar_position_y - lidar_position_y(1,1);
lidar_position_z = lidar_position_z - lidar_position_z(1,1);

lidar_position =  [lidar_position_x lidar_position_y lidar_position_z];

lid_quat = [];
for i=1:length(lidar_rot_mat)
    lid_quat(i,:) = rotm2quat([lidar_rot_mat(i,1) lidar_rot_mat(i,2) lidar_rot_mat(i,3); ...
                               lidar_rot_mat(i,4) lidar_rot_mat(i,5) lidar_rot_mat(i,6); ...
                               lidar_rot_mat(i,7) lidar_rot_mat(i,8) lidar_rot_mat(i,9)]);
end

lidar_odometry = [lidar_position lidar_rot_mat];
writematrix(lidar_odometry,"lidar_odometry.csv")

%% Proprioceptive and Exteroceptive "measurements"

measurements = [proprio_measurements lidar_odometry];
writematrix(measurements,"measurements.csv")

%% Saving Proprioceptive IS

is_position_x = invariant_smoother_estimates_position(1,1:end)';
is_position_y = invariant_smoother_estimates_position(2,1:end)';
is_position_z = invariant_smoother_estimates_position(3,1:end)';

is_velocity_x = invariant_smoother_estimates_linear_velocity(1,1:end)';
is_velocity_y = invariant_smoother_estimates_linear_velocity(2,1:end)';
is_velocity_z = invariant_smoother_estimates_linear_velocity(3,1:end)';

is_position = [is_position_x is_position_y is_position_z];
is_velocity = [is_velocity_x is_velocity_y is_velocity_z];

writematrix(is_velocity,"is_proprio_velocity.csv")

%%
est_rot = invariant_smoother_estimates_orientation';
est_quat = [];
for i=1:length(est_rot)
    est_quat(i,:) = rotm2quat([est_rot(i,1) est_rot(i,2) est_rot(i,3); ...
                               est_rot(i,4) est_rot(i,5) est_rot(i,6); ...
                               est_rot(i,7) est_rot(i,8) est_rot(i,9)]);
end
  
is_pose = [is_position est_quat];
writematrix(is_pose,"is_proprio_pose.csv")

%%
t_lid2base = [0.355, 0.0, 0.103];

%% 3D plot
gcf = figure('Position', get(0, 'Screensize'));
figure(13); %3
title('X-Y Trajectory')
hold on 
grid on
grid minor
plot3(invariant_smoother_estimates_position(1,1:end)-invariant_smoother_estimates_position(1,1),invariant_smoother_estimates_position(2,1:end)-invariant_smoother_estimates_position(2,1),invariant_smoother_estimates_position(3,1:end)-invariant_smoother_estimates_position(3,1))
plot3(sensors_lidar_odometry_t(1,1:end)-sensors_lidar_odometry_t(1,1),sensors_lidar_odometry_t(2,1:end)-sensors_lidar_odometry_t(2,1),sensors_lidar_odometry_t(3,1:end)-sensors_lidar_odometry_t(3,1))
legend('IS','LiDAR','Location','northeast')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

%% Position
gcf = figure('Position', get(0, 'Screensize'));
figure(14);
subplot(3,1,1) %
title('Position')
hold on 
grid on
grid minor
plot(invariant_smoother_estimates_position(1,1:end)-invariant_smoother_estimates_position(1,1))
plot(sensors_lidar_odometry_t(1,1:end)-sensors_lidar_odometry_t(1,1))
legend('IS-proprio','LiDAR','Location','NorthEast')
ylabel('x [m]')
subplot(3,1,2) %
hold on 
grid on
grid minor
plot(invariant_smoother_estimates_position(2,1:end)-invariant_smoother_estimates_position(2,1))
plot(sensors_lidar_odometry_t(2,1:end)-sensors_lidar_odometry_t(2,1))
ylabel('y [m]')
subplot(3,1,3) %
hold on 
grid on
grid minor
plot(invariant_smoother_estimates_position(3,1:end)-invariant_smoother_estimates_position(3,1))
plot(sensors_lidar_odometry_t(3,1:end)-sensors_lidar_odometry_t(3,1))
xlabel('samples')
ylabel('z [m]')


