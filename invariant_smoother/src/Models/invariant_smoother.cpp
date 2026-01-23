#include <Models/invariant_smoother.hpp>

InvariantSmoother::InvariantSmoother(){};    // (ROBOT_STATES state) : state_(state) {};	
InvariantSmoother::~InvariantSmoother(){};

void InvariantSmoother::initialize
(
    double dt_, EstimatorCovariances estimator_covariances,
    Eigen::Matrix<double, 16, 1> &initial_condition
)
{
    sliding_window_flag = false;
    marginalization_flag = false;
    frame_count = 0;
    time_count = 0;

    estimator_common_struct_.variable_contact_cov_mode = variable_contact_cov_mode;
    estimator_common_struct_.cov_amplifier = cov_amplifier;
    estimator_common_struct_.lidar_covariance_amplifier = lidar_covariance_amplifier;
    estimator_common_struct_.slip_rejection_mode = slip_rejection_mode;
    estimator_common_struct_.slip_threshold = slip_threshold;
    estimator_common_struct_.long_term_v_threshold = long_term_v_threshold;
    estimator_common_struct_.long_term_a_threshold = long_term_a_threshold;
	estimator_common_struct_.estimator_covariances_ = estimator_covariances;
    estimator_common_struct_.dt = dt_;
    estimator_common_struct_.Covariance_Reset();

    dt = dt_;
    gravity << 0.0, 0.0, -9.80665;

    rs[0].Position = initial_condition.block(0, 0, 3, 1);
	rs[0].Velocity.setZero();
	initial_quaternion = initial_condition.block(3, 0, 4, 1);

	rs[0].Rotation = Quaternion_to_Rotation_Matrix(initial_quaternion);
	rs[0].Bias_Acc.setZero();
	rs[0].Bias_Gyro.setZero();

	rs[1].Position = initial_condition.block(0, 0, 3, 1);
	rs[1].Velocity.setZero();

	rs[1].Rotation = Quaternion_to_Rotation_Matrix(initial_quaternion);
	rs[1].Bias_Acc.setZero();
	rs[1].Bias_Gyro.setZero();

} // end initialize


void InvariantSmoother::new_measurement
(
    Eigen::Matrix<double, num_z, 1> sensor_i, 
    Eigen::Matrix<bool, 4, 1> contact_i,
    Eigen::Matrix<double,3,1> x_lidar_i,
    Eigen::Matrix<double,3,3> R_lidar_i,
    Eigen::Matrix<double,3,1> sqrt_lidar_i,
    bool lidar_in_i,
    Eigen::Matrix<double,3,1> x_gps_i,
    Eigen::Matrix<double,3,1> sqrt_gps_i,
    bool gps_in_i
)
{
    // measurements storage
    estimation_z.block(num_z * frame_count, 0, num_z, 1) << sensor_i.block(0, 0, 30, 1);
	rs[frame_count].Contact = contact_i;
    rs[frame_count].LiDAR_In = lidar_in_i;
    rs[frame_count].GPS_In = gps_in_i;
    
    // making state
    if (frame_count == 0)       // initializing states
    {
        for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
        {
	        rs[0].Slip(k) = false;
	        rs[0].Hard_Contact(k) = rs[0].Contact(k) - rs[0].Slip(k);
	    }

        // estimating foot position
	    for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
        {
	        rs[0].d.block(3 * k, 0, 3, 1) = rs[0].Position + 
                                            rs[0].Rotation * LeggedRobotKinematics::GetImu2FootPosition(k, estimation_z.block<3,1>(6 + 3 * k,0), 
                                                                                                        Robot::FootRadiusForEstimator, 
                                                                                                        estimator_common_struct_.IMU2BD);
        }

    }
    else
    {
        // inner timestep larger than 0
	    imu_gyro_prev = estimation_z.block(num_z * (frame_count - 1), 0, 3, 1);
	    imu_acc_prev = estimation_z.block(num_z * (frame_count - 1) + 3, 0, 3, 1);
	    imu_gyro = estimation_z.block(num_z * (frame_count), 0, 3, 1);
	    encoder = estimation_z.block(num_z * (frame_count) + 6, 0, 12, 1);
	    encoder_dot = estimation_z.block(num_z * (frame_count) + num_z_imu + num_z_encoder, 0, num_z_encoderdot, 1);

        // propagating state using imu and lidar
        rs[frame_count].Bias_Gyro << rs[frame_count - 1].Bias_Gyro;
	    rs[frame_count].Bias_Acc << rs[frame_count - 1].Bias_Acc;

	    rs[frame_count].Velocity = rs[frame_count - 1].Velocity + rs[frame_count - 1].Rotation * (imu_acc_prev - rs[frame_count - 1].Bias_Acc) * dt + gravity * dt;

        rs[frame_count].Rotation = rs[frame_count - 1].Rotation * Expm_Vec((imu_gyro_prev - rs[frame_count - 1].Bias_Gyro) * dt);
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(rs[frame_count].Rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
        rs[frame_count].Rotation = svd.matrixU() * svd.matrixV().transpose();
        	
        rs[frame_count].Position = rs[frame_count - 1].Position + rs[frame_count - 1].Velocity * dt
                                    + 0.5 * rs[frame_count - 1].Rotation * (imu_acc_prev - rs[frame_count - 1].Bias_Acc) * dt * dt
                                    + 0.5 * gravity * dt * dt;


        // estimating foot velocity
	    for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
        {
	        rs[frame_count].d_v.block(3 * k, 0, 3, 1) =   rs[frame_count].Velocity 
                                                        + rs[frame_count].Rotation * LeggedRobotKinematics::GetJacobian(k, encoder.block<3, 1>(3 * k, 0))
			                                            * encoder_dot.block<3, 1>(3 * k, 0)
                                                        + rs[frame_count].Rotation * Hat_so3(imu_gyro - rs[frame_count].Bias_Gyro)
			                                            * LeggedRobotKinematics::GetImu2FootPosition(k, encoder.block<3, 1>(3 * k, 0),
														                                             Robot::FootRadiusForEstimator,
                                                                                                     estimator_common_struct_.IMU2BD);                                                                                        

	        rs[frame_count].Slip(k) = false;

	        // slip rejection
	        if (slip_rejection_mode && rs[frame_count].Contact(k) && rs[frame_count].d_v.block(3 * k, 0, 3, 1).norm() > slip_threshold) 
            {
		        rs[frame_count].Slip(k) = true;
	        }
	        
            rs[frame_count].Hard_Contact(k) = rs[frame_count].Contact(k);
    	}

        // estimating foot position
        for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
        {
	        if (rs[frame_count].Hard_Contact(k) && rs[frame_count - 1].Hard_Contact(k)) 
            {
		        rs[frame_count].d.block(3 * k, 0, 3, 1) = rs[frame_count - 1].d.block(3 * k, 0, 3, 1);
	        }
            else 
            {
		        rs[frame_count].d.block(3 * k, 0, 3, 1) = rs[frame_count].Position + rs[frame_count].Rotation
			                                              * LeggedRobotKinematics::GetImu2FootPosition(k, encoder.block<3, 1>(3 * k, 0),
														                                               Robot::FootRadiusForEstimator, 
														                                               estimator_common_struct_.IMU2BD);
	        }
	    }
    }   // end if-else

    // contact number storage
    rs[frame_count].contact_leg_num = 0;
    for (int k = 0; k < 4; k++) 
    {
	    if (rs[frame_count].Hard_Contact(k) == true) 
        {
	        rs[frame_count].contact_leg_num++;
	    }
    }

    // state storage
    rs[frame_count].state_size = 6 + 15 + 3 * rs[frame_count].contact_leg_num; // why 33?
    if (frame_count == 0) 
    {
	    rs[frame_count].state_idx = 0;
    } 
    else 
    {
	    rs[frame_count].state_idx = rs[frame_count - 1].state_idx + rs[frame_count - 1].state_size;
    }

    // parameter storage
    rs[frame_count].para_size = 6 + 9 + 3 * rs[frame_count].contact_leg_num;    // why 27?
    if (frame_count == 0) 
    {
	    rs[frame_count].para_idx = 0;
    } 
    else 
    {
	    rs[frame_count].para_idx = rs[frame_count - 1].para_idx + rs[frame_count - 1].para_size;
    }

    // prior info storage
    if (frame_count == 0) 
    {
	    rs_prior = rs[0];
    }

    int i = frame_count - 1;
    int j = frame_count;

    if (frame_count > 0) 
    {
        int shared_contact = 0;
        for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
        {
            if (rs[i].Hard_Contact(k) && rs[j].Hard_Contact(k)) 
            {
                shared_contact = shared_contact + 1;
            }
        }
        fac_info[i].shared_contact = shared_contact;

        prop_para0_size = 9 + 3 * rs[i].contact_leg_num;
        prop_para1_size = 9 + 3 * rs[j].contact_leg_num;
        prop_res_size   = 9 + 3 * shared_contact + 6;
        
        fac_info[i].prop_para0_size = prop_para0_size;
	    fac_info[i].prop_para1_size = prop_para1_size;
	    fac_info[i].prop_res_size   = prop_res_size;

        // converting Xi_i to Xi size
        if (rs[i].contact_leg_num == fac_info[i].shared_contact) 
        {
	        Mi.resize(fac_info[i].prop_para0_size, fac_info[i].prop_para0_size);
	        Mi.setIdentity();
	    } 
        else 
        {
	        Mi.resize(9 + 3 * fac_info[i].shared_contact, fac_info[i].prop_para0_size);
	        Mi.setZero();
	        Mi.block(0, 0, 9, 9).setIdentity();

	        int omitted_leg_count = 0;
	        int count = 0;
	        for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
            {
		        if (rs[i].Hard_Contact(k)) 
                {
		            count++;
		            if (rs[i].Hard_Contact(k) == rs[j].Hard_Contact(k)) 
                    {
			            Mi.block(6 + 3 * (count - omitted_leg_count), 6 + 3 * count, 3, 3) << Eigen::Matrix3d::Identity();
		            } 
                    else 
                    {
			            omitted_leg_count++;
			            Mi.block(6 + 3 * (count - omitted_leg_count), 6 + 3 * count, 3, 3) << Eigen::Matrix3d::Zero();
		            }
		        }
	        }
        }
        fac_info[i].Mi.resize(fac_info[i].prop_para0_size, fac_info[i].prop_para0_size);
        fac_info[i].Mi = Mi;

        // converting original Xi_j (parameter 1) to common Xi size 
        if (rs[j].contact_leg_num == fac_info[i].shared_contact) 
        {
	        Mj.resize(fac_info[i].prop_para1_size, fac_info[i].prop_para1_size);
	        Mj.setIdentity();
	    } 
        else 
        {
	        Mj.resize(9 + 3 * fac_info[i].shared_contact, fac_info[i].prop_para1_size);
	        Mj.setZero();
	        Mj.block(0, 0, 9, 9).setIdentity();

	        int omitted_leg_count = 0;
	        int count = 0;
	        for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
            {
		        if (rs[j].Hard_Contact(k)) 
                {
		            count++;
		            if (rs[j].Hard_Contact(k) == rs[i].Hard_Contact(k)) 
                    {
			            Mj.block(6 + 3 * (count - omitted_leg_count), 6 + 3 * count, 3, 3) << Eigen::Matrix3d::Identity();
		            } 
                    else 
                    {
			            omitted_leg_count++;
			            Mj.block(6 + 3 * (count - omitted_leg_count), 6 + 3 * count, 3, 3) << Eigen::Matrix3d::Zero();
		            }
		        }
	        }
	    } // end if-else

	    fac_info[i].Mj = Mj;

        // propagation covariance
        sqrt_info_prop_primitive.resize(fac_info[i].prop_res_size, fac_info[i].prop_res_size);
	    sqrt_info_prop_primitive.setZero();
        sqrt_info_prop_primitive.block(6,6,3,3) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Gyro;
        sqrt_info_prop_primitive.block(9,9,3,3) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Acc;
        sqrt_info_prop_primitive.block(12,12,3,3) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Acc / (dt / sqrt(2));

        // contact covariance
        contact_cov_array_temp = estimator_common_struct_.Variable_Contact_Cov(rs[i].Hard_Contact, rs[i].d_v);
        int count = 0;
        for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
        {
	        if (rs[i].Hard_Contact(k) && rs[j].Hard_Contact(k)) 
            {	
		        sqrt_contact_info_temp.setZero();
		        sqrt_contact_info_temp.diagonal() << 1.0 / sqrt(contact_cov_array_temp[k](0)), 1.0 / sqrt(contact_cov_array_temp[k](1)), 1.0 / sqrt(contact_cov_array_temp[k](2));
		        sqrt_info_prop_primitive.block(15 + 3 * count,15 + 3 * count,3,3) = sqrt_contact_info_temp;
		        count++;
	        }
	    }
    	sqrt_info_prop_primitive.block(0,0,3,3) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Bias_Gyro;
	    sqrt_info_prop_primitive.block(3,3,3,3) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Bias_Acc;
        fac_info[i].prop_primitive_sqrt_info.resizeLike(sqrt_info_prop_primitive);
	    fac_info[i].prop_primitive_sqrt_info = sqrt_info_prop_primitive;

    } // end if frame_count > 0

    // measurements
    fac_info[j].meas_para_size = 9 + 3 * rs[j].contact_leg_num;
    fac_info[j].Z = estimation_z.block(num_z * j, 0, num_z, 1);
    fac_info[j].leg_info.clear();

    if (preintegration_mode)
    {
     	fac_info[j].z_buffer_.resize(z_buffer.size());
	    fac_info[j].z_buffer_ = z_buffer;   
        z_buffer.clear();
    }

    fac_info[j].X_LiDAR = x_lidar_i;
    fac_info[j].R_LiDAR = R_lidar_i;
    // fac_info[j].sqrt_LiDAR = sqrt(estimator_common_struct_.cov_amplifier * estimator_common_struct_.lidar_covariance_amplifier) * sqrt_lidar_i;
    fac_info[j].sqrt_LiDAR = sqrt_lidar_i;
    double rot_std_x = 0.01; // standard deviation for rotation about x-axis
    double rot_std_y = 0.01; // standard deviation for rotation about y-axis
    double rot_std_z = 0.01; // standard deviation for rotation about z-axis
    fac_info[j].sqrt_LiDAR_rot(0) = rot_std_x;
    fac_info[j].sqrt_LiDAR_rot(1) = rot_std_y;
    fac_info[j].sqrt_LiDAR_rot(2) = rot_std_z;

    fac_info[j].X_GPS = x_gps_i;
    fac_info[j].sqrt_GPS = sqrt_gps_i;


    int count = 0;
    for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
    {
	    if (rs[j].Hard_Contact(k)) 
        {
	        kinematics_info kin_k(k);
	        kin_k.leg_num_in_state = count;
	        enc = fac_info[j].Z.block(6 + 3 * k, 0, 3, 1);
	        kin_k.fk_kin = LeggedRobotKinematics::GetImu2FootPosition(k, enc, Robot::FootRadiusForEstimator,estimator_common_struct_.IMU2BD);
	        kin_k.meas_primitive_sqrt_info = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Encoder * LeggedRobotKinematics::GetJacobian(k, enc).inverse();
	        fac_info[j].leg_info.insert(kin_k);
	        count++;
        }
    }

    for (int p = 0; p <= frame_count; p++) 
    {
	    for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
        {
	        rs[p].d_v.block(3 * k, 0, 3, 1) = rs[p].Velocity + rs[p].Rotation * LeggedRobotKinematics::GetJacobian(k, estimation_z.block<3, 1>(num_z * p + 6 + 3 * k, 0))
			                                                                  * estimation_z.block<3, 1>(num_z * p + num_z_imu + num_z_encoder + 3 * k, 0)
		                                                                      + rs[p].Rotation * Hat_so3(estimation_z.block<3, 1>(num_z * p, 0) - rs[p].Bias_Gyro)
			                                                                  * LeggedRobotKinematics::GetImu2FootPosition(k, estimation_z.block<3, 1>(num_z * p + 6 + 3 * k, 0),
														                                                                   Robot::FootRadiusForEstimator, estimator_common_struct_.IMU2BD);
	    }
    }
    
    for (int p = 0; p < frame_count; p++) 
    {
	    contact_cov_array_temp = estimator_common_struct_.Variable_Contact_Cov(rs[p].Hard_Contact, rs[p].d_v);
	    int count = 0;
	    for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
        {
	        if (rs[p].Hard_Contact(k) && rs[p + 1].Hard_Contact(k)) 
            {
		        sqrt_contact_info_temp.setZero();
		        sqrt_contact_info_temp.diagonal() << 1.0 / sqrt(contact_cov_array_temp[k](0)), 1.0 / sqrt(contact_cov_array_temp[k](1)), 1.0 / sqrt(contact_cov_array_temp[k](2));
		        fac_info[p].prop_primitive_sqrt_info.block<3, 3>(15 + 3 * count, 15 + 3 * count) = sqrt_contact_info_temp;
		        count++;
	        }
	    }
    }

} // end new_measurement

void InvariantSmoother::optimization_solve()
{
    // cost_before = 0;
    // cost = 0;
    // total_backppgn_number = 0;
    // iteration_number = 0;

    gradient.resize(rs[frame_count].para_idx + rs[frame_count].para_size, 1);
    gradient.setZero();

    hessian.resize(rs[frame_count].para_idx + rs[frame_count].para_size, rs[frame_count].para_idx + rs[frame_count].para_size);
	hessian.setZero();

    delta_zeta_Xi.resizeLike(gradient);
    delta_zeta_Xi.setZero();

    perturbation.resizeLike(gradient);
    perturbation.setZero();

    InvFactors factor;

    factor.preintegration_mode_ = preintegration_mode;

    if (marginalization_flag) 
    {
        factor.marg_initialize(rs, estimation_z.block(0, 0, num_z * (frame_count + 1), 1),marginalized_H, 
                               marginalized_hessian, marginalized_b, marginalized_gradient, fac_info, estimator_common_struct_);

        factor.marg_update_nget_grad_hess_cost(0, rs, delta_zeta_Xi, hessian, gradient, cost, lidar_in_i, x_lidar_i, R_lidar_i);

        for (int iter = 1; iter <= max_iteration; iter++) 
        {
            cost_before = cost;
            perturbation = hessian.colPivHouseholderQr().solve(-gradient);

            if (perturbation.hasNaN())
            {
                perturbation.setZero();
            }
            if (iter>=1)
            {
                delta_zeta_Xi_temp.resizeLike(delta_zeta_Xi);
                ROBOT_STATES rs_temp[WINDOW_SIZE + 1];
                // cost_temp = 0;      // should not be 0
                t = ALPHA;
                // backpropagate_count = 0;

                while (true)
                {
                    perturbation = t * perturbation;

                    for (int k = 0; k <= WINDOW_SIZE; k++) 
                    {
			            rs_temp[k] = rs[k];
		            }

                    delta_zeta_Xi_temp = delta_zeta_Xi;
		            delta_zeta_Xi += perturbation;

                    if (retract_all_flag) 
                    {
			            retract_manifold(0);
		            } 
                    else if (marginalization_flag) 
                    {
			            retract_manifold(1);
		            } 
                    else 
                    {
			            retract_manifold(0);
		            }

                    factor.marg_update_nget_grad_hess_cost(false,rs,delta_zeta_Xi,hessian,gradient,cost_temp,lidar_in_i, x_lidar_i, R_lidar_i);
                    backpropagate_count++;
                    
                    if (max_backpropagate_num == 1000)
                    {
                        break;
                    }
                    else if (backpropagate_count > max_backpropagate_num)
                    {
                        if (delta_zeta_Xi.hasNaN())
                        {
                            delta_zeta_Xi.setZero();
                        }

                        for (int k = 0; k < WINDOW_SIZE; k++)
                        {
                            rs[k] = rs_temp[k];
                        }

                        break;                                              
                    }
                    else if (cost_temp > cost)
                    {
                        delta_zeta_Xi = delta_zeta_Xi_temp;
                        for (int k = 0; k < WINDOW_SIZE; k++)
                        {
                            rs[k] = rs_temp[k];
                        }
                        t = t*backppgn_rate;
                        cost_temp = 0;

                        if (backpropagate_count == 1)
                        {
                            total_backppgn_number++;
                        }                                              
                    }
                    else
                    {
                        break;
                    }                                                         
                }                
            }
            else
            {
                delta_zeta_Xi += perturbation;

                if (retract_all_flag)
                {
                    retract_manifold(0);
                }
                else if (marginalization_flag)
                {
                    retract_manifold(1);
                }
                else
                {
                    retract_manifold(0);
                }                
            }

            // update dv
            factor.marg_update_nget_grad_hess_cost(0,rs,delta_zeta_Xi,hessian,gradient,cost,lidar_in_i, x_lidar_i, R_lidar_i);

            if ((iter > 1) && (((sqrt(pow(cost - cost_before, 2)) / cost_before) < optimization_epsilon) || (iter == max_iteration)))
	        {
		        iteration_number = iter;
		        break;
	        } 
            else if (max_iteration == 1) 
            {
		        iteration_number = iter;		
		        break;
	        }            
        }

    }
    else
    {
        factor.batch_initialize(rs,rs_prior,estimation_z.block(0,0,num_z*(frame_count+1),1),fac_info,estimator_common_struct_);
        factor.batch_update_nget_grad_hess_cost(0,rs,hessian,gradient,cost,lidar_in_i, x_lidar_i, R_lidar_i);

        for (int iter = 1; iter <= max_iteration; iter++) 
        {
            cost_before = cost;
            delta_zeta_Xi = hessian.colPivHouseholderQr().solve(-gradient);

            if (iter >= 1) 
            {
                delta_zeta_Xi_temp.resizeLike(delta_zeta_Xi);
                ROBOT_STATES rs_temp[WINDOW_SIZE + 1];
                // cost_temp = 0;      // should not be 0
                t = ALPHA;
                // backpropagate_count = 0;              

                while (true) 
                {
                    delta_zeta_Xi = t * delta_zeta_Xi;
                    delta_zeta_Xi_temp = delta_zeta_Xi;

                    for (int k = 0; k <= WINDOW_SIZE; k++) 
                    {
			            rs_temp[k] = rs[k];
		            }
                    retract_manifold(0);

                    factor.batch_update_nget_grad_hess_cost(0,rs,hessian,gradient,cost_temp,lidar_in_i, x_lidar_i, R_lidar_i);
                    backpropagate_count++;

                    if (max_backpropagate_num == 1000) 
                    {
                        break;
                    }
                    else if (backpropagate_count >= max_backpropagate_num)
                    {
                        delta_zeta_Xi.setZero();

                        for (int k = 0; k <= WINDOW_SIZE; k++) 
                        {
			                rs[k] = rs_temp[k];
			            }
                        break;
                    }
                    else if (cost_temp > cost)
                    {
                        delta_zeta_Xi = delta_zeta_Xi_temp;

                        for (int k = 0; k <= WINDOW_SIZE; k++) 
                        {
			                rs[k] = rs_temp[k];
			            }

                        t = t * backppgn_rate;
                        cost_temp = 0;

			            if (backpropagate_count == 1) 
                        {
			                total_backppgn_number++;
			            }

                    }
                    else
                    {
                        break;
                    }
                }
            }
            else
            {
                retract_manifold(0);
            }

            // update dv(0)
            factor.batch_update_nget_grad_hess_cost(0,rs,hessian,gradient,cost_temp,lidar_in_i, x_lidar_i, R_lidar_i);

            if ((iter > 1) && (((sqrt(pow(cost - cost_before, 2)) / cost_before) < optimization_epsilon) || (iter == max_iteration)))
            {
		        iteration_number = iter;
		        break;
	        } 
            else if (max_iteration == 1) 
            {
		        iteration_number = iter;
		        break;
	        }
        }
    }

    // Schur complement method for marginalization
    if (frame_count == WINDOW_SIZE) 
    {
        SAVE_BUFFER[idx_cost][time_count] = cost;

        if (marginalization_flag) 
        {
            factor.marg_update_nget_grad_hess_cost(true, rs, delta_zeta_Xi, hessian, gradient, cost,lidar_in_i, x_lidar_i, R_lidar_i);
        }
        else
        {
            factor.batch_update_nget_grad_hess_cost(true, rs, hessian, gradient, cost, lidar_in_i, x_lidar_i, R_lidar_i);
        }

        m = rs[0].para_size;
        n = 0;

        for(int i=0; i < frame_count; i++)
	    {
	  		n += rs[i+1].para_size;
	    }

	    HMM.resize(rs[0].para_size, rs[0].para_size);
	    HMM.setZero();
	    HMM = hessian.block(0, 0, rs[0].para_size, rs[0].para_size);

	    HMR.resize(rs[0].para_size, rs[1].para_size);
	    HMR.setZero();
        HMR = hessian.block(0, rs[0].para_size, rs[0].para_size, rs[1].para_size);

	    HRR.resize(rs[1].para_size, rs[1].para_size);
	    HRR.setZero();
        HRR = hessian.block(rs[0].para_size, rs[0].para_size, rs[1].para_size, rs[1].para_size);

        marginalized_H.resize(rs[1].para_size, rs[1].para_size);
	    marginalized_H.setZero();
        marginalized_H = HRR - HMR.transpose() * HMM.colPivHouseholderQr().solve(HMR);
        
	    marginalized_hessian.resize(rs[1].para_size, rs[1].para_size);
	    marginalized_hessian.setZero();
        marginalized_hessian = marginalized_H;

        marginalized_b.resize(rs[1].para_size, 1);
	    marginalized_b.setZero();
        marginalized_b = gradient.block(rs[0].para_size, 0, rs[1].para_size, 1) - HMR.transpose() * HMM.colPivHouseholderQr().solve(gradient.block(0, 0, rs[0].para_size, 1));

	    marginalized_gradient.resize(rs[1].para_size, 1);
	    marginalized_gradient.setZero();

        marginalized_gradient = marginalized_b;

        if(!marginalized_hessian.allFinite())
        {
	        marginalized_hessian = marginalized_hessian_bef;
	        marginalized_H = marginalized_hessian;
	    }
	    
        if(!marginalized_gradient.allFinite())
	    {
	        marginalized_gradient = marginalized_gradient_bef;
	        marginalized_b = marginalized_gradient;
	    }

	    marginalized_hessian_bef = marginalized_hessian;
	    marginalized_gradient_bef = marginalized_gradient;

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(marginalized_H);
        s_vec = Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array(), 0));
		s_vec_inv = Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0));
	    s_vec_sqrt = s_vec.cwiseSqrt();
        s_vec_inv_sqrt = s_vec_inv.cwiseSqrt();

        marginalized_H = s_vec_sqrt.asDiagonal() * saes.eigenvectors().transpose();
	    marginalized_b = s_vec_inv_sqrt.asDiagonal() * saes.eigenvectors().transpose() * marginalized_b;
	    
        marginalization_flag = true;
    }

} // end optimization_solve

void InvariantSmoother::retract_manifold(int start_frame)
{
    for (int i = start_frame; i <= frame_count; i++) 
    {
        X_s.resize(5 + rs[i].contact_leg_num, 5 + rs[i].contact_leg_num);
        X_s.setIdentity();
        X_s.block<3, 3>(0, 0) = rs[i].Rotation;
	    X_s.block<3, 1>(0, 3) = rs[i].Velocity;
	    X_s.block<3, 1>(0, 4) = rs[i].Position;

        int count = 0;
        for (int k = 0; k < estimator_common_struct_.leg_no; k++)
        {
            if (rs[i].Hard_Contact(k))
            {
                X_s.block<3, 1>(0, 5 + count) = rs[i].d.block(3 * k, 0, 3, 1);
                count++;
            }
        }

        X_s = Expm_seK_Vec(delta_zeta_Xi.block(rs[i].para_idx + 6, 0, rs[i].para_size - 6, 1), 2 + rs[i].contact_leg_num) * X_s;
        	
        rs[i].Rotation = X_s.block<3, 3>(0, 0);
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(rs[i].Rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
	    rs[i].Rotation = svd.matrixU() * svd.matrixV().transpose();

        rs[i].Velocity = X_s.block<3, 1>(0, 3);
	    rs[i].Position = X_s.block<3, 1>(0, 4);

        count = 0;
        for (int k = 0; k < estimator_common_struct_.leg_no; k++)
        {
            if (rs[i].Hard_Contact(k)) 
            {
                rs[i].d.block(3 * k, 0, 3, 1) = X_s.block<3, 1>(0, 5 + count);
                count++;
            }  
        }

        rs[i].Bias_Gyro = rs[i].Bias_Gyro + delta_zeta_Xi.block(rs[i].para_idx, 0, 3, 1);
	    rs[i].Bias_Acc  = rs[i].Bias_Acc  + delta_zeta_Xi.block(rs[i].para_idx + 3, 0, 3, 1);

        delta_zeta_Xi.block(rs[i].para_idx, 0, rs[i].para_size, 1).setZero();

    }

} // end retract_manifold


void InvariantSmoother::update_dv(int start_frame)
{
    for (int i = start_frame; i <= frame_count; i++) 
    {
        for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
        {
            // estimating foot velocity
            rs[i].d_v.block(3 * k, 0, 3, 1) =   rs[i].Velocity 
                                              + rs[i].Rotation * LeggedRobotKinematics::GetJacobian(k, estimation_z.block<3, 1>(num_z * i + 6 + 3 * k, 0)) 
                                              * estimation_z.block<3, 1>(num_z * i + num_z_imu + num_z_encoder + 3 * k, 0)
		                                      + rs[i].Rotation * Hat_so3(estimation_z.block<3, 1>(num_z * i, 0) - rs[i].Bias_Gyro)
			                                  * LeggedRobotKinematics::GetImu2FootPosition(k, estimation_z.block<3, 1>(num_z * i + 6 + 3 * k, 0),
														                                   Robot::FootRadiusForEstimator, estimator_common_struct_.IMU2BD);

            if (slip_rejection_mode && rs[i].Contact(k) && rs[i].d_v.block(3 * k, 0, 3, 1).norm() > slip_threshold)
            {
                rs[i].Slip(k) = true;
            }
            else
            {
                rs[i].Slip(k) = false;
            }
        }
    }

}   // end update_dv


void InvariantSmoother::send_states(ROBOT_STATES &state_)
{
    w_pos_imu2bd = rs[frame_count].Rotation * estimator_common_struct_.IMU2BD;
    state_.Position = rs[frame_count].Position + w_pos_imu2bd;
    w_gyro = rs[frame_count].Rotation * (estimation_z.block(num_z * (frame_count), 0, 3, 1) - rs[frame_count].Bias_Gyro);

    state_.Velocity  = rs[frame_count].Velocity + w_gyro.cross(w_pos_imu2bd);
    state_.Bias_Gyro = rs[frame_count].Bias_Gyro;
    state_.Bias_Acc  = rs[frame_count].Bias_Acc;
    state_.Rotation  = rs[frame_count].Rotation;

    if (frame_count == 0) 
    {
	    state_.Hard_Contact = rs[frame_count].Hard_Contact;
	    state_.Contact = rs[frame_count].Contact;
	    state_.Slip = rs[frame_count].Slip;
	    state_.d = rs[frame_count].d;
	    state_.d_v = rs[frame_count].d_v;
    }
    else 
    {
	    state_.Hard_Contact = rs[frame_count - 1].Hard_Contact;
	    state_.Contact = rs[frame_count - 1].Contact;
	    state_.Slip = rs[frame_count - 1].Slip;
	    state_.d = rs[frame_count].d;
	    state_.d_v = rs[frame_count - 1].d_v;
    } 

    // return state_;

}   // end send_states


void InvariantSmoother::sliding_window()
{
    estimation_z.block(0, 0, num_z * WINDOW_SIZE, 1) = estimation_z.block(num_z, 0, num_z * WINDOW_SIZE, 1);
    delta_zeta_Xi.setZero();

    for (int i = 0; i < WINDOW_SIZE; i++) 
    {
        std::swap(rs[i], rs[i + 1]);

        if (i > 0) 
        {
           	rs[i].state_idx = rs[i - 1].state_idx + rs[i - 1].state_size;
	        rs[i].para_idx  = rs[i - 1].para_idx + rs[i - 1].para_size; 
        }
        else
        {
            rs[1].state_idx = rs[0].state_idx;
	        rs[0].state_idx = 0;
	        rs[1].para_idx = rs[0].para_idx;
	        rs[0].para_idx = 0;   
        }

        std::swap(fac_info[i], fac_info[i + 1]);
    }

} // end sliding_window


void InvariantSmoother::save_one_step(int cnt)
{
    if (cnt < SAVEMAXCNT) 
    {
        temp_eul = Rotation_to_EulerZYX(rs[frame_count].Rotation);
        for (int i = 0; i < 3; i++) 
        {
            SAVE_BUFFER[idx_ESTIMATED_Position + i][cnt] = rs[frame_count].Position(i);
            SAVE_BUFFER[idx_ESTIMATED_Velocity + i][cnt] = rs[frame_count].Velocity(i);
            SAVE_BUFFER[idx_ESTIMATED_Bias_Gyro + i][cnt] = rs[frame_count].Bias_Gyro(i);
            SAVE_BUFFER[idx_ESTIMATED_Bias_Acc + i][cnt] = rs[frame_count].Bias_Acc(i);
            SAVE_BUFFER[idx_ESTIMATED_rpy + i][cnt] = temp_eul(i);   
        }

        for (int i = 0; i < 3; i++) 
        {
            for (int j = 0; j < 3; j++) 
            {
                SAVE_BUFFER[idx_ESTIMATED_Rotation + i * 3 + j][cnt] = rs[frame_count].Rotation(i, j);
            }
        }

        for (int i = 0; i < estimator_common_struct_.leg_no; i++) 
        {
            for (int k = 0; k < 3; k++) 
            {
                if (cnt >= WINDOW_SIZE) 
                {
                    SAVE_BUFFER[idx_ESTIMATED_dv + 3 * i + k][cnt] = rs[frame_count].d_v(3 * i + k);
                }  
            }
        }

        for (int i = 0; i < estimator_common_struct_.leg_no; i++) 
        {
            SAVE_BUFFER[idx_ESTIMATED_Contact + i][cnt] = rs[frame_count].Contact(i);
            SAVE_BUFFER[idx_ESTIMATED_Slip + i][cnt] = rs[frame_count].Slip(i);
            SAVE_BUFFER[idx_ESTIMATED_Hard_Contact + i][cnt] = rs[frame_count].Hard_Contact(i); 
        }

        SAVE_BUFFER[idx_iteration_No][cnt] = iteration_number;
        SAVE_BUFFER[idx_backppgn_No][cnt] = total_backppgn_number;

    }
    else
    {
        cnt = SAVEMAXCNT - 1;
    }

} // end save_one_step


void InvariantSmoother::sensor_data_buffering(Eigen::Matrix<double, num_z, 1> sensor_i) 
{
    z_buffer.push_back(sensor_i);

} // end sensor_data_buffering


void InvariantSmoother::one_step
(
    Eigen::Matrix<double, num_z, 1> sensor_i, Eigen::Matrix<bool, 4, 1> contact_i, ROBOT_STATES &state_,
    Eigen::Matrix<double, 3, 1> x_lid_i, Eigen::Matrix<double,3,3> R_lid_i, Eigen::Matrix<double, 3, 1> sqrt_lid_i, bool lid_in_i,
    Eigen::Matrix<double, 3, 1> x_gps_i,	Eigen::Matrix<double, 3, 1> sqrt_gps_i, bool gps_in_i
)
{
    clock_t start, finish;
    start = clock();
    
    x_lidar_i = x_lid_i;
    R_lidar_i = R_lid_i;
    lidar_in_i = lid_in_i;

    x_gps_i = x_gps_i;
    gps_in_i = gps_in_i;

    // std::cout << "Smoother!" << std::endl;

    new_measurement(sensor_i, contact_i, x_lidar_i, R_lidar_i, sqrt_lid_i, lidar_in_i,x_gps_i, sqrt_gps_i, gps_in_i);

    if (frame_count > 0) 
    {
	    optimization_solve();
    }

    send_states(state_);

    save_one_step(time_count);

    frame_count++;

    if (frame_count > WINDOW_SIZE) 
    {
	    frame_count = WINDOW_SIZE;
	    sliding_window_flag = true;
    }

    // time_count++;

    if (sliding_window_flag) 
    {
	    sliding_window();
    }

    finish = clock();
    time_per_step = (double)(finish - start) / CLOCKS_PER_SEC;
    SAVE_BUFFER[idx_time_per_step][time_count] = time_per_step;

}   // end one_step

// ROBOT_STATES InvariantSmoother::getState() { 
//     return state_; 
// }